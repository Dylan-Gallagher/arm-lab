//! Predeclared cross-implementation RRT-Connect comparison.
//!
//! See `docs/oxmpl_baseline_protocol.md` for the frozen design and claim
//! boundaries. OxMPL is independently maintained; every returned path is
//! revalidated with arm-lab's collision checker before it counts as success.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use arm_lab::plan::{edge_free, path_length};
use arm_lab::{Chain, CollisionChecker};
use mujoco_rs::prelude::*;
use oxmpl::base::error::{PlanningError, StateSamplingError};
use oxmpl::base::goal::{Goal, GoalRegion, GoalSampleableRegion};
use oxmpl::base::planner::{Planner, PlannerConfig};
use oxmpl::base::problem_definition::ProblemDefinition;
use oxmpl::base::space::{RealVectorStateSpace, StateSpace};
use oxmpl::base::state::RealVectorState;
use oxmpl::base::validity::StateValidityChecker;
use oxmpl::geometric::RRTConnect;
use rand::Rng;
use sha2::{Digest, Sha256};

const ASSET_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ur5e/");
const QUERY_SHA256: &str = "c07b6227183a55e273120e37c15e86d99f717dc8f47b6a9d3d989d05415b5452";
const PLANNING_SHA256: &str = "b739009c0a4b3691bc910f67ad079a944e1b16c83d341bcd078179bda6c13055";
const EXPECTED_BLOCKED_QUERIES: usize = 217;
const REPLICATES: usize = 5;
const PLANNER_SEED_BASE: u64 = 202_608_180_000;
const MAX_DISTANCE_RAD: f64 = 0.25;
const GOAL_BIAS: f64 = 0.05;
const EDGE_RESOLUTION_RAD: f64 = 0.05;
const JOINT_LIMIT_INSET_RAD: f64 = 0.05;
const SOLVE_TIMEOUT: Duration = Duration::from_millis(250);
const ENDPOINT_TOLERANCE_RAD: f64 = 1e-9;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Scene {
    index: usize,
    name: &'static str,
    file: &'static str,
}

const SCENES: [Scene; 3] = [
    Scene {
        index: 0,
        name: "open_floor",
        file: "scene.xml",
    },
    Scene {
        index: 1,
        name: "offset_pillar",
        file: "scene_cluttered.xml",
    },
    Scene {
        index: 2,
        name: "tabletop_pillar",
        file: "scene_pickplace.xml",
    },
];

#[derive(Clone, Debug, PartialEq)]
struct QueryRow {
    scene: Scene,
    query_index: usize,
    start: Vec<f64>,
    goal: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct InHouseRow {
    success: bool,
    seed: u64,
}

#[derive(Debug)]
struct Inputs {
    queries: Vec<QueryRow>,
    in_house: BTreeMap<(usize, usize, usize), InHouseRow>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum OxmplStatus {
    Success,
    Timeout,
    PlanningError,
    InvalidPath,
}

impl OxmplStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::Success => "success",
            Self::Timeout => "timeout",
            Self::PlanningError => "planning_error",
            Self::InvalidPath => "invalid_path",
        }
    }

    fn parse(value: &str) -> Self {
        match value {
            "success" => Self::Success,
            "timeout" => Self::Timeout,
            "planning_error" => Self::PlanningError,
            "invalid_path" => Self::InvalidPath,
            _ => panic!("unknown OxMPL status: {value}"),
        }
    }

    fn succeeded(self) -> bool {
        self == Self::Success
    }
}

#[derive(Clone, Debug, PartialEq)]
struct ResultRow {
    scene: Scene,
    query_index: usize,
    replicate: usize,
    seed: u64,
    in_house_success: bool,
    oxmpl_status: OxmplStatus,
    oxmpl_detail: String,
    elapsed_ms: f64,
    waypoints: usize,
    path_cost_rad: f64,
    path_sha256: String,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct PairCounts {
    total: usize,
    in_house_success: usize,
    oxmpl_success: usize,
    both: usize,
    in_house_only: usize,
    oxmpl_only: usize,
    neither: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    Run,
    Write,
    Check,
    VerifyArtifacts,
}

fn main() {
    let mode = parse_mode();
    let inputs = load_inputs();

    if mode == Mode::VerifyArtifacts {
        verify_committed_artifacts(&inputs);
        return;
    }

    let rows = run_study(&inputs);
    validate_result_layout(&inputs, &rows);
    let csv = render_csv(&rows);
    let report = render_report(&rows);

    match mode {
        Mode::Run => print!("{report}"),
        Mode::Write => write_artifacts(&csv, &report),
        Mode::Check => check_artifacts(&csv, &report),
        Mode::VerifyArtifacts => unreachable!(),
    }
}

fn parse_mode() -> Mode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [] => Mode::Run,
        [arg] if arg == "--write" => Mode::Write,
        [arg] if arg == "--check" => Mode::Check,
        [arg] if arg == "--verify-artifacts" => Mode::VerifyArtifacts,
        _ => panic!("usage: oxmpl_baseline [--write|--check|--verify-artifacts]"),
    }
}

fn docs_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs")
}

fn load_inputs() -> Inputs {
    let docs = docs_dir();
    let query_bytes = std::fs::read(docs.join("randomized_eval_queries.csv"))
        .expect("read frozen randomized queries");
    let planning_bytes = std::fs::read(docs.join("randomized_eval_planning.csv"))
        .expect("read frozen randomized planning rows");
    assert_eq!(sha256_hex(&query_bytes), QUERY_SHA256, "query input hash");
    assert_eq!(
        sha256_hex(&planning_bytes),
        PLANNING_SHA256,
        "planning input hash"
    );

    let query_csv = std::str::from_utf8(&query_bytes).expect("query CSV UTF-8");
    let planning_csv = std::str::from_utf8(&planning_bytes).expect("planning CSV UTF-8");
    let queries = parse_queries(query_csv);
    let in_house = parse_in_house(planning_csv);

    assert_eq!(
        queries.len(),
        EXPECTED_BLOCKED_QUERIES,
        "frozen blocked-query count"
    );
    assert_eq!(
        in_house.len(),
        EXPECTED_BLOCKED_QUERIES * REPLICATES,
        "frozen paired in-repository trial count"
    );
    for query in &queries {
        for replicate in 0..REPLICATES {
            let key = (query.scene.index, query.query_index, replicate);
            let row = in_house.get(&key).expect("paired in-repository row");
            assert_eq!(
                row.seed,
                planner_seed(query.scene.index, query.query_index, replicate),
                "paired seed"
            );
        }
    }
    Inputs { queries, in_house }
}

fn parse_queries(csv: &str) -> Vec<QueryRow> {
    let mut lines = csv.lines();
    let header = lines.next().expect("query CSV header");
    assert_eq!(
        header,
        "scene,scene_file,query_index,generator_seed,accepted_draw_index,\
         cumulative_rejected_start_collision,cumulative_rejected_goal_collision,\
         cumulative_rejected_short,start_q_rad,goal_q_rad,distance_rad,direct_path_free"
    );
    let mut queries = Vec::new();
    for (line_index, line) in lines.enumerate() {
        let fields: Vec<_> = line.split(',').collect();
        assert_eq!(
            fields.len(),
            12,
            "query CSV line {} columns",
            line_index + 2
        );
        if parse_bool(fields[11]) {
            continue;
        }
        let scene = scene_from_fields(fields[0], fields[1]);
        queries.push(QueryRow {
            scene,
            query_index: parse(fields[2], "query index"),
            start: parse_vector(fields[8], "start"),
            goal: parse_vector(fields[9], "goal"),
        });
    }
    queries.sort_by_key(|row| (row.scene.index, row.query_index));
    queries
}

fn parse_in_house(csv: &str) -> BTreeMap<(usize, usize, usize), InHouseRow> {
    let mut lines = csv.lines();
    let header = lines.next().expect("planning CSV header");
    assert_eq!(
        header,
        "scene,scene_file,query_index,variant,replicate,seed,direct_path_free,status,\
         plan_elapsed_ms,iterations,nodes,shortcut_waypoints,path_samples,path_cost_rad"
    );
    let mut rows = BTreeMap::new();
    for (line_index, line) in lines.enumerate() {
        let fields: Vec<_> = line.split(',').collect();
        assert_eq!(
            fields.len(),
            14,
            "planning CSV line {} columns",
            line_index + 2
        );
        if fields[3] != "default" || parse_bool(fields[6]) {
            continue;
        }
        let scene = scene_from_fields(fields[0], fields[1]);
        let query_index = parse(fields[2], "planning query index");
        let replicate = parse(fields[4], "planning replicate");
        let seed = parse(fields[5], "planning seed");
        assert!(replicate < REPLICATES, "unexpected planning replicate");
        assert!(
            matches!(fields[7], "success" | "unconnected"),
            "unexpected in-repository status"
        );
        let key = (scene.index, query_index, replicate);
        assert!(
            rows.insert(
                key,
                InHouseRow {
                    success: fields[7] == "success",
                    seed,
                },
            )
            .is_none(),
            "duplicate in-repository row"
        );
    }
    rows
}

fn parse_vector(value: &str, label: &str) -> Vec<f64> {
    let values: Vec<f64> = value.split(';').map(|field| parse(field, label)).collect();
    assert_eq!(values.len(), 6, "{label} dimensions");
    assert!(
        values.iter().all(|value| value.is_finite()),
        "{label} finite"
    );
    values
}

fn parse_bool(value: &str) -> bool {
    match value {
        "true" => true,
        "false" => false,
        _ => panic!("invalid boolean: {value}"),
    }
}

fn parse<T: std::str::FromStr>(value: &str, label: &str) -> T {
    value
        .parse()
        .unwrap_or_else(|_| panic!("invalid {label}: {value}"))
}

fn scene_from_fields(name: &str, file: &str) -> Scene {
    SCENES
        .iter()
        .copied()
        .find(|scene| scene.name == name && scene.file == file)
        .unwrap_or_else(|| panic!("unknown scene/file pair: {name}/{file}"))
}

fn planner_seed(scene_index: usize, query_index: usize, replicate: usize) -> u64 {
    PLANNER_SEED_BASE + 10_000 * scene_index as u64 + 10 * query_index as u64 + replicate as u64
}

fn run_study(inputs: &Inputs) -> Vec<ResultRow> {
    let mut rows = Vec::with_capacity(EXPECTED_BLOCKED_QUERIES * REPLICATES);
    for scene in SCENES {
        let model = Arc::new(load_model(scene));
        let chain = Chain::from_mujoco(&model, "ur5e", "wrist_3_link", "attachment_site")
            .expect("extract UR5e chain");
        let bounds = planning_bounds(&chain);
        let mut space = RealVectorStateSpace::new(chain.dof(), Some(bounds.clone()))
            .expect("bounded OxMPL state space");
        let extent = space.get_maximum_extent();
        space.set_longest_valid_segment_fraction(0.5 / extent);
        assert!(
            (space.get_longest_valid_segment_length() * 0.1 - EDGE_RESOLUTION_RAD).abs() < 1e-12,
            "OxMPL edge resolution configuration"
        );
        let space = Arc::new(space);

        let scene_queries: Vec<_> = inputs
            .queries
            .iter()
            .filter(|query| query.scene == scene)
            .collect();
        println!(
            "{}: {} blocked queries, {} paired trials",
            scene.name,
            scene_queries.len(),
            scene_queries.len() * REPLICATES
        );
        for query in scene_queries {
            for replicate in 0..REPLICATES {
                let key = (scene.index, query.query_index, replicate);
                let in_house = inputs.in_house.get(&key).expect("paired row");
                let result = run_oxmpl_trial(
                    &model,
                    &chain,
                    Arc::clone(&space),
                    &bounds,
                    query,
                    replicate,
                    in_house,
                );
                println!(
                    "  query {:03} rep {}: in_house={} oxmpl={} ({:.2} ms)",
                    query.query_index,
                    replicate,
                    if in_house.success {
                        "success"
                    } else {
                        "failed"
                    },
                    result.oxmpl_status.as_str(),
                    result.elapsed_ms
                );
                rows.push(result);
            }
        }
    }
    rows
}

fn load_model(scene: Scene) -> MjModel {
    let path = format!("{ASSET_DIR}{}", scene.file);
    MjModel::from_xml(&path).unwrap_or_else(|error| panic!("load {}: {error}", scene.file))
}

fn planning_bounds(chain: &Chain) -> Vec<(f64, f64)> {
    chain
        .joint_limits()
        .into_iter()
        .map(|limit| {
            let (lower, upper) = limit.unwrap_or((-3.0, 3.0));
            let inset = (lower + JOINT_LIMIT_INSET_RAD, upper - JOINT_LIMIT_INSET_RAD);
            assert!(inset.0 < inset.1, "joint limit survives inset");
            inset
        })
        .collect()
}

#[derive(Clone)]
struct PointGoal {
    target: RealVectorState,
}

impl Goal<RealVectorState> for PointGoal {
    fn is_satisfied(&self, state: &RealVectorState) -> bool {
        l2(&state.values, &self.target.values) <= ENDPOINT_TOLERANCE_RAD
    }
}

impl GoalRegion<RealVectorState> for PointGoal {
    fn distance_goal(&self, state: &RealVectorState) -> f64 {
        l2(&state.values, &self.target.values)
    }
}

impl GoalSampleableRegion<RealVectorState> for PointGoal {
    fn sample_goal(&self, _rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
        Ok(self.target.clone())
    }
}

struct MujocoValidity {
    bounds: Vec<(f64, f64)>,
    checker: Mutex<CollisionChecker<Arc<MjModel>>>,
}

impl StateValidityChecker<RealVectorState> for MujocoValidity {
    fn is_valid(&self, state: &RealVectorState) -> bool {
        if !numeric_state_is_valid(&state.values, &self.bounds) {
            return false;
        }
        !self
            .checker
            .lock()
            .expect("OxMPL collision checker lock")
            .collides(&state.values)
    }
}

fn run_oxmpl_trial(
    model: &Arc<MjModel>,
    chain: &Chain,
    space: Arc<RealVectorStateSpace>,
    bounds: &[(f64, f64)],
    query: &QueryRow,
    replicate: usize,
    in_house: &InHouseRow,
) -> ResultRow {
    let seed = planner_seed(query.scene.index, query.query_index, replicate);
    assert_eq!(seed, in_house.seed, "paired seed before OxMPL run");
    let goal = Arc::new(PointGoal {
        target: RealVectorState::new(query.goal.clone()),
    });
    let problem = Arc::new(ProblemDefinition {
        space: Arc::clone(&space),
        start_states: vec![RealVectorState::new(query.start.clone())],
        goal,
    });
    let validity = Arc::new(MujocoValidity {
        bounds: bounds.to_vec(),
        checker: Mutex::new(CollisionChecker::new(Arc::clone(model), chain)),
    });
    let mut planner = RRTConnect::new(
        MAX_DISTANCE_RAD,
        GOAL_BIAS,
        &PlannerConfig { seed: Some(seed) },
    );
    planner.setup(problem, validity);
    let start_time = Instant::now();
    let solved = planner.solve(SOLVE_TIMEOUT);
    let elapsed_ms = start_time.elapsed().as_secs_f64() * 1e3;

    let (oxmpl_status, oxmpl_detail, waypoints, path_cost_rad, path_sha256) = match solved {
        Ok(path) => {
            let path: Vec<Vec<f64>> = path.0.into_iter().map(|state| state.values).collect();
            let waypoints = path.len();
            let cost = path_length(&path);
            let digest = digest_path(&path);
            let mut validation_checker = CollisionChecker::new(Arc::clone(model), chain);
            let detail = validate_path(&path, &query.start, &query.goal, bounds, |q| {
                validation_checker.collides(q)
            });
            match detail {
                Ok(()) => (
                    OxmplStatus::Success,
                    "validated".to_string(),
                    waypoints,
                    cost,
                    digest,
                ),
                Err(detail) => (OxmplStatus::InvalidPath, detail, waypoints, cost, digest),
            }
        }
        Err(PlanningError::Timeout) => (
            OxmplStatus::Timeout,
            "solve_timeout".to_string(),
            0,
            0.0,
            String::new(),
        ),
        Err(error) => (
            OxmplStatus::PlanningError,
            planning_error_name(&error).to_string(),
            0,
            0.0,
            String::new(),
        ),
    };

    ResultRow {
        scene: query.scene,
        query_index: query.query_index,
        replicate,
        seed,
        in_house_success: in_house.success,
        oxmpl_status,
        oxmpl_detail,
        elapsed_ms,
        waypoints,
        path_cost_rad,
        path_sha256,
    }
}

fn planning_error_name(error: &PlanningError) -> &'static str {
    match error {
        PlanningError::Timeout => "solve_timeout",
        PlanningError::NoSolutionFound => "no_solution_found",
        PlanningError::PlannerUninitialised => "planner_uninitialised",
        PlanningError::InvalidStartState => "invalid_start_state",
        PlanningError::UnsampledStateSpace => "unsampled_state_space",
    }
}

fn numeric_state_is_valid(values: &[f64], bounds: &[(f64, f64)]) -> bool {
    values.len() == bounds.len()
        && values.iter().zip(bounds).all(|(value, (lower, upper))| {
            value.is_finite() && *value >= *lower && *value <= *upper
        })
}

fn validate_path(
    path: &[Vec<f64>],
    start: &[f64],
    goal: &[f64],
    bounds: &[(f64, f64)],
    mut collides: impl FnMut(&[f64]) -> bool,
) -> Result<(), String> {
    if path.is_empty() {
        return Err("empty_path".to_string());
    }
    for state in path {
        if state.len() != bounds.len() {
            return Err("invalid_dimension".to_string());
        }
        if state.iter().any(|value| !value.is_finite()) {
            return Err("non_finite_state".to_string());
        }
        if !numeric_state_is_valid(state, bounds) {
            return Err("state_out_of_bounds".to_string());
        }
    }
    if l2(&path[0], start) > ENDPOINT_TOLERANCE_RAD {
        return Err("start_mismatch".to_string());
    }
    if l2(path.last().expect("non-empty path"), goal) > ENDPOINT_TOLERANCE_RAD {
        return Err("goal_mismatch".to_string());
    }
    if collides(&path[0]) {
        return Err("start_collision".to_string());
    }
    let mut scratch = vec![0.0; bounds.len()];
    for segment in path.windows(2) {
        if !edge_free(
            &segment[0],
            &segment[1],
            EDGE_RESOLUTION_RAD,
            &mut collides,
            &mut scratch,
        ) {
            return Err("edge_collision".to_string());
        }
    }
    Ok(())
}

fn l2(a: &[f64], b: &[f64]) -> f64 {
    if a.len() != b.len() {
        return f64::INFINITY;
    }
    a.iter()
        .zip(b)
        .map(|(left, right)| (left - right).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn digest_path(path: &[Vec<f64>]) -> String {
    let mut hasher = Sha256::new();
    hasher.update((path.len() as u64).to_le_bytes());
    for state in path {
        hasher.update((state.len() as u64).to_le_bytes());
        for value in state {
            hasher.update(value.to_bits().to_le_bytes());
        }
    }
    format_digest(hasher.finalize())
}

fn sha256_hex(bytes: &[u8]) -> String {
    format_digest(Sha256::digest(bytes))
}

fn format_digest(digest: impl AsRef<[u8]>) -> String {
    digest
        .as_ref()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn validate_result_layout(inputs: &Inputs, rows: &[ResultRow]) {
    assert_eq!(
        rows.len(),
        EXPECTED_BLOCKED_QUERIES * REPLICATES,
        "result row count"
    );
    let mut keys = BTreeSet::new();
    for row in rows {
        assert!(row.replicate < REPLICATES, "result replicate range");
        let key = (row.scene.index, row.query_index, row.replicate);
        assert!(keys.insert(key), "duplicate result row");
        let input = inputs.in_house.get(&key).expect("result input pair");
        assert_eq!(row.seed, input.seed, "result seed");
        assert_eq!(
            row.in_house_success, input.success,
            "result in-repository status"
        );
        assert!(row.elapsed_ms.is_finite() && row.elapsed_ms >= 0.0);
        if row.oxmpl_status == OxmplStatus::Success {
            assert_eq!(row.oxmpl_detail, "validated");
            assert!(row.waypoints >= 2);
            assert!(row.path_cost_rad.is_finite() && row.path_cost_rad > 0.0);
            assert_eq!(row.path_sha256.len(), 64);
        }
    }
}

fn aggregate(rows: impl Iterator<Item = ResultRow>) -> PairCounts {
    let mut counts = PairCounts::default();
    for row in rows {
        let oxmpl_success = row.oxmpl_status.succeeded();
        counts.total += 1;
        counts.in_house_success += usize::from(row.in_house_success);
        counts.oxmpl_success += usize::from(oxmpl_success);
        match (row.in_house_success, oxmpl_success) {
            (true, true) => counts.both += 1,
            (true, false) => counts.in_house_only += 1,
            (false, true) => counts.oxmpl_only += 1,
            (false, false) => counts.neither += 1,
        }
    }
    assert_eq!(
        counts.total,
        counts.both + counts.in_house_only + counts.oxmpl_only + counts.neither
    );
    counts
}

fn exact_mcnemar_p(in_house_only: usize, oxmpl_only: usize) -> f64 {
    let n = in_house_only + oxmpl_only;
    if n == 0 {
        return 1.0;
    }
    let tail = in_house_only.min(oxmpl_only);
    let ln_choose = (0..tail).fold(0.0, |sum, index| {
        sum + ((n - index) as f64).ln() - ((index + 1) as f64).ln()
    });
    let mut probability = (ln_choose - n as f64 * 2.0f64.ln()).exp();
    let mut cumulative = probability;
    for value in (1..=tail).rev() {
        probability *= value as f64 / (n - value + 1) as f64;
        cumulative += probability;
    }
    (2.0 * cumulative).min(1.0)
}

fn success_histograms(rows: &[ResultRow]) -> ([usize; 6], [usize; 6]) {
    let mut per_query: BTreeMap<(usize, usize), (usize, usize)> = BTreeMap::new();
    for row in rows {
        let entry = per_query
            .entry((row.scene.index, row.query_index))
            .or_default();
        entry.0 += usize::from(row.in_house_success);
        entry.1 += usize::from(row.oxmpl_status.succeeded());
    }
    assert_eq!(per_query.len(), EXPECTED_BLOCKED_QUERIES);
    let mut in_house = [0; 6];
    let mut oxmpl = [0; 6];
    for (left, right) in per_query.values() {
        assert!(*left <= REPLICATES && *right <= REPLICATES);
        in_house[*left] += 1;
        oxmpl[*right] += 1;
    }
    (in_house, oxmpl)
}

fn render_csv(rows: &[ResultRow]) -> String {
    let mut csv = String::from(
        "scene,scene_file,query_index,replicate,seed,in_house_status,oxmpl_status,\
         oxmpl_detail,oxmpl_elapsed_ms,oxmpl_waypoints,oxmpl_path_cost_rad,\
         oxmpl_path_sha256\n",
    );
    for row in rows {
        writeln!(
            csv,
            "{},{},{},{},{},{},{},{},{:.8},{},{:.8},{}",
            row.scene.name,
            row.scene.file,
            row.query_index,
            row.replicate,
            row.seed,
            if row.in_house_success {
                "success"
            } else {
                "failed"
            },
            row.oxmpl_status.as_str(),
            row.oxmpl_detail,
            row.elapsed_ms,
            row.waypoints,
            row.path_cost_rad,
            row.path_sha256
        )
        .expect("render result row");
    }
    csv
}

fn parse_results(csv: &str) -> Vec<ResultRow> {
    let mut lines = csv.lines();
    let header = lines.next().expect("OxMPL result header");
    assert_eq!(
        header,
        "scene,scene_file,query_index,replicate,seed,in_house_status,oxmpl_status,\
         oxmpl_detail,oxmpl_elapsed_ms,oxmpl_waypoints,oxmpl_path_cost_rad,\
         oxmpl_path_sha256"
    );
    let mut rows = Vec::new();
    for (line_index, line) in lines.enumerate() {
        let fields: Vec<_> = line.split(',').collect();
        assert_eq!(fields.len(), 12, "result line {} columns", line_index + 2);
        rows.push(ResultRow {
            scene: scene_from_fields(fields[0], fields[1]),
            query_index: parse(fields[2], "result query index"),
            replicate: parse(fields[3], "result replicate"),
            seed: parse(fields[4], "result seed"),
            in_house_success: match fields[5] {
                "success" => true,
                "failed" => false,
                _ => panic!("invalid in-repository result status"),
            },
            oxmpl_status: OxmplStatus::parse(fields[6]),
            oxmpl_detail: fields[7].to_string(),
            elapsed_ms: parse(fields[8], "OxMPL elapsed time"),
            waypoints: parse(fields[9], "OxMPL waypoints"),
            path_cost_rad: parse(fields[10], "OxMPL path cost"),
            path_sha256: fields[11].to_string(),
        });
    }
    rows
}

fn render_report(rows: &[ResultRow]) -> String {
    let pooled = aggregate(rows.iter().cloned());
    let p_value = exact_mcnemar_p(pooled.in_house_only, pooled.oxmpl_only);
    let invalid = rows
        .iter()
        .filter(|row| row.oxmpl_status == OxmplStatus::InvalidPath)
        .count();
    let (in_house_histogram, oxmpl_histogram) = success_histograms(rows);
    let in_house_any: usize = in_house_histogram[1..].iter().sum();
    let oxmpl_any: usize = oxmpl_histogram[1..].iter().sum();

    let mut report = String::new();
    report.push_str("# Independent OxMPL RRT-Connect comparison\n\n");
    report.push_str(
        "This result follows the outcome-blind `docs/oxmpl_baseline_protocol.md`. \
         It compares the in-repository planner with independently maintained \
         OxMPL 0.6.0 on all **217 frozen blocked-direct queries**, five paired \
         seeds each (**1,085 trials**). Every nominal OxMPL solution was \
         independently revalidated at 0.05-rad edge spacing before counting \
         as success.\n\n",
    );
    writeln!(
        report,
        "Pooled seed-trial success: **{}/{} ({:.2}%)** in-repository and **{}/{} ({:.2}%)** OxMPL.",
        pooled.in_house_success,
        pooled.total,
        percent(pooled.in_house_success, pooled.total),
        pooled.oxmpl_success,
        pooled.total,
        percent(pooled.oxmpl_success, pooled.total)
    )
    .expect("render pooled headline");
    writeln!(
        report,
        "Paired outcomes: **{} both**, **{} in-repository only**, **{} OxMPL only**, and **{} neither**; the predeclared exact two-sided McNemar/binomial calculation is **{p_value:.8}**. Because five seeds share each query, this calculation is descriptive only and does not satisfy independent-pair assumptions.\n",
        pooled.both, pooled.in_house_only, pooled.oxmpl_only, pooled.neither
    )
    .expect("render paired headline");

    report.push_str("## Results by scene\n\n");
    report.push_str(
        "| Scene | Trials | In-repository success | OxMPL success | Both | In-repository only | OxMPL only | Neither |\n\
         |---|---:|---:|---:|---:|---:|---:|---:|\n",
    );
    for scene in SCENES {
        let counts = aggregate(rows.iter().filter(|row| row.scene == scene).cloned());
        writeln!(
            report,
            "| {} | {} | {} | {} | {} | {} | {} | {} |",
            scene.name,
            counts.total,
            counts.in_house_success,
            counts.oxmpl_success,
            counts.both,
            counts.in_house_only,
            counts.oxmpl_only,
            counts.neither
        )
        .expect("render scene result");
    }

    report.push_str("\n## Per-query repeat distribution\n\n");
    report.push_str(
        "| Successful seeds out of five | In-repository queries | OxMPL queries |\n\
         |---:|---:|---:|\n",
    );
    for successes in 0..=REPLICATES {
        writeln!(
            report,
            "| {successes} | {} | {} |",
            in_house_histogram[successes], oxmpl_histogram[successes]
        )
        .expect("render repeat histogram");
    }
    writeln!(
        report,
        "\nAt least one of five seeds succeeded on **{in_house_any}/{}** blocked queries in-repository and **{oxmpl_any}/{}** with OxMPL. OxMPL returned **{invalid} invalid nominal paths** after independent checking.\n",
        EXPECTED_BLOCKED_QUERIES, EXPECTED_BLOCKED_QUERIES
    )
    .expect("render query recovery");

    report.push_str(
        "## Frozen configuration and integrity\n\n\
         - Inputs: query CSV `c07b622...b5452`; in-repository planning CSV `b739009...13055`.\n\
         - OxMPL: crates.io `oxmpl = 0.6.0`, crate archive `11988e3...980dc`, archive VCS commit `6ee31f4...e13`.\n\
         - Both planners use 0.25-rad extensions, 0.05 goal bias, the frozen paired seeds, and the same MuJoCo emitted-contact state predicate.\n\
         - OxMPL 0.6.0 motion sampling was configured to at most 0.05 rad; every returned segment was then rechecked independently at 0.05 rad.\n\
         - OxMPL received a fixed 250-ms wall-time budget; the in-repository artifact used 2,000 iterations. Runtime is intentionally not compared.\n\
         - OxMPL consumes eight-decimal serialized query coordinates; the original in-repository rows used pre-serialization values, a maximum representational difference of `5e-9` rad per coordinate.\n\n",
    );

    report.push_str(
        "The exact host, two-run resource measurements, artifact hashes, and \
         deterministic comparison gate are recorded in \
         [`docs/oxmpl_baseline_validation.md`](oxmpl_baseline_validation.md).\n\n",
    );

    report.push_str(
        "## Interpretation boundaries\n\n\
         - This is a cross-implementation comparison on one frozen cohort, not evidence that either planner is universally better.\n\
         - Five seeds for one query are repeated algorithm trials, not independent task draws. The retained seed-level McNemar/binomial calculation is descriptive, not a valid population significance test; no population confidence interval is attached.\n\
         - Path lengths are retained in the CSV only as diagnostics. The in-repository path is shortcut and densified; OxMPL returns a raw tree path, so cost is not a fair quality comparison.\n\
         - OxMPL's timeout is wall-clock-bound. Two consecutive runs on the recorded host must agree, but materially different hardware can change borderline timeout outcomes.\n\
         - Seed integers and replicate indices are paired, but arm-lab and OxMPL use different PRNGs; they do not receive identical random samples.\n\
         - Collision checks sample configurations and MuJoCo emitted contacts. They do not certify continuous avoidance or positive clearance.\n\
         - This is deterministic simulation evidence: no hardware, grasp-success, uncertainty, or sim-to-real claim is introduced.\n\n\
         ## Reproduce\n\n\
         ```bash\n\
         cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --write\n\
         cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --check\n\
         cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --verify-artifacts\n\
         ```\n\n\
         `oxmpl_elapsed_ms` is observational wall time and the only field normalized by `--check`.\n",
    );
    report
}

fn percent(numerator: usize, denominator: usize) -> f64 {
    100.0 * numerator as f64 / denominator as f64
}

fn write_artifacts(csv: &str, report: &str) {
    let docs = docs_dir();
    std::fs::write(docs.join("oxmpl_baseline_results.csv"), csv).expect("write OxMPL result CSV");
    std::fs::write(docs.join("oxmpl_baseline_results.md"), report)
        .expect("write OxMPL result report");
    println!("wrote independent OxMPL comparison artifacts");
}

fn check_artifacts(generated_csv: &str, generated_report: &str) {
    let docs = docs_dir();
    let committed_csv = std::fs::read_to_string(docs.join("oxmpl_baseline_results.csv"))
        .expect("read committed OxMPL result CSV");
    let committed_report = std::fs::read_to_string(docs.join("oxmpl_baseline_results.md"))
        .expect("read committed OxMPL result report");
    assert_artifact_equal(
        "OxMPL deterministic result fields",
        &normalize_elapsed(generated_csv),
        &normalize_elapsed(&committed_csv),
    );
    assert_artifact_equal("OxMPL report", generated_report, &committed_report);
    println!("OxMPL pinned-host artifact check passed; only wall time was normalized");
}

fn verify_committed_artifacts(inputs: &Inputs) {
    let docs = docs_dir();
    let committed_csv = std::fs::read_to_string(docs.join("oxmpl_baseline_results.csv"))
        .expect("read committed OxMPL result CSV");
    let committed_report = std::fs::read_to_string(docs.join("oxmpl_baseline_results.md"))
        .expect("read committed OxMPL result report");
    let rows = parse_results(&committed_csv);
    validate_result_layout(inputs, &rows);
    assert_artifact_equal("OxMPL result CSV", &render_csv(&rows), &committed_csv);
    assert_artifact_equal(
        "OxMPL report derived from CSV",
        &render_report(&rows),
        &committed_report,
    );
    println!("OxMPL frozen input, artifact schema, and report derivation checks passed");
}

fn normalize_elapsed(csv: &str) -> String {
    let mut lines = csv.lines();
    let header = lines.next().expect("result CSV header");
    let columns: Vec<_> = header.split(',').collect();
    let elapsed_index = columns
        .iter()
        .position(|column| *column == "oxmpl_elapsed_ms")
        .expect("OxMPL elapsed column");
    let mut normalized = format!("{header}\n");
    for line in lines {
        let mut fields: Vec<_> = line.split(',').collect();
        assert_eq!(fields.len(), columns.len(), "result CSV column count");
        fields[elapsed_index] = "<ignored-wall-clock>";
        writeln!(normalized, "{}", fields.join(",")).expect("normalize result row");
    }
    normalized
}

fn assert_artifact_equal(label: &str, generated: &str, committed: &str) {
    if generated == committed {
        return;
    }
    let mismatch = generated
        .lines()
        .zip(committed.lines())
        .position(|(left, right)| left != right)
        .map_or_else(
            || generated.lines().count().min(committed.lines().count()) + 1,
            |index| index + 1,
        );
    panic!("stale {label}: first mismatch at line {mismatch}");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_blocked_query_and_default_planning_rows() {
        let query_csv = "scene,scene_file,query_index,generator_seed,accepted_draw_index,cumulative_rejected_start_collision,cumulative_rejected_goal_collision,cumulative_rejected_short,start_q_rad,goal_q_rad,distance_rad,direct_path_free\nopen_floor,scene.xml,7,1,2,0,0,0,0;1;2;3;4;5,1;2;3;4;5;6,2.0,false\nopen_floor,scene.xml,8,1,3,0,0,0,0;1;2;3;4;5,1;2;3;4;5;6,2.0,true\n";
        let rows = parse_queries(query_csv);
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].query_index, 7);

        let planning_csv = "scene,scene_file,query_index,variant,replicate,seed,direct_path_free,status,plan_elapsed_ms,iterations,nodes,shortcut_waypoints,path_samples,path_cost_rad\nopen_floor,scene.xml,7,default,0,202608180070,false,success,1,2,3,4,5,6\nopen_floor,scene.xml,7,zero_goal_bias,0,202608180070,false,success,1,2,3,4,5,6\n";
        let parsed = parse_in_house(planning_csv);
        assert_eq!(parsed.len(), 1);
        assert!(parsed[&(0, 7, 0)].success);
    }

    #[test]
    fn planner_seeds_match_frozen_formula() {
        assert_eq!(planner_seed(0, 0, 0), 202_608_180_000);
        assert_eq!(planner_seed(1, 99, 4), 202_608_190_994);
        assert_eq!(planner_seed(2, 42, 3), 202_608_200_423);
    }

    #[test]
    fn path_validation_checks_endpoints_bounds_and_edges() {
        let bounds = [(-2.0, 2.0), (-2.0, 2.0)];
        let good = vec![vec![0.0, 0.0], vec![0.5, 0.0], vec![1.0, 0.0]];
        assert_eq!(
            validate_path(&good, &[0.0, 0.0], &[1.0, 0.0], &bounds, |_| false),
            Ok(())
        );
        assert_eq!(
            validate_path(&good, &[0.1, 0.0], &[1.0, 0.0], &bounds, |_| false),
            Err("start_mismatch".to_string())
        );
        assert_eq!(
            validate_path(&good, &[0.0, 0.0], &[1.0, 0.0], &bounds, |q| q[0] > 0.7),
            Err("edge_collision".to_string())
        );
        let out_of_bounds = vec![vec![0.0, 0.0], vec![3.0, 0.0]];
        assert_eq!(
            validate_path(&out_of_bounds, &[0.0, 0.0], &[3.0, 0.0], &bounds, |_| false),
            Err("state_out_of_bounds".to_string())
        );
    }

    #[test]
    fn paired_aggregation_retains_all_four_cells() {
        let scene = SCENES[0];
        let combinations = [(true, true), (true, false), (false, true), (false, false)];
        let rows = combinations
            .into_iter()
            .enumerate()
            .map(|(index, (in_house, oxmpl))| ResultRow {
                scene,
                query_index: index,
                replicate: 0,
                seed: index as u64,
                in_house_success: in_house,
                oxmpl_status: if oxmpl {
                    OxmplStatus::Success
                } else {
                    OxmplStatus::Timeout
                },
                oxmpl_detail: String::new(),
                elapsed_ms: 0.0,
                waypoints: 0,
                path_cost_rad: 0.0,
                path_sha256: String::new(),
            });
        let counts = aggregate(rows);
        assert_eq!(counts.total, 4);
        assert_eq!(counts.in_house_success, 2);
        assert_eq!(counts.oxmpl_success, 2);
        assert_eq!(counts.both, 1);
        assert_eq!(counts.in_house_only, 1);
        assert_eq!(counts.oxmpl_only, 1);
        assert_eq!(counts.neither, 1);
    }

    #[test]
    fn exact_mcnemar_matches_small_known_cases() {
        assert_eq!(exact_mcnemar_p(0, 0), 1.0);
        assert_eq!(exact_mcnemar_p(1, 1), 1.0);
        assert!((exact_mcnemar_p(0, 5) - 0.0625).abs() < 1e-12);
        assert!((exact_mcnemar_p(2, 8) - 0.109375).abs() < 1e-12);
    }

    #[test]
    fn elapsed_normalization_changes_only_wall_time() {
        let csv = "a,oxmpl_elapsed_ms,b\nx,12.3,y\n";
        assert_eq!(
            normalize_elapsed(csv),
            "a,oxmpl_elapsed_ms,b\nx,<ignored-wall-clock>,y\n"
        );
    }

    #[test]
    #[should_panic(expected = "stale test artifact: first mismatch at line 2")]
    fn stale_artifact_is_rejected() {
        assert_artifact_equal("test artifact", "header\nnew\n", "header\nold\n");
    }
}
