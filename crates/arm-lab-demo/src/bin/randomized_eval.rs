//! Predeclared randomized planning and nominal-tracking evaluation.
//!
//! The complete design and interpretation boundaries are frozen in
//! `docs/randomized_eval_protocol.md`. This executable retains all accepted
//! queries and outcomes; it does not enforce result-based promotion gates.
//!
//! ```text
//! cargo run --release -p arm-lab-demo --bin randomized_eval
//! cargo run --release -p arm-lab-demo --bin randomized_eval -- --write
//! cargo run --release -p arm-lab-demo --bin randomized_eval -- --check
//! ```

use std::fmt::Write as _;
use std::path::Path;

use arm_lab::plan::{PlanStatus, edge_free, rrt_connect};
use arm_lab::traj::{TrajLimits, Trajectory, time_parameterize};
use arm_lab::{Chain, CollisionChecker, PlanConfig, Rng};
use arm_lab_demo::{read_q, set_ctrl};
use mujoco_rs::prelude::*;

const ASSET_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/ur5e/");
const ACTUATORS: [&str; 6] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
];

const QUERIES_PER_SCENE: usize = 100;
const MAX_DRAWS_PER_SCENE: usize = 100_000;
const MIN_QUERY_DISTANCE_RAD: f64 = 0.75;
const QUERY_SEED_BASE: u64 = 202_608_170_100;
const PLANNER_SEED_BASE: u64 = 202_608_180_000;
const BLOCKED_REPLICATES: usize = 5;

const KV_OVER_KP: f64 = 0.2;
const SETTLE_STEPS: usize = 250;
const HOLD_STEPS: usize = 250;
const PASS_RMS_RAD: f64 = 0.03;
const PASS_MAX_RAD: f64 = 0.10;
const PASS_FINAL_RAD: f64 = 0.02;
const EXECUTION_CONTACT_THRESHOLD_M: f64 = 0.0;
const LIMITS: TrajLimits = TrajLimits {
    v_max: 0.55,
    a_max: 1.8,
    j_max: 8.0,
};

#[derive(Clone, Copy)]
struct Scene {
    name: &'static str,
    file: &'static str,
}

const SCENES: [Scene; 3] = [
    Scene {
        name: "open_floor",
        file: "scene.xml",
    },
    Scene {
        name: "offset_pillar",
        file: "scene_cluttered.xml",
    },
    Scene {
        name: "tabletop_pillar",
        file: "scene_pickplace.xml",
    },
];

#[derive(Clone)]
struct QueryRow {
    scene_index: usize,
    scene: &'static str,
    scene_file: &'static str,
    query_index: usize,
    generator_seed: u64,
    accepted_draw_index: usize,
    rejected_start_collision: usize,
    rejected_goal_collision: usize,
    rejected_short: usize,
    start: Vec<f64>,
    goal: Vec<f64>,
    distance_rad: f64,
    direct_free: bool,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PlannerVariant {
    Default,
    ZeroGoalBias,
}

impl PlannerVariant {
    fn name(self) -> &'static str {
        match self {
            Self::Default => "default",
            Self::ZeroGoalBias => "zero_goal_bias",
        }
    }

    fn config(self, seed: u64) -> PlanConfig {
        PlanConfig {
            seed,
            goal_bias: match self {
                Self::Default => PlanConfig::default().goal_bias,
                Self::ZeroGoalBias => 0.0,
            },
            ..PlanConfig::default()
        }
    }
}

struct PlanningRow {
    scene: &'static str,
    scene_file: &'static str,
    query_index: usize,
    variant: PlannerVariant,
    replicate: usize,
    seed: u64,
    direct_free: bool,
    status: PlanStatus,
    elapsed_ms: f64,
    iterations: usize,
    nodes: usize,
    shortcut_waypoints: usize,
    path_samples: usize,
    path_cost_rad: f64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Controller {
    Position,
    VelocityFf,
}

impl Controller {
    fn name(self) -> &'static str {
        match self {
            Self::Position => "position PD",
            Self::VelocityFf => "PD + velocity FF",
        }
    }

    fn velocity_ff(self) -> bool {
        matches!(self, Self::VelocityFf)
    }
}

const CONTROLLERS: [Controller; 2] = [Controller::Position, Controller::VelocityFf];

struct TrackingRow {
    scene: &'static str,
    scene_file: &'static str,
    query_index: usize,
    seed: u64,
    direct_free: bool,
    controller: Controller,
    trajectory_samples: usize,
    trajectory_duration_s: f64,
    rms_joint_rad: f64,
    max_joint_rad: f64,
    final_joint_rad: f64,
    max_ee_pos_m: f64,
    peak_force_fraction: f64,
    saturated_step_fraction: f64,
    settle_penetration_steps: usize,
    path_penetration_steps: usize,
    hold_penetration_steps: usize,
    max_penetration_m: f64,
    worst_contact: Option<String>,
    numeric_pass: bool,
    full_pass: bool,
}

#[derive(Default)]
struct PenetrationMetrics {
    steps: usize,
    max_penetration_m: f64,
    worst_contact: Option<String>,
}

impl PenetrationMetrics {
    fn observe(&mut self, checker: &mut CollisionChecker<&MjModel>, q: &[f64]) {
        let contacts = checker.robot_contacts(q);
        if contacts.is_empty() {
            return;
        }
        self.steps += 1;
        for contact in contacts {
            let penetration = (-contact.distance_m).max(0.0);
            if penetration > self.max_penetration_m {
                self.max_penetration_m = penetration;
                self.worst_contact = Some(contact.identity());
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Run,
    Write,
    Check,
}

fn parse_mode() -> Mode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [] => Mode::Run,
        [arg] if arg == "--write" => Mode::Write,
        [arg] if arg == "--check" => Mode::Check,
        _ => panic!("usage: randomized_eval [--write|--check]"),
    }
}

fn main() {
    let mode = parse_mode();
    let mut queries = Vec::with_capacity(SCENES.len() * QUERIES_PER_SCENE);
    let mut planning = Vec::new();
    let mut tracking = Vec::new();

    for (scene_index, scene) in SCENES.iter().copied().enumerate() {
        let model = load_model(scene);
        let chain = extract_chain(&model);
        let scene_queries = sample_queries(scene_index, scene, &model, &chain);
        let blocked = scene_queries
            .iter()
            .filter(|query| !query.direct_free)
            .count();
        println!(
            "{}: sampled {} queries ({} direct-free, {} blocked)",
            scene.name,
            scene_queries.len(),
            scene_queries.len() - blocked,
            blocked
        );

        for query in &scene_queries {
            let mut canonical_trajectory = None;
            let variants: &[PlannerVariant] = if query.direct_free {
                &[PlannerVariant::Default]
            } else {
                &[PlannerVariant::Default, PlannerVariant::ZeroGoalBias]
            };

            for &variant in variants {
                let replicates = if query.direct_free {
                    1
                } else {
                    BLOCKED_REPLICATES
                };
                for replicate in 0..replicates {
                    let seed = planner_seed(scene_index, query.query_index, replicate);
                    let mut collision = CollisionChecker::new(&model, &chain);
                    let plan = rrt_connect(
                        &chain,
                        &query.start,
                        &query.goal,
                        |q| collision.collides(q),
                        &variant.config(seed),
                    );
                    if variant == PlannerVariant::Default
                        && replicate == 0
                        && plan.status == PlanStatus::Success
                    {
                        canonical_trajectory = Some((
                            seed,
                            time_parameterize(&plan.path, &LIMITS, model.opt().timestep),
                        ));
                    }
                    planning.push(PlanningRow {
                        scene: scene.name,
                        scene_file: scene.file,
                        query_index: query.query_index,
                        variant,
                        replicate,
                        seed,
                        direct_free: query.direct_free,
                        status: plan.status,
                        elapsed_ms: 1e3 * plan.elapsed_s,
                        iterations: plan.iterations,
                        nodes: plan.nodes,
                        shortcut_waypoints: plan.waypoints.len(),
                        path_samples: plan.path.len(),
                        path_cost_rad: plan.cost,
                    });
                }
            }

            if let Some((seed, trajectory)) = canonical_trajectory {
                for controller in CONTROLLERS {
                    tracking.push(run_tracking_case(
                        scene,
                        query,
                        seed,
                        controller,
                        &model,
                        &chain,
                        &trajectory,
                    ));
                }
            }
        }

        queries.extend(scene_queries);
    }

    validate_layout(&queries, &planning, &tracking);
    let canonical_successes = canonical_plans(&planning)
        .filter(|row| row.status == PlanStatus::Success)
        .count();
    println!(
        "summary: {}/{} canonical plans succeeded; {} tracking cases executed",
        canonical_successes,
        queries.len(),
        tracking.len()
    );

    match mode {
        Mode::Run => {}
        Mode::Write => write_artifacts(&queries, &planning, &tracking),
        Mode::Check => check_artifacts(&queries, &planning, &tracking),
    }
}

fn load_model(scene: Scene) -> MjModel {
    let path = format!("{ASSET_DIR}{}", scene.file);
    MjModel::from_xml(&path).unwrap_or_else(|error| panic!("load {}: {error}", scene.file))
}

fn extract_chain(model: &MjModel) -> Chain {
    Chain::from_mujoco(model, "ur5e", "wrist_3_link", "attachment_site")
        .expect("extract UR5e chain")
}

fn sample_queries(
    scene_index: usize,
    scene: Scene,
    model: &MjModel,
    chain: &Chain,
) -> Vec<QueryRow> {
    let generator_seed = QUERY_SEED_BASE + scene_index as u64;
    let mut rng = Rng::new(generator_seed);
    let mut collision = CollisionChecker::new(model, chain);
    let mut queries = Vec::with_capacity(QUERIES_PER_SCENE);
    let mut rejected_start_collision = 0usize;
    let mut rejected_goal_collision = 0usize;
    let mut rejected_short = 0usize;

    for draw_index in 1..=MAX_DRAWS_PER_SCENE {
        let start = chain.sample_uniform(&mut rng);
        let goal = chain.sample_uniform(&mut rng);
        if collision.collides(&start) {
            rejected_start_collision += 1;
            continue;
        }
        if collision.collides(&goal) {
            rejected_goal_collision += 1;
            continue;
        }
        let distance_rad = l2(&start, &goal);
        if distance_rad < MIN_QUERY_DISTANCE_RAD {
            rejected_short += 1;
            continue;
        }

        let mut scratch = vec![0.0; chain.dof()];
        let direct_free = edge_free(
            &start,
            &goal,
            PlanConfig::default().resolution,
            &mut |q| collision.collides(q),
            &mut scratch,
        );
        queries.push(QueryRow {
            scene_index,
            scene: scene.name,
            scene_file: scene.file,
            query_index: queries.len(),
            generator_seed,
            accepted_draw_index: draw_index,
            rejected_start_collision,
            rejected_goal_collision,
            rejected_short,
            start,
            goal,
            distance_rad,
            direct_free,
        });
        if queries.len() == QUERIES_PER_SCENE {
            break;
        }
    }

    assert_eq!(
        queries.len(),
        QUERIES_PER_SCENE,
        "{} did not produce the frozen query cohort within {} draws",
        scene.name,
        MAX_DRAWS_PER_SCENE
    );
    queries
}

fn planner_seed(scene_index: usize, query_index: usize, replicate: usize) -> u64 {
    PLANNER_SEED_BASE + 10_000 * scene_index as u64 + 10 * query_index as u64 + replicate as u64
}

fn canonical_plans(planning: &[PlanningRow]) -> impl Iterator<Item = &PlanningRow> {
    planning
        .iter()
        .filter(|row| row.variant == PlannerVariant::Default && row.replicate == 0)
}

fn validate_layout(queries: &[QueryRow], planning: &[PlanningRow], tracking: &[TrackingRow]) {
    assert_eq!(queries.len(), SCENES.len() * QUERIES_PER_SCENE);
    for (scene_index, scene) in SCENES.iter().enumerate() {
        assert_eq!(
            queries
                .iter()
                .filter(|query| query.scene_index == scene_index)
                .count(),
            QUERIES_PER_SCENE,
            "{} query cohort size",
            scene.name
        );
    }

    let expected_planning_rows: usize = queries
        .iter()
        .map(|query| {
            if query.direct_free {
                1
            } else {
                2 * BLOCKED_REPLICATES
            }
        })
        .sum();
    assert_eq!(planning.len(), expected_planning_rows);
    for query in queries {
        let rows: Vec<_> = planning
            .iter()
            .filter(|row| row.scene == query.scene && row.query_index == query.query_index)
            .collect();
        assert_eq!(
            rows.len(),
            if query.direct_free {
                1
            } else {
                2 * BLOCKED_REPLICATES
            },
            "{}/{} planner row count",
            query.scene,
            query.query_index
        );
        assert_eq!(
            rows.iter()
                .filter(|row| { row.variant == PlannerVariant::Default && row.replicate == 0 })
                .count(),
            1,
            "{}/{} canonical row count",
            query.scene,
            query.query_index
        );
    }

    let canonical_successes = canonical_plans(planning)
        .filter(|row| row.status == PlanStatus::Success)
        .count();
    assert_eq!(tracking.len(), 2 * canonical_successes);
}

#[allow(clippy::too_many_arguments)]
fn run_tracking_case(
    scene: Scene,
    query: &QueryRow,
    seed: u64,
    controller: Controller,
    model: &MjModel,
    chain: &Chain,
    trajectory: &Trajectory,
) -> TrackingRow {
    let mut data = MjData::new(model);
    let q_start = trajectory.q.first().expect("trajectory start");
    for (&address, &value) in chain.qpos_addresses().iter().zip(q_start) {
        data.qpos_mut()[address] = value;
    }
    for address in chain.dof_addresses() {
        data.qvel_mut()[address] = 0.0;
    }
    data.forward();

    let zero = vec![0.0; chain.dof()];
    let mut collision_checker = CollisionChecker::new(model, chain);
    collision_checker.contact_threshold = EXECUTION_CONTACT_THRESHOLD_M;
    let mut settle_penetration = PenetrationMetrics::default();
    let mut path_penetration = PenetrationMetrics::default();
    let mut hold_penetration = PenetrationMetrics::default();

    for _ in 0..SETTLE_STEPS {
        control_step(controller, &mut data, q_start, &zero);
        settle_penetration.observe(&mut collision_checker, &read_q(&data, chain));
    }

    let mut sum_sq = 0.0;
    let mut max_joint = 0.0f64;
    let mut max_ee = 0.0f64;
    let mut peak_force_fraction = 0.0f64;
    let mut saturated_steps = 0usize;
    for (q_des, qd_des) in trajectory.q.iter().zip(&trajectory.qd) {
        control_step(controller, &mut data, q_des, qd_des);
        let q_measured = read_q(&data, chain);
        path_penetration.observe(&mut collision_checker, &q_measured);
        let error = l2(&q_measured, q_des);
        sum_sq += error * error;
        max_joint = max_joint.max(error);
        let ee_error = (arm_lab::kinematics::fk(chain, &q_measured)
            .translation
            .vector
            - arm_lab::kinematics::fk(chain, q_des).translation.vector)
            .norm();
        max_ee = max_ee.max(ee_error);
        let force_fraction = force_fraction(model, &data);
        peak_force_fraction = peak_force_fraction.max(force_fraction);
        saturated_steps += usize::from(force_fraction >= 0.999);
    }

    let q_goal = trajectory.q.last().expect("trajectory goal");
    for _ in 0..HOLD_STEPS {
        control_step(controller, &mut data, q_goal, &zero);
        hold_penetration.observe(&mut collision_checker, &read_q(&data, chain));
    }

    let rms_joint = (sum_sq / trajectory.len() as f64).sqrt();
    let final_joint = l2(&read_q(&data, chain), q_goal);
    let penetration_steps =
        settle_penetration.steps + path_penetration.steps + hold_penetration.steps;
    let numeric_pass = numeric_gates_pass(rms_joint, max_joint, final_joint);
    let max_penetration_m = settle_penetration
        .max_penetration_m
        .max(path_penetration.max_penetration_m)
        .max(hold_penetration.max_penetration_m);
    let worst_contact = [&settle_penetration, &path_penetration, &hold_penetration]
        .into_iter()
        .max_by(|left, right| left.max_penetration_m.total_cmp(&right.max_penetration_m))
        .and_then(|metrics| metrics.worst_contact.clone());

    TrackingRow {
        scene: scene.name,
        scene_file: scene.file,
        query_index: query.query_index,
        seed,
        direct_free: query.direct_free,
        controller,
        trajectory_samples: trajectory.len(),
        trajectory_duration_s: trajectory.duration,
        rms_joint_rad: rms_joint,
        max_joint_rad: max_joint,
        final_joint_rad: final_joint,
        max_ee_pos_m: max_ee,
        peak_force_fraction,
        saturated_step_fraction: saturated_steps as f64 / trajectory.len() as f64,
        settle_penetration_steps: settle_penetration.steps,
        path_penetration_steps: path_penetration.steps,
        hold_penetration_steps: hold_penetration.steps,
        max_penetration_m,
        worst_contact,
        numeric_pass,
        full_pass: full_gates_pass(rms_joint, max_joint, final_joint, penetration_steps),
    }
}

fn control_step(
    controller: Controller,
    data: &mut MjData<&MjModel>,
    q_des: &[f64],
    qd_des: &[f64],
) {
    let mut ctrl = q_des.to_vec();
    if controller.velocity_ff() {
        for (command, velocity) in ctrl.iter_mut().zip(qd_des) {
            *command += KV_OVER_KP * velocity;
        }
    }
    set_ctrl(data, &ACTUATORS, &ctrl);
    data.step();
}

fn force_fraction(model: &MjModel, data: &MjData<&MjModel>) -> f64 {
    data.actuator_force()
        .iter()
        .zip(model.actuator_forcerange())
        .map(|(&force, range)| {
            let limit = range[0].abs().max(range[1].abs());
            if limit > 0.0 {
                force.abs() / limit
            } else {
                0.0
            }
        })
        .fold(0.0, f64::max)
}

fn numeric_gates_pass(rms: f64, max: f64, final_error: f64) -> bool {
    rms <= PASS_RMS_RAD && max <= PASS_MAX_RAD && final_error <= PASS_FINAL_RAD
}

fn full_gates_pass(rms: f64, max: f64, final_error: f64, penetration_steps: usize) -> bool {
    numeric_gates_pass(rms, max, final_error) && penetration_steps == 0
}

fn l2(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(left, right)| (left - right).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn status_name(status: PlanStatus) -> &'static str {
    match status {
        PlanStatus::Success => "success",
        PlanStatus::StartCollision => "start_collision",
        PlanStatus::GoalCollision => "goal_collision",
        PlanStatus::Unconnected => "unconnected",
    }
}

fn format_joint_vector(q: &[f64]) -> String {
    q.iter()
        .map(|value| format!("{value:.8}"))
        .collect::<Vec<_>>()
        .join(";")
}

fn wilson_interval(successes: usize, total: usize) -> (f64, f64) {
    assert!(total > 0, "Wilson interval needs a non-empty sample");
    let n = total as f64;
    let p = successes as f64 / n;
    let z = 1.959_963_984_540_054;
    let z2 = z * z;
    let denominator = 1.0 + z2 / n;
    let center = (p + z2 / (2.0 * n)) / denominator;
    let half = z * ((p * (1.0 - p) + z2 / (4.0 * n)) / n).sqrt() / denominator;
    ((center - half).max(0.0), (center + half).min(1.0))
}

fn format_rate(successes: usize, total: usize) -> String {
    if total == 0 {
        return "0/0 (not evaluated)".to_string();
    }
    let (lower, upper) = wilson_interval(successes, total);
    format!(
        "{successes}/{total} ({:.1}%; 95% Wilson {:.1}--{:.1}%)",
        100.0 * successes as f64 / total as f64,
        100.0 * lower,
        100.0 * upper
    )
}

struct Artifacts {
    queries_csv: String,
    planning_csv: String,
    tracking_csv: String,
    report: String,
}

fn render_artifacts(
    queries: &[QueryRow],
    planning: &[PlanningRow],
    tracking: &[TrackingRow],
) -> Artifacts {
    let mut queries_csv = String::from(
        "scene,scene_file,query_index,generator_seed,accepted_draw_index,cumulative_rejected_start_collision,cumulative_rejected_goal_collision,cumulative_rejected_short,start_q_rad,goal_q_rad,distance_rad,direct_path_free\n",
    );
    for row in queries {
        writeln!(
            queries_csv,
            "{},{},{},{},{},{},{},{},{},{},{:.8},{}",
            row.scene,
            row.scene_file,
            row.query_index,
            row.generator_seed,
            row.accepted_draw_index,
            row.rejected_start_collision,
            row.rejected_goal_collision,
            row.rejected_short,
            format_joint_vector(&row.start),
            format_joint_vector(&row.goal),
            row.distance_rad,
            row.direct_free
        )
        .expect("format query CSV");
    }

    let mut planning_csv = String::from(
        "scene,scene_file,query_index,variant,replicate,seed,direct_path_free,status,plan_elapsed_ms,iterations,nodes,shortcut_waypoints,path_samples,path_cost_rad\n",
    );
    for row in planning {
        writeln!(
            planning_csv,
            "{},{},{},{},{},{},{},{},{:.8},{},{},{},{},{:.8}",
            row.scene,
            row.scene_file,
            row.query_index,
            row.variant.name(),
            row.replicate,
            row.seed,
            row.direct_free,
            status_name(row.status),
            row.elapsed_ms,
            row.iterations,
            row.nodes,
            row.shortcut_waypoints,
            row.path_samples,
            row.path_cost_rad
        )
        .expect("format planning CSV");
    }

    let mut tracking_csv = String::from(
        "scene,scene_file,query_index,seed,direct_path_free,controller,trajectory_samples,trajectory_duration_s,rms_joint_rad,max_joint_rad,final_joint_rad,max_ee_pos_m,peak_force_fraction,saturated_step_fraction,settle_penetration_steps,path_penetration_steps,hold_penetration_steps,max_penetration_m,worst_contact,numeric_pass,full_pass\n",
    );
    for row in tracking {
        writeln!(
            tracking_csv,
            "{},{},{},{},{},{},{},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{},{},{},{:.8},{},{},{}",
            row.scene,
            row.scene_file,
            row.query_index,
            row.seed,
            row.direct_free,
            row.controller.name(),
            row.trajectory_samples,
            row.trajectory_duration_s,
            row.rms_joint_rad,
            row.max_joint_rad,
            row.final_joint_rad,
            row.max_ee_pos_m,
            row.peak_force_fraction,
            row.saturated_step_fraction,
            row.settle_penetration_steps,
            row.path_penetration_steps,
            row.hold_penetration_steps,
            row.max_penetration_m,
            row.worst_contact.as_deref().unwrap_or(""),
            row.numeric_pass,
            row.full_pass
        )
        .expect("format tracking CSV");
    }

    let report = render_report(queries, planning, tracking);
    Artifacts {
        queries_csv,
        planning_csv,
        tracking_csv,
        report,
    }
}

fn render_report(
    queries: &[QueryRow],
    planning: &[PlanningRow],
    tracking: &[TrackingRow],
) -> String {
    let mut report = String::from(
        "# Randomized UR5e planning and nominal tracking evaluation (simulation)\n\n\
         This result follows the predeclared `docs/randomized_eval_protocol.md`. It retains the first 100 accepted independently sampled joint-space queries in each of three shipped scenes, without filtering on direct-path or planner outcome. **This is deterministic simulation evidence, not hardware validation, a workspace-uniform task distribution, continuous safety, or sim-to-real evidence.**\n\n",
    );

    report.push_str("## Query sampling and canonical planning\n\n");
    report.push_str(
        "| Scene | Accepted draw | Endpoint/short rejections | Direct baseline | Canonical default RRT | Blocked recovered |\n|---|---:|---:|---:|---:|---:|\n",
    );
    for (scene_index, scene) in SCENES.iter().copied().enumerate() {
        let scene_queries: Vec<_> = queries
            .iter()
            .filter(|row| row.scene_index == scene_index)
            .collect();
        let final_query = scene_queries.last().expect("scene query cohort");
        let direct = scene_queries.iter().filter(|row| row.direct_free).count();
        let canonical: Vec<_> = canonical_plans(planning)
            .filter(|row| row.scene == scene.name)
            .collect();
        let solved = canonical
            .iter()
            .filter(|row| row.status == PlanStatus::Success)
            .count();
        let recovered = canonical
            .iter()
            .filter(|row| !row.direct_free && row.status == PlanStatus::Success)
            .count();
        writeln!(
            report,
            "| {} | {} | {}/{}/{} | {} | {} | {}/{} |",
            scene.name,
            final_query.accepted_draw_index,
            final_query.rejected_start_collision,
            final_query.rejected_goal_collision,
            final_query.rejected_short,
            format_rate(direct, scene_queries.len()),
            format_rate(solved, canonical.len()),
            recovered,
            scene_queries.len() - direct
        )
        .expect("format scene summary");
    }

    let pooled_direct = queries.iter().filter(|row| row.direct_free).count();
    let canonical: Vec<_> = canonical_plans(planning).collect();
    let pooled_solved = canonical
        .iter()
        .filter(|row| row.status == PlanStatus::Success)
        .count();
    let pooled_recovered = canonical
        .iter()
        .filter(|row| !row.direct_free && row.status == PlanStatus::Success)
        .count();
    writeln!(
        report,
        "\nEqual-scene pooled direct baseline: **{}**. Canonical default RRT: **{}**. It recovers **{}/{}** blocked-direct queries.\n",
        format_rate(pooled_direct, queries.len()),
        format_rate(pooled_solved, canonical.len()),
        pooled_recovered,
        queries.len() - pooled_direct
    )
    .expect("format pooled planning");
    let canonical_unconnected = canonical
        .iter()
        .filter(|row| row.status == PlanStatus::Unconnected)
        .count();
    writeln!(
        report,
        "All 300 endpoint pairs pass the frozen endpoint predicate by construction. Canonical outcomes retain **{pooled_solved} success** and **{canonical_unconnected} unconnected** statuses; no failed query is removed.\n"
    )
    .expect("format canonical statuses");

    report.push_str("## Blocked-query goal-bias ablation\n\n");
    let blocked_queries: Vec<_> = queries.iter().filter(|row| !row.direct_free).collect();
    report.push_str(
        "| Scene | Blocked queries | Default successes | Zero-goal-bias successes |\n|---|---:|---:|---:|\n",
    );
    for scene in SCENES {
        let scene_blocked = blocked_queries
            .iter()
            .filter(|query| query.scene == scene.name)
            .count();
        let default_rows: Vec<_> = planning
            .iter()
            .filter(|row| {
                row.scene == scene.name
                    && !row.direct_free
                    && row.variant == PlannerVariant::Default
            })
            .collect();
        let zero_rows: Vec<_> = planning
            .iter()
            .filter(|row| {
                row.scene == scene.name
                    && !row.direct_free
                    && row.variant == PlannerVariant::ZeroGoalBias
            })
            .collect();
        writeln!(
            report,
            "| {} | {} | {}/{} | {}/{} |",
            scene.name,
            scene_blocked,
            default_rows
                .iter()
                .filter(|row| row.status == PlanStatus::Success)
                .count(),
            default_rows.len(),
            zero_rows
                .iter()
                .filter(|row| row.status == PlanStatus::Success)
                .count(),
            zero_rows.len()
        )
        .expect("format scene ablation");
    }
    report.push('\n');
    for variant in [PlannerVariant::Default, PlannerVariant::ZeroGoalBias] {
        let rows: Vec<_> = planning
            .iter()
            .filter(|row| !row.direct_free && row.variant == variant)
            .collect();
        let successes = rows
            .iter()
            .filter(|row| row.status == PlanStatus::Success)
            .count();
        let mut histogram = [0usize; BLOCKED_REPLICATES + 1];
        for query in &blocked_queries {
            let count = rows
                .iter()
                .filter(|row| {
                    row.scene == query.scene
                        && row.query_index == query.query_index
                        && row.status == PlanStatus::Success
                })
                .count();
            histogram[count] += 1;
        }
        writeln!(
            report,
            "- `{}`: {}/{} successful planner trials. Per-query success histogram (0/5 through 5/5): {:?}.",
            variant.name(),
            successes,
            rows.len(),
            histogram
        )
        .expect("format ablation");
    }
    report.push_str(
        "\nRepeated planner trials probe algorithmic seed sensitivity on the same blocked queries; they are not independent task samples and receive no Wilson interval.\n\n",
    );

    report.push_str("## Nominal tracking\n\n");
    report.push_str(
        "Tracking is replayed only when the canonical default plan succeeds. Both controllers receive each identical successful trajectory.\n\n| Controller | Replays | Numeric gates | Full zero-penetration gate |\n|---|---:|---:|---:|\n",
    );
    for controller in CONTROLLERS {
        let rows: Vec<_> = tracking
            .iter()
            .filter(|row| row.controller == controller)
            .collect();
        let numeric = rows.iter().filter(|row| row.numeric_pass).count();
        let full = rows.iter().filter(|row| row.full_pass).count();
        writeln!(
            report,
            "| {} | {} | {} | {} |",
            controller.name(),
            rows.len(),
            format_rate(numeric, rows.len()),
            format_rate(full, rows.len())
        )
        .expect("format tracking summary");
    }
    report.push_str(
        "\n### Scene and sampled-penetration breakdown\n\n| Scene | Controller | Replays | Numeric | Full | Penetration cases | Steps S/P/H | Max penetration (m) |\n|---|---|---:|---:|---:|---:|---:|---:|\n",
    );
    for scene in SCENES {
        for controller in CONTROLLERS {
            let rows: Vec<_> = tracking
                .iter()
                .filter(|row| row.scene == scene.name && row.controller == controller)
                .collect();
            let penetration_cases = rows
                .iter()
                .filter(|row| {
                    row.settle_penetration_steps
                        + row.path_penetration_steps
                        + row.hold_penetration_steps
                        > 0
                })
                .count();
            let settle_steps: usize = rows.iter().map(|row| row.settle_penetration_steps).sum();
            let path_steps: usize = rows.iter().map(|row| row.path_penetration_steps).sum();
            let hold_steps: usize = rows.iter().map(|row| row.hold_penetration_steps).sum();
            let max_penetration = rows
                .iter()
                .map(|row| row.max_penetration_m)
                .fold(0.0f64, f64::max);
            writeln!(
                report,
                "| {} | {} | {} | {}/{} | {}/{} | {}/{} | {}/{}/{} | {:.8} |",
                scene.name,
                controller.name(),
                rows.len(),
                rows.iter().filter(|row| row.numeric_pass).count(),
                rows.len(),
                rows.iter().filter(|row| row.full_pass).count(),
                rows.len(),
                penetration_cases,
                rows.len(),
                settle_steps,
                path_steps,
                hold_steps,
                max_penetration
            )
            .expect("format scene tracking breakdown");
        }
    }

    let mut ff_only = 0usize;
    let mut pd_only = 0usize;
    for position in tracking
        .iter()
        .filter(|row| row.controller == Controller::Position)
    {
        let velocity = tracking
            .iter()
            .find(|row| {
                row.scene == position.scene
                    && row.query_index == position.query_index
                    && row.controller == Controller::VelocityFf
            })
            .expect("paired velocity-FF tracking row");
        ff_only += usize::from(velocity.full_pass && !position.full_pass);
        pd_only += usize::from(position.full_pass && !velocity.full_pass);
    }
    let penetration_cases = tracking
        .iter()
        .filter(|row| {
            row.settle_penetration_steps + row.path_penetration_steps + row.hold_penetration_steps
                > 0
        })
        .count();
    let max_penetration = tracking
        .iter()
        .map(|row| row.max_penetration_m)
        .fold(0.0f64, f64::max);
    writeln!(
        report,
        "\nPaired full-gate discordance: **{ff_only} FF-pass/PD-fail** trajectories and **{pd_only} PD-pass/FF-fail** trajectories. Canonical planning failures with no replay: **{}**. Sampled execution penetration occurs in **{penetration_cases}/{}** tracking cases; maximum depth is **{max_penetration:.8} m**.\n",
        queries.len() - pooled_solved,
        tracking.len()
    )
    .expect("format paired tracking");

    report.push_str(
        "## Scope and limitations\n\n\
         - The accepted pairs are uniform in compiled joint limits conditional on two collision-free endpoints and at least 0.75-rad separation. They are not uniform in Cartesian workspace or representative of a deployment task distribution.\n\
         - Wilson intervals describe repeated draws from this declared conditional generator only. The generator, scenes, and robot model remain fixed.\n\
         - Direct and RRT collision checks sample joint-space edges at 0.05-rad L2 spacing and use the existing MuJoCo emitted-contact predicate. They do not certify continuous collision avoidance or positive clearance.\n\
         - Tracking uses one canonical successful path per accepted query, the nominal plant only, and sampled `dist < 0` execution checks at 2-ms states. Planning failures receive no tracking replay and remain visible in the denominator.\n\
         - The goal-bias comparison changes one planner field; it is an ablation, not a comparison with an independent planning implementation.\n\
         - There is no randomized plant uncertainty, sensing, localization, grasping, payload, hardware, or sim-to-real evidence in this extension.\n\n\
         ## Reproduce\n\n\
         ```bash\n\
         cargo run --release -p arm-lab-demo --bin randomized_eval -- --write\n\
         cargo run --release -p arm-lab-demo --bin randomized_eval -- --check\n\
         ```\n\n\
         Raw artifacts retain all 300 accepted queries, every canonical and blocked-query replicate plan, and both controller replays for every successful canonical path. `plan_elapsed_ms` is observational wall time and the only field normalized by `--check`.\n",
    );
    report
}

fn docs_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs")
}

fn write_artifacts(queries: &[QueryRow], planning: &[PlanningRow], tracking: &[TrackingRow]) {
    let artifacts = render_artifacts(queries, planning, tracking);
    let docs = docs_dir();
    std::fs::write(
        docs.join("randomized_eval_queries.csv"),
        artifacts.queries_csv,
    )
    .expect("write randomized query CSV");
    std::fs::write(
        docs.join("randomized_eval_planning.csv"),
        artifacts.planning_csv,
    )
    .expect("write randomized planning CSV");
    std::fs::write(
        docs.join("randomized_eval_tracking.csv"),
        artifacts.tracking_csv,
    )
    .expect("write randomized tracking CSV");
    std::fs::write(docs.join("randomized_eval_results.md"), artifacts.report)
        .expect("write randomized Markdown report");
    println!("wrote randomized evaluation query, planning, tracking, and report artifacts");
}

fn check_artifacts(queries: &[QueryRow], planning: &[PlanningRow], tracking: &[TrackingRow]) {
    let generated = render_artifacts(queries, planning, tracking);
    let docs = docs_dir();
    let committed_queries = std::fs::read_to_string(docs.join("randomized_eval_queries.csv"))
        .expect("read randomized query CSV");
    let committed_planning = std::fs::read_to_string(docs.join("randomized_eval_planning.csv"))
        .expect("read randomized planning CSV");
    let committed_tracking = std::fs::read_to_string(docs.join("randomized_eval_tracking.csv"))
        .expect("read randomized tracking CSV");
    let committed_report = std::fs::read_to_string(docs.join("randomized_eval_results.md"))
        .expect("read randomized Markdown report");
    assert_artifact_equal("query CSV", &generated.queries_csv, &committed_queries);
    assert_artifact_equal(
        "planning CSV deterministic fields",
        &normalize_planning_elapsed(&generated.planning_csv),
        &normalize_planning_elapsed(&committed_planning),
    );
    assert_artifact_equal("tracking CSV", &generated.tracking_csv, &committed_tracking);
    assert_artifact_equal("Markdown report", &generated.report, &committed_report);
    println!("randomized artifact check passed; only plan_elapsed_ms was normalized");
}

fn normalize_planning_elapsed(csv: &str) -> String {
    let mut lines = csv.lines();
    let header = lines.next().expect("planning CSV header");
    let columns: Vec<_> = header.split(',').collect();
    let elapsed_index = columns
        .iter()
        .position(|column| *column == "plan_elapsed_ms")
        .expect("plan_elapsed_ms column");
    let mut normalized = format!("{header}\n");
    for line in lines {
        let mut fields: Vec<_> = line.split(',').collect();
        assert_eq!(fields.len(), columns.len(), "planning CSV column count");
        fields[elapsed_index] = "<ignored-wall-clock>";
        writeln!(normalized, "{}", fields.join(",")).expect("normalize planning row");
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
    fn frozen_cohort_layout_is_three_by_one_hundred() {
        assert_eq!(SCENES.len(), 3);
        assert_eq!(QUERIES_PER_SCENE, 100);
        assert_eq!(BLOCKED_REPLICATES, 5);
    }

    #[test]
    fn planner_seeds_have_disjoint_scene_and_query_ranges() {
        assert_eq!(planner_seed(0, 0, 0), 202_608_180_000);
        assert_eq!(planner_seed(0, 99, 4), 202_608_180_994);
        assert_eq!(planner_seed(1, 0, 0), 202_608_190_000);
        assert_eq!(planner_seed(2, 99, 4), 202_608_200_994);
    }

    #[test]
    fn wilson_interval_matches_known_half_sample() {
        let (lower, upper) = wilson_interval(50, 100);
        assert!((lower - 0.403_831_53).abs() < 1e-8);
        assert!((upper - 0.596_168_47).abs() < 1e-8);
    }

    #[test]
    fn numeric_and_penetration_gates_remain_separate() {
        assert!(numeric_gates_pass(0.01, 0.02, 0.01));
        assert!(!numeric_gates_pass(0.031, 0.02, 0.01));
        assert!(full_gates_pass(0.01, 0.02, 0.01, 0));
        assert!(!full_gates_pass(0.01, 0.02, 0.01, 1));
        assert_eq!(EXECUTION_CONTACT_THRESHOLD_M, 0.0);
    }

    #[test]
    fn empty_rate_is_reported_without_hiding_a_failed_evaluation() {
        assert_eq!(format_rate(0, 0), "0/0 (not evaluated)");
    }

    #[test]
    fn planning_normalizer_ignores_only_wall_clock() {
        let header = "scene,plan_elapsed_ms,status\n";
        let baseline = format!("{header}open,1.25,success\n");
        let timing = format!("{header}open,99.0,success\n");
        let outcome = format!("{header}open,1.25,unconnected\n");
        assert_eq!(
            normalize_planning_elapsed(&baseline),
            normalize_planning_elapsed(&timing)
        );
        assert_ne!(
            normalize_planning_elapsed(&baseline),
            normalize_planning_elapsed(&outcome)
        );
    }

    #[test]
    #[should_panic(expected = "stale query CSV")]
    fn artifact_comparison_rejects_stale_content() {
        assert_artifact_equal("query CSV", "expected\n", "stale\n");
    }
}
