//! Deterministic multi-scene, multi-query integration benchmark.
//!
//! This executable complements, rather than replaces, the single-trajectory
//! controller robustness envelope. It evaluates five planner seeds on nine
//! fixed scene-query fixtures (six unique joint-pair definitions) across all
//! three shipped UR5e scenes. The canonical-seed trajectory for every fixture
//! is then replayed with position PD and with position PD plus desired-velocity
//! feedforward, both on the nominal plant and on the same fixed combined plant
//! shift used by `robustness_bench`.
//!
//! ```text
//! cargo run --release -p arm-lab-demo --bin multi_query_bench
//! cargo run --release -p arm-lab-demo --bin multi_query_bench -- --write
//! cargo run --release -p arm-lab-demo --bin multi_query_bench -- --check
//! ```

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::path::Path;

use arm_lab::plan::{PlanStatus, edge_free, rrt_connect};
use arm_lab::traj::{TrajLimits, Trajectory, time_parameterize};
use arm_lab::{Chain, CollisionChecker, PlanConfig};
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
const JOINTS: [&str; 6] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
];

const CANONICAL_SEED: u64 = 20260816;
const SEEDS: [u64; 5] = [20260816, 20260817, 20260818, 20260819, 20260820];
const KV_OVER_KP: f64 = 0.2;
const SETTLE_STEPS: usize = 250;
const HOLD_STEPS: usize = 250;
const PASS_RMS_RAD: f64 = 0.03;
const PASS_MAX_RAD: f64 = 0.10;
const PASS_FINAL_RAD: f64 = 0.02;
/// Execution audit counts only emitted robot contacts with signed distance
/// below zero: sampled geometric penetration, not a positive clearance claim.
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

#[derive(Clone, Copy)]
struct Query {
    scene: usize,
    name: &'static str,
    start_delta: [f64; 6],
    goal_delta: [f64; 6],
}

// Hand-designed, fixed joint-space fixtures. Three queries have obstructed
// straight interpolants: offset_pillar/positive_pan and both cross-workspace
// queries in tabletop_pillar. All endpoints are collision-free by assertion.
const QUERIES: [Query; 9] = [
    Query {
        scene: 0,
        name: "positive_pan",
        start_delta: [0.0; 6],
        goal_delta: [1.10, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    Query {
        scene: 0,
        name: "shoulder_elbow",
        start_delta: [0.0; 6],
        goal_delta: [0.35, 0.40, -0.55, 0.25, 0.0, 0.0],
    },
    Query {
        scene: 0,
        name: "wrist_reorientation",
        start_delta: [0.0; 6],
        goal_delta: [0.25, -0.15, 0.25, -0.35, 0.60, -0.70],
    },
    Query {
        scene: 1,
        name: "positive_pan",
        start_delta: [0.0; 6],
        goal_delta: [1.10, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    Query {
        scene: 1,
        name: "negative_pan",
        start_delta: [0.0; 6],
        goal_delta: [-1.00, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    Query {
        scene: 1,
        name: "shoulder_elbow",
        start_delta: [0.0; 6],
        goal_delta: [0.35, 0.40, -0.55, 0.25, 0.0, 0.0],
    },
    Query {
        scene: 2,
        name: "cross_workspace",
        start_delta: [-0.40, 0.15, -0.20, 0.10, 0.0, 0.0],
        goal_delta: [0.75, 0.25, -0.45, 0.25, 0.15, -0.20],
    },
    Query {
        scene: 2,
        name: "reverse_cross_workspace",
        start_delta: [0.65, 0.20, -0.35, 0.15, 0.10, -0.20],
        goal_delta: [-0.45, 0.10, -0.10, -0.10, -0.20, 0.35],
    },
    Query {
        scene: 2,
        name: "wrist_reorientation",
        start_delta: [0.0; 6],
        goal_delta: [0.25, -0.15, 0.25, -0.35, 0.60, -0.70],
    },
];

#[derive(Clone, Copy)]
struct PlantScenario {
    name: &'static str,
    payload_kg: f64,
    actuator_scale: f64,
    extra_damping: f64,
    delay_ms: f64,
    pulse_nm: f64,
}

const PLANTS: [PlantScenario; 2] = [
    PlantScenario {
        name: "nominal",
        payload_kg: 0.0,
        actuator_scale: 1.0,
        extra_damping: 0.0,
        delay_ms: 0.0,
        pulse_nm: 0.0,
    },
    PlantScenario {
        name: "combined moderate shift",
        payload_kg: 1.0,
        actuator_scale: 0.80,
        extra_damping: 1.0,
        delay_ms: 10.0,
        pulse_nm: 10.0,
    },
];

#[derive(Clone, Copy)]
enum Controller {
    Position,
    VelocityFf,
}

const CONTROLLERS: [Controller; 2] = [Controller::Position, Controller::VelocityFf];

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

struct PlanningRow {
    scene: &'static str,
    scene_file: &'static str,
    query: &'static str,
    seed: u64,
    start: Vec<f64>,
    goal: Vec<f64>,
    direct_free: bool,
    status: PlanStatus,
    elapsed_ms: f64,
    iterations: usize,
    nodes: usize,
    shortcut_waypoints: usize,
    path_samples: usize,
    path_cost_rad: f64,
    trajectory_samples: usize,
    trajectory_duration_s: f64,
}

#[derive(Clone)]
struct TrackingRow {
    scene: &'static str,
    scene_file: &'static str,
    query: &'static str,
    direct_free: bool,
    plant: &'static str,
    controller: &'static str,
    trajectory_samples: usize,
    trajectory_duration_s: f64,
    rms_joint_rad: f64,
    max_joint_rad: f64,
    final_joint_rad: f64,
    max_ee_pos_m: f64,
    peak_force_fraction: f64,
    saturated_step_fraction: f64,
    settle_collisions: CollisionPhaseMetrics,
    path_collisions: CollisionPhaseMetrics,
    hold_collisions: CollisionPhaseMetrics,
    pass: bool,
}

impl TrackingRow {
    fn penetration_steps(&self) -> usize {
        self.settle_collisions.steps + self.path_collisions.steps + self.hold_collisions.steps
    }

    fn max_penetration_m(&self) -> f64 {
        self.settle_collisions
            .max_penetration_m
            .max(self.path_collisions.max_penetration_m)
            .max(self.hold_collisions.max_penetration_m)
    }
}

#[derive(Clone, Default)]
struct CollisionPhaseMetrics {
    steps: usize,
    max_penetration_m: f64,
    worst_contact_distance_m: Option<f64>,
    worst_contact: Option<String>,
}

impl CollisionPhaseMetrics {
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
                self.worst_contact_distance_m = Some(contact.distance_m);
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
        _ => panic!("usage: multi_query_bench [--write|--check]"),
    }
}

struct DelayedCommand {
    ctrl: Vec<f64>,
}

fn main() {
    let mode = parse_mode();
    let mut planning_rows = Vec::with_capacity(QUERIES.len() * SEEDS.len());
    let mut tracking_rows = Vec::with_capacity(QUERIES.len() * PLANTS.len() * CONTROLLERS.len());

    for (scene_index, scene) in SCENES.iter().copied().enumerate() {
        let nominal_model = load_model(scene);
        let nominal_chain = extract_chain(&nominal_model);
        let home = home_configuration(&nominal_model, &nominal_chain);

        for query in QUERIES.iter().copied().filter(|q| q.scene == scene_index) {
            let start = offset(&home, query.start_delta);
            let goal = offset(&home, query.goal_delta);
            assert_within_limits(&nominal_chain, &start, "start", scene, query);
            assert_within_limits(&nominal_chain, &goal, "goal", scene, query);

            let mut collision = CollisionChecker::new(&nominal_model, &nominal_chain);
            assert!(
                !collision.collides(&start),
                "{}/{} start is in collision",
                scene.name,
                query.name
            );
            assert!(
                !collision.collides(&goal),
                "{}/{} goal is in collision",
                scene.name,
                query.name
            );
            let mut scratch = vec![0.0; nominal_chain.dof()];
            let direct_free = edge_free(
                &start,
                &goal,
                PlanConfig::default().resolution,
                &mut |q| collision.collides(q),
                &mut scratch,
            );

            let mut canonical_trajectory = None;
            for seed in SEEDS {
                let mut collision = CollisionChecker::new(&nominal_model, &nominal_chain);
                let plan = rrt_connect(
                    &nominal_chain,
                    &start,
                    &goal,
                    |q| collision.collides(q),
                    &PlanConfig {
                        seed,
                        ..PlanConfig::default()
                    },
                );
                assert_eq!(
                    plan.status,
                    PlanStatus::Success,
                    "{}/{} failed at seed {}",
                    scene.name,
                    query.name,
                    seed
                );
                let trajectory =
                    time_parameterize(&plan.path, &LIMITS, nominal_model.opt().timestep);
                if seed == CANONICAL_SEED {
                    canonical_trajectory = Some(trajectory.clone());
                }
                planning_rows.push(PlanningRow {
                    scene: scene.name,
                    scene_file: scene.file,
                    query: query.name,
                    seed,
                    start: start.clone(),
                    goal: goal.clone(),
                    direct_free,
                    status: plan.status,
                    elapsed_ms: 1e3 * plan.elapsed_s,
                    iterations: plan.iterations,
                    nodes: plan.nodes,
                    shortcut_waypoints: plan.waypoints.len(),
                    path_samples: plan.path.len(),
                    path_cost_rad: plan.cost,
                    trajectory_samples: trajectory.len(),
                    trajectory_duration_s: trajectory.duration,
                });
            }

            let trajectory = canonical_trajectory.expect("canonical-seed trajectory");
            println!(
                "{:<18} {:<24} direct={} · {} samples / {:.2} s",
                scene.name,
                query.name,
                if direct_free { "free" } else { "blocked" },
                trajectory.len(),
                trajectory.duration
            );

            for plant_scenario in PLANTS {
                let plant_model = perturbed_model(scene, plant_scenario);
                let plant_chain = extract_chain(&plant_model);
                for controller in CONTROLLERS {
                    let metrics = run_tracking_case(
                        scene,
                        query,
                        direct_free,
                        plant_scenario,
                        controller,
                        &plant_model,
                        &plant_chain,
                        &trajectory,
                    );
                    println!(
                        "  {:<23} | {:<16} | rms {:>7.4} | max {:>7.4} | final {:>7.4} | penetration steps {:>4} | pen {:>7.3} mm | {}",
                        metrics.plant,
                        metrics.controller,
                        metrics.rms_joint_rad,
                        metrics.max_joint_rad,
                        metrics.final_joint_rad,
                        metrics.penetration_steps(),
                        1e3 * metrics.max_penetration_m(),
                        if metrics.pass { "PASS" } else { "FAIL" }
                    );
                    tracking_rows.push(metrics);
                }
            }
        }
    }

    let blocked_queries = planning_rows
        .iter()
        .filter(|row| row.seed == CANONICAL_SEED && !row.direct_free)
        .count();
    let direct_trials = planning_rows.iter().filter(|row| row.direct_free).count();
    let direct_successes = planning_rows
        .iter()
        .filter(|row| row.direct_free && row.status == PlanStatus::Success)
        .count();
    let obstructed_trials = planning_rows.iter().filter(|row| !row.direct_free).count();
    let obstructed_successes = planning_rows
        .iter()
        .filter(|row| !row.direct_free && row.status == PlanStatus::Success)
        .count();
    let velocity_passes = tracking_rows
        .iter()
        .filter(|row| row.controller == Controller::VelocityFf.name() && row.pass)
        .count();
    println!(
        "summary: direct-free {direct_successes}/{direct_trials}; obstructed {obstructed_successes}/{obstructed_trials}; {blocked_queries}/{} blocked-direct fixtures; {velocity_passes}/{} velocity-FF passes",
        QUERIES.len(),
        QUERIES.len() * PLANTS.len()
    );

    match mode {
        Mode::Run => {}
        Mode::Write => write_artifacts(&planning_rows, &tracking_rows),
        Mode::Check => check_artifacts(&planning_rows, &tracking_rows),
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

fn home_configuration(model: &MjModel, chain: &Chain) -> Vec<f64> {
    let mut data = MjData::new(model);
    let home_key = model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("home keyframe");
    data.reset_keyframe(home_key).expect("reset to home");
    data.forward();
    read_q(&data, chain)
}

fn offset(home: &[f64], delta: [f64; 6]) -> Vec<f64> {
    home.iter().zip(delta).map(|(q, dq)| q + dq).collect()
}

fn assert_within_limits(chain: &Chain, q: &[f64], endpoint: &str, scene: Scene, query: Query) {
    for (index, (&value, limits)) in q.iter().zip(chain.joint_limits()).enumerate() {
        if let Some((lower, upper)) = limits {
            assert!(
                (lower..=upper).contains(&value),
                "{}/{} {} joint {} is outside [{}, {}]",
                scene.name,
                query.name,
                endpoint,
                index,
                lower,
                upper
            );
        }
    }
}

fn perturbed_model(scene: Scene, scenario: PlantScenario) -> MjModel {
    let path = format!("{ASSET_DIR}{}", scene.file);
    let mut spec = MjSpec::from_xml(&path).expect("load editable UR5e scene");

    if scenario.payload_kg > 0.0 {
        let payload = spec
            .body_mut("wrist_3_link")
            .expect("wrist body")
            .add_body();
        payload
            .set_name("multi_query_payload")
            .expect("valid payload name");
        payload.set_explicitinertial(true);
        payload.set_mass(scenario.payload_kg);
        payload.pos_mut().copy_from_slice(&[0.0, 0.10, 0.0]);
        let inertia = 0.0017 * scenario.payload_kg;
        payload
            .inertia_mut()
            .copy_from_slice(&[inertia, inertia, inertia]);
    }

    if scenario.extra_damping > 0.0 {
        for name in JOINTS {
            spec.joint_mut(name).expect("benchmark joint").damping_mut()[0] +=
                scenario.extra_damping;
        }
    }

    if scenario.actuator_scale != 1.0 {
        for name in ACTUATORS {
            let actuator = spec.actuator_mut(name).expect("benchmark actuator");
            actuator.gainprm_mut()[0] *= scenario.actuator_scale;
            actuator.biasprm_mut()[1] *= scenario.actuator_scale;
            actuator.biasprm_mut()[2] *= scenario.actuator_scale;
        }
    }

    spec.compile().expect("compile perturbed UR5e model")
}

#[allow(clippy::too_many_arguments)]
fn run_tracking_case(
    scene: Scene,
    query: Query,
    direct_free: bool,
    scenario: PlantScenario,
    controller: Controller,
    model: &MjModel,
    chain: &Chain,
    trajectory: &Trajectory,
) -> TrackingRow {
    let mut data = MjData::new(model);
    let home_key = model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("home keyframe");
    data.reset_keyframe(home_key).expect("reset to home");

    let q_start = trajectory.q.first().expect("trajectory start");
    for (&address, &value) in chain.qpos_addresses().iter().zip(q_start) {
        data.qpos_mut()[address] = value;
    }
    for address in chain.dof_addresses() {
        data.qvel_mut()[address] = 0.0;
    }
    data.forward();

    let dt = model.opt().timestep;
    let delay_steps = (scenario.delay_ms * 1e-3 / dt).round() as usize;
    let zero = vec![0.0; chain.dof()];
    let mut queue = VecDeque::with_capacity(delay_steps + 1);
    let mut collision_checker = CollisionChecker::new(model, chain);
    collision_checker.contact_threshold = EXECUTION_CONTACT_THRESHOLD_M;
    let mut settle_collisions = CollisionPhaseMetrics::default();
    let mut path_collisions = CollisionPhaseMetrics::default();
    let mut hold_collisions = CollisionPhaseMetrics::default();
    for _ in 0..delay_steps {
        queue.push_back(DelayedCommand {
            ctrl: q_start.clone(),
        });
    }

    for _ in 0..SETTLE_STEPS {
        control_step(
            controller, scenario, &mut data, chain, q_start, &zero, &mut queue, false,
        );
        settle_collisions.observe(&mut collision_checker, &read_q(&data, chain));
    }

    let mut sum_sq = 0.0;
    let mut samples = 0usize;
    let mut max_joint = 0.0f64;
    let mut max_ee = 0.0f64;
    let mut peak_force_fraction = 0.0f64;
    let mut saturated_steps = 0usize;
    let pulse_start = trajectory.len() * 45 / 100;
    let pulse_steps = (0.120 / dt).round() as usize;

    for (index, (q_des, qd_des)) in trajectory.q.iter().zip(&trajectory.qd).enumerate() {
        let pulse =
            scenario.pulse_nm != 0.0 && (pulse_start..pulse_start + pulse_steps).contains(&index);
        control_step(
            controller, scenario, &mut data, chain, q_des, qd_des, &mut queue, pulse,
        );

        let q_measured = read_q(&data, chain);
        path_collisions.observe(&mut collision_checker, &q_measured);
        let error = l2(&q_measured, q_des);
        sum_sq += error * error;
        samples += 1;
        max_joint = max_joint.max(error);
        let ee_error = (arm_lab::kinematics::fk(chain, &q_measured)
            .translation
            .vector
            - arm_lab::kinematics::fk(chain, q_des).translation.vector)
            .norm();
        max_ee = max_ee.max(ee_error);
        let fraction = force_fraction(model, &data);
        peak_force_fraction = peak_force_fraction.max(fraction);
        saturated_steps += usize::from(fraction >= 0.999);
    }

    let q_goal = trajectory.q.last().expect("trajectory goal");
    for _ in 0..HOLD_STEPS {
        control_step(
            controller, scenario, &mut data, chain, q_goal, &zero, &mut queue, false,
        );
        hold_collisions.observe(&mut collision_checker, &read_q(&data, chain));
    }
    let final_joint = l2(&read_q(&data, chain), q_goal);
    let rms_joint = (sum_sq / samples as f64).sqrt();
    let penetration_steps = settle_collisions.steps + path_collisions.steps + hold_collisions.steps;
    let pass = case_pass(rms_joint, max_joint, final_joint, penetration_steps);

    TrackingRow {
        scene: scene.name,
        scene_file: scene.file,
        query: query.name,
        direct_free,
        plant: scenario.name,
        controller: controller.name(),
        trajectory_samples: trajectory.len(),
        trajectory_duration_s: trajectory.duration,
        rms_joint_rad: rms_joint,
        max_joint_rad: max_joint,
        final_joint_rad: final_joint,
        max_ee_pos_m: max_ee,
        peak_force_fraction,
        saturated_step_fraction: saturated_steps as f64 / samples as f64,
        settle_collisions,
        path_collisions,
        hold_collisions,
        pass,
    }
}

#[allow(clippy::too_many_arguments)]
fn control_step(
    controller: Controller,
    scenario: PlantScenario,
    data: &mut MjData<&MjModel>,
    chain: &Chain,
    q_des: &[f64],
    qd_des: &[f64],
    queue: &mut VecDeque<DelayedCommand>,
    pulse: bool,
) {
    let mut ctrl = q_des.to_vec();
    if controller.velocity_ff() {
        for (command, velocity) in ctrl.iter_mut().zip(qd_des) {
            *command += KV_OVER_KP * velocity;
        }
    }
    queue.push_back(DelayedCommand { ctrl });
    let command = queue.pop_front().expect("delayed command");

    set_ctrl(data, &ACTUATORS, &command.ctrl);
    data.qfrc_applied_mut().fill(0.0);
    if pulse {
        data.qfrc_applied_mut()[chain.dof_addresses()[1]] -= scenario.pulse_nm;
    }
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

fn l2(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn numeric_gates_pass(row: &TrackingRow) -> bool {
    row.rms_joint_rad <= PASS_RMS_RAD
        && row.max_joint_rad <= PASS_MAX_RAD
        && row.final_joint_rad <= PASS_FINAL_RAD
}

fn case_pass(rms_joint: f64, max_joint: f64, final_joint: f64, penetration_steps: usize) -> bool {
    rms_joint <= PASS_RMS_RAD
        && max_joint <= PASS_MAX_RAD
        && final_joint <= PASS_FINAL_RAD
        && penetration_steps == 0
}

fn worst_collision(row: &TrackingRow) -> (&'static str, &CollisionPhaseMetrics) {
    [
        ("settle", &row.settle_collisions),
        ("path", &row.path_collisions),
        ("hold", &row.hold_collisions),
    ]
    .into_iter()
    .max_by(|(_, left), (_, right)| left.max_penetration_m.total_cmp(&right.max_penetration_m))
    .expect("three collision phases")
}

fn format_optional_distance(value: Option<f64>) -> String {
    value.map_or_else(String::new, |distance| format!("{distance:.8}"))
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

struct Artifacts {
    planning_csv: String,
    tracking_csv: String,
    report: String,
}

fn render_artifacts(planning: &[PlanningRow], tracking: &[TrackingRow]) -> Artifacts {
    let mut planning_csv = String::from(
        "scene,scene_file,query,seed,start_q_rad,goal_q_rad,direct_path_free,status,plan_elapsed_ms,iterations,nodes,shortcut_waypoints,path_samples,path_cost_rad,trajectory_samples,trajectory_duration_s\n",
    );
    for row in planning {
        writeln!(
            planning_csv,
            "{},{},{},{},{},{},{},{},{:.8},{},{},{},{},{:.8},{},{:.8}",
            row.scene,
            row.scene_file,
            row.query,
            row.seed,
            format_joint_vector(&row.start),
            format_joint_vector(&row.goal),
            row.direct_free,
            status_name(row.status),
            row.elapsed_ms,
            row.iterations,
            row.nodes,
            row.shortcut_waypoints,
            row.path_samples,
            row.path_cost_rad,
            row.trajectory_samples,
            row.trajectory_duration_s
        )
        .expect("format planning CSV");
    }
    let mut tracking_csv = String::from(
        "scene,scene_file,query,seed,direct_path_free,plant,controller,trajectory_samples,trajectory_duration_s,rms_joint_rad,max_joint_rad,final_joint_rad,max_ee_pos_m,peak_force_fraction,saturated_step_fraction,settle_penetration_steps,settle_max_penetration_m,settle_worst_contact_distance_m,settle_worst_contact,path_penetration_steps,path_max_penetration_m,path_worst_contact_distance_m,path_worst_contact,hold_penetration_steps,hold_max_penetration_m,hold_worst_contact_distance_m,hold_worst_contact,penetration_steps,max_penetration_m,pass\n",
    );
    for row in tracking {
        writeln!(
            tracking_csv,
            "{},{},{},{},{},{},{},{},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{},{:.8},{},{},{},{:.8},{},{},{},{:.8},{},{},{},{:.8},{}",
            row.scene,
            row.scene_file,
            row.query,
            CANONICAL_SEED,
            row.direct_free,
            row.plant,
            row.controller,
            row.trajectory_samples,
            row.trajectory_duration_s,
            row.rms_joint_rad,
            row.max_joint_rad,
            row.final_joint_rad,
            row.max_ee_pos_m,
            row.peak_force_fraction,
            row.saturated_step_fraction,
            row.settle_collisions.steps,
            row.settle_collisions.max_penetration_m,
            format_optional_distance(row.settle_collisions.worst_contact_distance_m),
            row.settle_collisions.worst_contact.as_deref().unwrap_or(""),
            row.path_collisions.steps,
            row.path_collisions.max_penetration_m,
            format_optional_distance(row.path_collisions.worst_contact_distance_m),
            row.path_collisions.worst_contact.as_deref().unwrap_or(""),
            row.hold_collisions.steps,
            row.hold_collisions.max_penetration_m,
            format_optional_distance(row.hold_collisions.worst_contact_distance_m),
            row.hold_collisions.worst_contact.as_deref().unwrap_or(""),
            row.penetration_steps(),
            row.max_penetration_m(),
            row.pass
        )
        .expect("format tracking CSV");
    }

    let direct_trials = planning.iter().filter(|row| row.direct_free).count();
    let direct_successes = planning
        .iter()
        .filter(|row| row.direct_free && row.status == PlanStatus::Success)
        .count();
    let obstructed_trials = planning.iter().filter(|row| !row.direct_free).count();
    let obstructed_successes = planning
        .iter()
        .filter(|row| !row.direct_free && row.status == PlanStatus::Success)
        .count();
    let mut report = format!(
        "# UR5e multi-scene, multi-query benchmark (simulation)\n\n\
         This deterministic extension evaluates **nine fixed scene-query fixtures (three per scene, six unique joint-pair definitions) across {} shipped MJCF scenes**. Repeating selected joint pairs across scenes creates controlled geometry comparisons. Five fixed planner seeds per fixture produce {} planning trials. The canonical-seed trajectory for each fixture is then replayed using two controller variants against the nominal plant and one fixed combined shift, producing {} tracking trials. Three fixtures have collision-blocked straight interpolants. **This is simulation evidence, not hardware validation or a sim-to-real guarantee.**\n\n\
         Planner headline: **{direct_successes}/{direct_trials} direct-free trials** and **{obstructed_successes}/{obstructed_trials} obstructed trials** succeeded.\n\n\
         The numeric tracking limits are reused unchanged from the earlier robustness envelope: temporal RMS six-joint L2 error <= {:.2} rad, maximum error <= {:.2} rad, and final error after a {}-step hold <= {:.2} rad. In addition, a tracking case passes only with **zero sampled robot-penetration steps** across settling, path execution, and hold. The audit uses a contact threshold of exactly 0.0 m and counts only MuJoCo-emitted robot contacts with signed distance `< 0`; signed distance, maximum actual penetration, and worst geom/body pair are retained in the raw CSV. This is not a positive-clearance certificate.\n\n\
         `plan_elapsed_ms` is observational wall-clock data: it is machine- and load-dependent and is **not byte-stable**. The bounded `--check` mode ignores only that column while verifying every committed deterministic planning field, all tracking fields/outcomes, and this report.\n\n\
         ## Planning and trajectory summary\n\n\
         | Scene | Query | Direct interpolant | Planner success | Cost range (rad) | Canonical trajectory |\n\
         |---|---|:---:|:---:|---:|---:|\n",
        SCENES.len(),
        planning.len(),
        tracking.len(),
        PASS_RMS_RAD,
        PASS_MAX_RAD,
        HOLD_STEPS,
        PASS_FINAL_RAD
    );

    for query in QUERIES {
        let scene = SCENES[query.scene];
        let rows: Vec<&PlanningRow> = planning
            .iter()
            .filter(|row| row.scene == scene.name && row.query == query.name)
            .collect();
        let successes = rows
            .iter()
            .filter(|row| row.status == PlanStatus::Success)
            .count();
        let min_cost = rows
            .iter()
            .map(|row| row.path_cost_rad)
            .fold(f64::INFINITY, f64::min);
        let max_cost = rows
            .iter()
            .map(|row| row.path_cost_rad)
            .fold(0.0f64, f64::max);
        let canonical = rows
            .iter()
            .find(|row| row.seed == CANONICAL_SEED)
            .expect("canonical planning row");
        writeln!(
            report,
            "| {} | {} | {} | {}/{} | {:.3}--{:.3} | {} samples / {:.2} s |",
            scene.name,
            query.name,
            if canonical.direct_free {
                "free"
            } else {
                "blocked"
            },
            successes,
            rows.len(),
            min_cost,
            max_cost,
            canonical.trajectory_samples,
            canonical.trajectory_duration_s
        )
        .expect("format planning table");
    }

    report.push_str(
        "\n## Tracking results\n\n\
         Each cell is RMS / maximum / final six-joint L2 error in radians, followed by sampled penetration steps in settle/path/hold and maximum actual penetration. `PASS` requires all three numeric limits and zero penetration steps.\n\n\
         | Scene | Query | Plant | Position PD | PD + velocity FF |\n\
         |---|---|---|---:|---:|\n",
    );
    for query in QUERIES {
        let scene = SCENES[query.scene];
        for plant in PLANTS {
            let position = tracking
                .iter()
                .find(|row| {
                    row.scene == scene.name
                        && row.query == query.name
                        && row.plant == plant.name
                        && row.controller == Controller::Position.name()
                })
                .expect("position row");
            let velocity = tracking
                .iter()
                .find(|row| {
                    row.scene == scene.name
                        && row.query == query.name
                        && row.plant == plant.name
                        && row.controller == Controller::VelocityFf.name()
                })
                .expect("velocity row");
            writeln!(
                report,
                "| {} | {} | {} | {:.4}/{:.4}/{:.4}; p {}/{}/{}; pen {:.3} mm {} | {:.4}/{:.4}/{:.4}; p {}/{}/{}; pen {:.3} mm {} |",
                scene.name,
                query.name,
                plant.name,
                position.rms_joint_rad,
                position.max_joint_rad,
                position.final_joint_rad,
                position.settle_collisions.steps,
                position.path_collisions.steps,
                position.hold_collisions.steps,
                1e3 * position.max_penetration_m(),
                if position.pass { "PASS" } else { "FAIL" },
                velocity.rms_joint_rad,
                velocity.max_joint_rad,
                velocity.final_joint_rad,
                velocity.settle_collisions.steps,
                velocity.path_collisions.steps,
                velocity.hold_collisions.steps,
                1e3 * velocity.max_penetration_m(),
                if velocity.pass { "PASS" } else { "FAIL" }
            )
            .expect("format tracking table");
        }
    }

    let position_passes = tracking
        .iter()
        .filter(|row| row.controller == Controller::Position.name() && row.pass)
        .count();
    let velocity_passes = tracking
        .iter()
        .filter(|row| row.controller == Controller::VelocityFf.name() && row.pass)
        .count();
    let position_numeric_passes = tracking
        .iter()
        .filter(|row| row.controller == Controller::Position.name() && numeric_gates_pass(row))
        .count();
    let velocity_numeric_passes = tracking
        .iter()
        .filter(|row| row.controller == Controller::VelocityFf.name() && numeric_gates_pass(row))
        .count();
    let penetration_cases: Vec<&TrackingRow> = tracking
        .iter()
        .filter(|row| row.penetration_steps() > 0)
        .collect();
    let total_penetration_steps: usize = penetration_cases
        .iter()
        .map(|row| row.penetration_steps())
        .sum();
    let max_penetration_m = penetration_cases
        .iter()
        .map(|row| row.max_penetration_m())
        .fold(0.0f64, f64::max);
    writeln!(
        report,
        "\n## Aggregate result\n\n- Direct-free planner trials: {direct_successes}/{direct_trials} succeeded.\n- Obstructed planner trials: {obstructed_successes}/{obstructed_trials} succeeded.\n- Position PD: {position_numeric_passes}/{} meet the numeric tracking gates; {position_passes}/{} pass after the zero-penetration requirement.\n- PD + velocity feedforward: {velocity_numeric_passes}/{} meet the numeric tracking gates; {velocity_passes}/{} pass after the zero-penetration requirement.\n- Executed-penetration audit: {}/{} cases have sampled robot penetration; {total_penetration_steps} total phase-steps, maximum actual penetration {:.8} m.\n",
        tracking.len() / 2,
        tracking.len() / 2,
        tracking.len() / 2,
        tracking.len() / 2,
        penetration_cases.len(),
        tracking.len(),
        max_penetration_m
    )
    .expect("format aggregate result");

    if !penetration_cases.is_empty() {
        report.push_str(
            "\n## Executed penetration cases\n\n\
             Penetration steps are shown as settle/path/hold. The worst emitted contact is selected by maximum actual penetration; every listed signed distance is below zero.\n\n\
             | Scene | Query | Plant | Controller | Steps S/P/H | Max penetration (m) | Worst phase / signed distance / geom pair |\n\
             |---|---|---|---|---:|---:|---|\n",
        );
        for row in &penetration_cases {
            let (phase, worst) = worst_collision(row);
            writeln!(
                report,
                "| {} | {} | {} | {} | {}/{}/{} | {:.8} | {} / {:.8} / {} |",
                row.scene,
                row.query,
                row.plant,
                row.controller,
                row.settle_collisions.steps,
                row.path_collisions.steps,
                row.hold_collisions.steps,
                row.max_penetration_m(),
                phase,
                worst
                    .worst_contact_distance_m
                    .expect("colliding phase signed distance"),
                worst
                    .worst_contact
                    .as_deref()
                    .expect("colliding phase identity")
            )
            .expect("format collision case");
        }
    }

    writeln!(
        report,
        "\nThe fixed `tabletop_pillar/reverse_cross_workspace` fixture is retained in full, including any penetration-failing outcomes; no fixture or failed case is removed from either artifact."
    )
    .expect("format retained negative statement");

    report.push_str(
        "\n## Exact scope and limitations\n\n\
         - The fixtures are deterministic and hand-designed, not sampled from a scene or query distribution. These counts are not estimates of a workspace-wide success probability.\n\
         - All scenes use the same UR5e model and actuator interface. The open scene contains only a floor; the other two scenes are distinct layouts but both use pillar-like obstacles.\n\
         - Five planner seeds probe sampling variability, but controller tracking uses one canonical path per query. `plan_elapsed_ms` is machine- and load-dependent, is not byte-stable, and is the only field ignored by `--check`.\n\
         - Only position PD and its desired-velocity-feedforward variant are compared here. This extension does not show that nominal-bias or integral-residual results generalize across queries.\n\
         - The combined plant is one deterministic condition: 1 kg payload at 0.10 m, 80% actuator gains, +1 Nms/rad joint damping, 10 ms command latency, and a 10 Nm / 120 ms shoulder-lift pulse at 45% of each trajectory. It is not a randomized uncertainty distribution.\n\
         - Planner collision checks remain discrete at 0.05 rad in joint-space L2 and use MuJoCo's emitted-contact set. The repository's positive contact threshold filters emitted candidates; with these zero-margin geoms it does not establish positive geometric clearance. Planner behavior is retained, but no 1-mm-clearance claim is made.\n\
         - The execution gate checks signed distance `< 0` at each 2-ms simulation state in settle, path, and hold. It detects sampled penetration, not positive-distance near misses or continuous swept-volume collision between samples. Polyline corners remain unblended, so the scalar time law does not certify global acceleration or jerk.\n\
         - There is no sensor noise, contact-rich grasping, hardware experiment, or sim-to-real guarantee.\n\n\
         ## Reproduce\n\n\
         ```bash\n\
         cargo run --release -p arm-lab-demo --bin multi_query_bench -- --write\n\
         cargo run --release -p arm-lab-demo --bin multi_query_bench -- --check\n\
         ```\n\n\
         Raw artifacts: `docs/multi_query_planning.csv` (all 45 planner trials, including exact joint vectors) and `docs/multi_query_tracking.csv` (all 36 tracking trials, numeric metrics, per-phase penetration counts/depths, and worst emitted-contact identities).\n",
    );

    Artifacts {
        planning_csv,
        tracking_csv,
        report,
    }
}

fn docs_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs")
}

fn write_artifacts(planning: &[PlanningRow], tracking: &[TrackingRow]) {
    let docs = docs_dir();
    std::fs::create_dir_all(&docs).expect("create docs directory");
    let artifacts = render_artifacts(planning, tracking);
    std::fs::write(
        docs.join("multi_query_planning.csv"),
        artifacts.planning_csv,
    )
    .expect("write planning CSV");
    std::fs::write(
        docs.join("multi_query_tracking.csv"),
        artifacts.tracking_csv,
    )
    .expect("write tracking CSV");
    std::fs::write(docs.join("multi_query_results.md"), artifacts.report)
        .expect("write Markdown report");
    println!(
        "wrote docs/multi_query_planning.csv, docs/multi_query_tracking.csv, and docs/multi_query_results.md"
    );
}

fn check_artifacts(planning: &[PlanningRow], tracking: &[TrackingRow]) {
    let docs = docs_dir();
    let generated = render_artifacts(planning, tracking);
    let committed_planning = std::fs::read_to_string(docs.join("multi_query_planning.csv"))
        .expect("read committed planning CSV");
    let committed_tracking = std::fs::read_to_string(docs.join("multi_query_tracking.csv"))
        .expect("read committed tracking CSV");
    let committed_report = std::fs::read_to_string(docs.join("multi_query_results.md"))
        .expect("read committed Markdown report");

    assert_artifact_equal(
        "planning CSV deterministic fields",
        &normalize_planning_elapsed(&generated.planning_csv),
        &normalize_planning_elapsed(&committed_planning),
    );
    assert_artifact_equal("tracking CSV", &generated.tracking_csv, &committed_tracking);
    assert_artifact_equal("Markdown report", &generated.report, &committed_report);
    println!(
        "artifact check passed: all deterministic fields and outcomes match; plan_elapsed_ms ignored"
    );
}

fn normalize_planning_elapsed(csv: &str) -> String {
    let mut lines = csv.lines();
    let header = lines.next().expect("planning CSV header");
    let columns: Vec<&str> = header.split(',').collect();
    let elapsed_index = columns
        .iter()
        .position(|column| *column == "plan_elapsed_ms")
        .expect("plan_elapsed_ms column");
    let mut normalized = format!("{header}\n");
    for line in lines {
        let mut fields: Vec<&str> = line.split(',').collect();
        assert_eq!(
            fields.len(),
            columns.len(),
            "planning CSV row has wrong column count"
        );
        fields[elapsed_index] = "<ignored-wall-clock>";
        writeln!(normalized, "{}", fields.join(",")).expect("normalize planning CSV");
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
    panic!(
        "stale {label}: first mismatch at line {mismatch}; rerun with --write and audit changes"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn benchmark_layout_is_three_scenes_by_three_queries() {
        assert_eq!(SCENES.len(), 3);
        assert_eq!(QUERIES.len(), 9);
        for index in 0..SCENES.len() {
            assert_eq!(
                QUERIES.iter().filter(|query| query.scene == index).count(),
                3
            );
        }
    }

    #[test]
    fn canonical_seed_is_in_planner_seed_set() {
        assert!(SEEDS.contains(&CANONICAL_SEED));
    }

    #[test]
    fn repeated_fixtures_leave_six_unique_joint_pairs() {
        let mut unique = Vec::new();
        for query in QUERIES {
            let pair = (query.start_delta, query.goal_delta);
            if !unique.contains(&pair) {
                unique.push(pair);
            }
        }
        assert_eq!(unique.len(), 6);
    }

    #[test]
    fn artifact_normalizer_ignores_only_wall_clock_column() {
        let header = "scene,plan_elapsed_ms,status\n";
        let first = format!("{header}open,1.25000000,success\n");
        let timing_only = format!("{header}open,99.00000000,success\n");
        let stale_status = format!("{header}open,1.25000000,unconnected\n");
        assert_eq!(
            normalize_planning_elapsed(&first),
            normalize_planning_elapsed(&timing_only)
        );
        assert_ne!(
            normalize_planning_elapsed(&first),
            normalize_planning_elapsed(&stale_status)
        );
    }

    #[test]
    fn execution_audit_uses_zero_threshold_and_actual_penetration() {
        let signed_distance: f64 = -0.004;
        let penetration = (-signed_distance).max(0.0);
        assert_eq!(EXECUTION_CONTACT_THRESHOLD_M, 0.0);
        assert!((penetration - 0.004).abs() < 1e-12);
    }

    #[test]
    fn penetration_step_invalidates_otherwise_passing_case() {
        assert!(case_pass(0.01, 0.02, 0.01, 0));
        assert!(!case_pass(0.01, 0.02, 0.01, 1));
    }

    #[test]
    #[should_panic(expected = "stale fixture")]
    fn artifact_comparison_rejects_stale_deterministic_content() {
        assert_artifact_equal("fixture", "expected\n", "stale\n");
    }
}
