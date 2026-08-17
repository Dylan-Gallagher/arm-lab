//! Deterministic simulation-side robustness envelope for the UR5e tracker.
//!
//! The same collision-free path and per-edge corner-stop S-curves are replayed
//! against payload, actuator, damping, command-latency, and disturbance shifts.
//! The model-based controllers only receive bias forces from the unperturbed
//! nominal model, so perturbed runs do not leak the changed plant parameters
//! into the controller.
//!
//! ```text
//! cargo run --release -p arm-lab-demo --bin robustness_bench
//! cargo run --release -p arm-lab-demo --bin robustness_bench -- --write
//! ```

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::path::Path;

use arm_lab::plan::{PlanStatus, rrt_connect};
use arm_lab::traj::{TrajLimits, Trajectory, time_parameterize};
use arm_lab::{Chain, CollisionChecker, PlanConfig};
use arm_lab_demo::{read_q, set_ctrl};
use mujoco_rs::prelude::*;

const SCENE_XML: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../assets/ur5e/scene_cluttered.xml"
);

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

const SEED: u64 = 20260816;
const KV_OVER_KP: f64 = 0.2;
const SETTLE_STEPS: usize = 250;
const HOLD_STEPS: usize = 250;
const INTEGRAL_GAIN: f64 = 2.0;
const INTEGRAL_CTRL_LIMIT: f64 = 0.04;
const PASS_RMS_RAD: f64 = 0.03;
const PASS_MAX_RAD: f64 = 0.10;
const PASS_FINAL_RAD: f64 = 0.02;

#[derive(Clone, Copy)]
struct Scenario {
    name: &'static str,
    payload_kg: f64,
    actuator_scale: f64,
    extra_damping: f64,
    delay_ms: f64,
    pulse_nm: f64,
}

const SCENARIOS: [Scenario; 7] = [
    Scenario {
        name: "nominal",
        payload_kg: 0.0,
        actuator_scale: 1.0,
        extra_damping: 0.0,
        delay_ms: 0.0,
        pulse_nm: 0.0,
    },
    Scenario {
        name: "2 kg tool payload",
        payload_kg: 2.0,
        actuator_scale: 1.0,
        extra_damping: 0.0,
        delay_ms: 0.0,
        pulse_nm: 0.0,
    },
    Scenario {
        name: "65% actuator gains",
        payload_kg: 0.0,
        actuator_scale: 0.65,
        extra_damping: 0.0,
        delay_ms: 0.0,
        pulse_nm: 0.0,
    },
    Scenario {
        name: "+2 Nms/rad joint damping",
        payload_kg: 0.0,
        actuator_scale: 1.0,
        extra_damping: 2.0,
        delay_ms: 0.0,
        pulse_nm: 0.0,
    },
    Scenario {
        name: "20 ms command latency",
        payload_kg: 0.0,
        actuator_scale: 1.0,
        extra_damping: 0.0,
        delay_ms: 20.0,
        pulse_nm: 0.0,
    },
    Scenario {
        name: "20 Nm / 120 ms torque pulse",
        payload_kg: 0.0,
        actuator_scale: 1.0,
        extra_damping: 0.0,
        delay_ms: 0.0,
        pulse_nm: 20.0,
    },
    Scenario {
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
    NominalBias,
    ResidualIntegral,
}

const CONTROLLERS: [Controller; 4] = [
    Controller::Position,
    Controller::VelocityFf,
    Controller::NominalBias,
    Controller::ResidualIntegral,
];

impl Controller {
    fn name(self) -> &'static str {
        match self {
            Self::Position => "position PD",
            Self::VelocityFf => "PD + velocity FF",
            Self::NominalBias => "PD + velocity FF + nominal bias",
            Self::ResidualIntegral => "nominal bias + integral residual",
        }
    }

    fn velocity_ff(self) -> bool {
        !matches!(self, Self::Position)
    }

    fn nominal_bias(self) -> bool {
        matches!(self, Self::NominalBias | Self::ResidualIntegral)
    }

    fn integral(self) -> bool {
        matches!(self, Self::ResidualIntegral)
    }
}

struct DelayedCommand {
    ctrl: Vec<f64>,
    bias: Vec<f64>,
}

#[derive(Clone)]
struct Metrics {
    scenario: &'static str,
    controller: &'static str,
    rms_joint_rad: f64,
    max_joint_rad: f64,
    final_joint_rad: f64,
    max_ee_pos_m: f64,
    peak_force_fraction: f64,
    saturated_step_fraction: f64,
    pass: bool,
}

fn main() {
    let write_results = std::env::args().any(|arg| arg == "--write");

    let nominal_model = MjModel::from_xml(SCENE_XML).expect("load nominal UR5e scene");
    let nominal_chain = extract_chain(&nominal_model);
    let trajectory = benchmark_trajectory(&nominal_model, &nominal_chain);
    println!(
        "robustness envelope: {} samples, {:.2} s, {} scenarios x {} controllers",
        trajectory.len(),
        trajectory.duration,
        SCENARIOS.len(),
        CONTROLLERS.len()
    );

    let mut rows = Vec::with_capacity(SCENARIOS.len() * CONTROLLERS.len());
    for scenario in SCENARIOS {
        let plant_model = perturbed_model(scenario);
        let plant_chain = extract_chain(&plant_model);
        for controller in CONTROLLERS {
            let metrics = run_case(
                scenario,
                controller,
                &plant_model,
                &plant_chain,
                &nominal_model,
                &nominal_chain,
                &trajectory,
            );
            println!(
                "{:<28} | {:<35} | rms {:>7.4} | max {:>7.4} | final {:>7.4} | ee {:>7.4} | sat {:>5.1}% | {}",
                metrics.scenario,
                metrics.controller,
                metrics.rms_joint_rad,
                metrics.max_joint_rad,
                metrics.final_joint_rad,
                metrics.max_ee_pos_m,
                100.0 * metrics.saturated_step_fraction,
                if metrics.pass { "PASS" } else { "FAIL" }
            );
            rows.push(metrics);
        }
    }

    if write_results {
        write_artifacts(&rows, &trajectory);
    }
}

fn extract_chain(model: &MjModel) -> Chain {
    Chain::from_mujoco(model, "ur5e", "wrist_3_link", "attachment_site")
        .expect("extract UR5e chain")
}

fn benchmark_trajectory(model: &MjModel, chain: &Chain) -> Trajectory {
    let mut data = MjData::new(model);
    let home_key = model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("home keyframe");
    data.reset_keyframe(home_key).expect("reset nominal home");
    data.forward();
    let q_start = read_q(&data, chain);
    let mut q_goal = q_start.clone();
    q_goal[0] += 1.1;

    let mut collision = CollisionChecker::new(model, chain);
    let plan = rrt_connect(
        chain,
        &q_start,
        &q_goal,
        |q| collision.collides(q),
        &PlanConfig {
            seed: SEED,
            ..PlanConfig::default()
        },
    );
    assert_eq!(plan.status, PlanStatus::Success, "benchmark plan failed");
    time_parameterize(
        &plan.waypoints,
        &TrajLimits {
            v_max: 0.55,
            a_max: 1.8,
            j_max: 8.0,
        },
        model.opt().timestep,
    )
}

fn perturbed_model(scenario: Scenario) -> MjModel {
    let mut spec = MjSpec::from_xml(SCENE_XML).expect("load editable UR5e scene");

    if scenario.payload_kg > 0.0 {
        let payload = spec
            .body_mut("wrist_3_link")
            .expect("wrist body")
            .add_body();
        payload
            .set_name("benchmark_payload")
            .expect("valid payload name");
        payload.set_explicitinertial(true);
        payload.set_mass(scenario.payload_kg);
        payload.pos_mut().copy_from_slice(&[0.0, 0.10, 0.0]);
        // Conservative diagonal inertia for a compact tool/load at the flange.
        let inertia = 0.0017 * scenario.payload_kg;
        payload
            .inertia_mut()
            .copy_from_slice(&[inertia, inertia, inertia]);
    }

    if scenario.extra_damping > 0.0 {
        for name in JOINTS {
            let joint = spec.joint_mut(name).expect("benchmark joint");
            joint.damping_mut()[0] += scenario.extra_damping;
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
fn run_case(
    scenario: Scenario,
    controller: Controller,
    plant_model: &MjModel,
    plant_chain: &Chain,
    nominal_model: &MjModel,
    nominal_chain: &Chain,
    trajectory: &Trajectory,
) -> Metrics {
    let mut plant = MjData::new(plant_model);
    let mut nominal = MjData::new(nominal_model);
    let plant_home = plant_model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("plant home keyframe");
    let nominal_home = nominal_model
        .name_to_id(MjtObj::mjOBJ_KEY, "home")
        .expect("nominal home keyframe");
    plant.reset_keyframe(plant_home).expect("reset plant home");
    nominal
        .reset_keyframe(nominal_home)
        .expect("reset nominal home");
    plant.forward();
    nominal.forward();

    let dt = plant_model.opt().timestep;
    let delay_steps = (scenario.delay_ms * 1e-3 / dt).round() as usize;
    let q_home = read_q(&plant, plant_chain);
    let zero = vec![0.0; plant_chain.dof()];
    let mut integral_correction = vec![0.0; plant_chain.dof()];
    let mut queue = VecDeque::with_capacity(delay_steps + 1);
    for _ in 0..delay_steps {
        queue.push_back(DelayedCommand {
            ctrl: q_home.clone(),
            bias: vec![0.0; plant_chain.dof()],
        });
    }

    for _ in 0..SETTLE_STEPS {
        control_step(
            controller,
            scenario,
            &mut plant,
            plant_chain,
            &mut nominal,
            nominal_chain,
            &q_home,
            &zero,
            &mut integral_correction,
            &mut queue,
            false,
        );
    }

    let mut sum_sq = 0.0;
    let mut samples = 0usize;
    let mut max_joint = 0.0f64;
    let mut max_ee = 0.0f64;
    let mut peak_force_fraction = 0.0f64;
    let mut saturated_steps = 0usize;
    let pulse_start = trajectory.len() * 45 / 100;
    let pulse_steps = (0.120 / dt).round() as usize;

    for (i, (q_des, qd_des)) in trajectory.q.iter().zip(&trajectory.qd).enumerate() {
        let pulse =
            scenario.pulse_nm != 0.0 && (pulse_start..pulse_start + pulse_steps).contains(&i);
        control_step(
            controller,
            scenario,
            &mut plant,
            plant_chain,
            &mut nominal,
            nominal_chain,
            q_des,
            qd_des,
            &mut integral_correction,
            &mut queue,
            pulse,
        );

        let q_meas = read_q(&plant, plant_chain);
        let error = l2(&q_meas, q_des);
        sum_sq += error * error;
        samples += 1;
        max_joint = max_joint.max(error);
        let ee_error = (arm_lab::kinematics::fk(plant_chain, &q_meas)
            .translation
            .vector
            - arm_lab::kinematics::fk(plant_chain, q_des)
                .translation
                .vector)
            .norm();
        max_ee = max_ee.max(ee_error);
        let force_fraction = force_fraction(plant_model, &plant);
        peak_force_fraction = peak_force_fraction.max(force_fraction);
        saturated_steps += usize::from(force_fraction >= 0.999);
    }

    let q_goal = trajectory.q.last().expect("trajectory goal");
    for _ in 0..HOLD_STEPS {
        control_step(
            controller,
            scenario,
            &mut plant,
            plant_chain,
            &mut nominal,
            nominal_chain,
            q_goal,
            &zero,
            &mut integral_correction,
            &mut queue,
            false,
        );
    }
    let final_joint = l2(&read_q(&plant, plant_chain), q_goal);
    let rms_joint = (sum_sq / samples as f64).sqrt();
    let pass =
        rms_joint <= PASS_RMS_RAD && max_joint <= PASS_MAX_RAD && final_joint <= PASS_FINAL_RAD;

    Metrics {
        scenario: scenario.name,
        controller: controller.name(),
        rms_joint_rad: rms_joint,
        max_joint_rad: max_joint,
        final_joint_rad: final_joint,
        max_ee_pos_m: max_ee,
        peak_force_fraction,
        saturated_step_fraction: saturated_steps as f64 / samples as f64,
        pass,
    }
}

#[allow(clippy::too_many_arguments)]
fn control_step(
    controller: Controller,
    scenario: Scenario,
    plant: &mut MjData<&MjModel>,
    plant_chain: &Chain,
    nominal: &mut MjData<&MjModel>,
    nominal_chain: &Chain,
    q_des: &[f64],
    qd_des: &[f64],
    integral_correction: &mut [f64],
    queue: &mut VecDeque<DelayedCommand>,
    pulse: bool,
) {
    let q_meas = read_q(plant, plant_chain);
    let dt = plant.model().opt().timestep;
    let mut ctrl = q_des.to_vec();
    if controller.velocity_ff() {
        for (command, velocity) in ctrl.iter_mut().zip(qd_des) {
            *command += KV_OVER_KP * velocity;
        }
    }
    if controller.integral() {
        for ((correction, command), (&desired, &measured)) in integral_correction
            .iter_mut()
            .zip(&mut ctrl)
            .zip(q_des.iter().zip(&q_meas))
        {
            *correction = (*correction + INTEGRAL_GAIN * (desired - measured) * dt)
                .clamp(-INTEGRAL_CTRL_LIMIT, INTEGRAL_CTRL_LIMIT);
            *command += *correction;
        }
    }

    let bias = if controller.nominal_bias() {
        nominal_bias(plant, plant_chain, nominal, nominal_chain)
    } else {
        vec![0.0; plant_chain.dof()]
    };
    queue.push_back(DelayedCommand { ctrl, bias });
    let command = queue.pop_front().expect("delayed command");

    set_ctrl(plant, &ACTUATORS, &command.ctrl);
    plant.qfrc_applied_mut().fill(0.0);
    for (&dof, &force) in plant_chain.dof_addresses().iter().zip(&command.bias) {
        plant.qfrc_applied_mut()[dof] = force;
    }
    if pulse {
        // Shoulder-lift is challenged because it carries most of the gravity load.
        plant.qfrc_applied_mut()[plant_chain.dof_addresses()[1]] -= scenario.pulse_nm;
    }
    plant.step();
}

fn nominal_bias(
    plant: &MjData<&MjModel>,
    plant_chain: &Chain,
    nominal: &mut MjData<&MjModel>,
    nominal_chain: &Chain,
) -> Vec<f64> {
    for (&plant_q, nominal_q) in plant_chain
        .qpos_addresses()
        .iter()
        .zip(nominal_chain.qpos_addresses())
    {
        nominal.qpos_mut()[nominal_q] = plant.qpos()[plant_q];
    }
    for (&plant_d, nominal_d) in plant_chain
        .dof_addresses()
        .iter()
        .zip(nominal_chain.dof_addresses())
    {
        nominal.qvel_mut()[nominal_d] = plant.qvel()[plant_d];
    }
    nominal.forward();
    nominal_chain
        .dof_addresses()
        .iter()
        .map(|&dof| nominal.qfrc_bias()[dof])
        .collect()
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

fn write_artifacts(rows: &[Metrics], trajectory: &Trajectory) {
    let docs = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../docs");
    std::fs::create_dir_all(&docs).expect("create docs directory");

    let mut csv = String::from(
        "scenario,controller,rms_joint_rad,max_joint_rad,final_joint_rad,max_ee_pos_m,peak_force_fraction,saturated_step_fraction,pass\n",
    );
    for row in rows {
        writeln!(
            csv,
            "{},{},{:.8},{:.8},{:.8},{:.8},{:.8},{:.8},{}",
            row.scenario,
            row.controller,
            row.rms_joint_rad,
            row.max_joint_rad,
            row.final_joint_rad,
            row.max_ee_pos_m,
            row.peak_force_fraction,
            row.saturated_step_fraction,
            row.pass
        )
        .expect("format CSV");
    }
    std::fs::write(docs.join("robustness_results.csv"), csv).expect("write CSV");

    let mut markdown = format!(
        "# UR5e controller robustness envelope (simulation)\n\n\
         Deterministic MuJoCo stress test over a {:.2} s, {}-sample collision-free RRT-Connect path timed by the in-repo per-edge S-curves. Every shortcut corner is a full stop: joint position, velocity, and acceleration are continuous, and jerk is bounded almost everywhere, although jerk may jump between finite values and the path is not geometrically blended or time-optimal. **This is simulation evidence, not hardware validation or a sim-to-real guarantee.**\n\n\
         Pass threshold (declared in code): RMS joint error <= {:.2} rad, maximum joint error <= {:.2} rad, and final joint error <= {:.2} rad. Controller bias forces always come from the unchanged nominal model; perturbed plant parameters are not exposed to the controller.\n\n\
         | Scenario | Controller | RMS joint (rad) | Max joint (rad) | Final (rad) | Max EE pos (m) | Peak force/limit | Saturated steps | Result |\n\
         |---|---|---:|---:|---:|---:|---:|---:|:---:|\n",
        trajectory.duration,
        trajectory.len(),
        PASS_RMS_RAD,
        PASS_MAX_RAD,
        PASS_FINAL_RAD
    );
    for row in rows {
        writeln!(
            markdown,
            "| {} | {} | {:.4} | {:.4} | {:.4} | {:.4} | {:.0}% | {:.1}% | {} |",
            row.scenario,
            row.controller,
            row.rms_joint_rad,
            row.max_joint_rad,
            row.final_joint_rad,
            row.max_ee_pos_m,
            100.0 * row.peak_force_fraction,
            100.0 * row.saturated_step_fraction,
            if row.pass { "PASS" } else { "FAIL" }
        )
        .expect("format Markdown table");
    }
    markdown.push_str(
        "\n## Plant shifts\n\n\
         - Payload: a rigid inertial body 0.10 m from the wrist-3 frame.\n\
         - Actuator shift: proportional, derivative, and command gains scaled together; force limits unchanged.\n\
         - Damping: the stated viscous coefficient is added to all six joints.\n\
         - Latency: complete actuator commands, including nominal-model bias, are delayed.\n\
         - Pulse: deterministic external shoulder-lift torque begins at 45% of the path.\n\
         - Combined: 1 kg payload, 80% actuator gains, +1 Nms/rad damping, 10 ms latency, and a 10 Nm / 120 ms pulse.\n\n\
         ## Controllers\n\n\
         - `position PD`: the MuJoCo-Menagerie position actuator without trajectory feedforward.\n\
         - `PD + velocity FF`: adds `(kv/kp) * qd_des` to the position command.\n\
         - `PD + velocity FF + nominal bias`: adds gravity/Coriolis bias computed from an unperturbed model at the measured state.\n\
         - `nominal bias + integral residual`: adds a bounded integral correction (maximum +/-0.04 rad of command offset) to reject persistent mismatch.\n\n\
         ## Reproduce\n\n\
         ```bash\n\
         cargo run --release -p arm-lab-demo --bin robustness_bench -- --write\n\
         ```\n",
    );
    std::fs::write(docs.join("robustness_results.md"), markdown).expect("write Markdown");
    println!("wrote docs/robustness_results.csv and docs/robustness_results.md");
}
