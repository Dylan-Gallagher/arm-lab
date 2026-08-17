# UR5e controller robustness envelope (simulation)

Deterministic MuJoCo stress test over a 3.84 s, 1919-sample collision-free RRT-Connect path timed by the in-repo per-edge S-curves. Every shortcut corner is a full stop: joint position, velocity, and acceleration are continuous, and jerk is bounded almost everywhere, although jerk may jump between finite values and the path is not geometrically blended or time-optimal. **This is simulation evidence, not hardware validation or a sim-to-real guarantee.**

Pass threshold (declared in code): RMS joint error <= 0.03 rad, maximum joint error <= 0.10 rad, and final joint error <= 0.02 rad. Controller bias forces always come from the unchanged nominal model; perturbed plant parameters are not exposed to the controller.

| Scenario | Controller | RMS joint (rad) | Max joint (rad) | Final (rad) | Max EE pos (m) | Peak force/limit | Saturated steps | Result |
|---|---|---:|---:|---:|---:|---:|---:|:---:|
| nominal | position PD | 0.1406 | 0.1899 | 0.0111 | 0.0770 | 12% | 0.0% | FAIL |
| nominal | PD + velocity FF | 0.0093 | 0.0122 | 0.0117 | 0.0086 | 13% | 0.0% | PASS |
| nominal | PD + velocity FF + nominal bias | 0.0015 | 0.0020 | 0.0001 | 0.0014 | 3% | 0.0% | PASS |
| nominal | nominal bias + integral residual | 0.0018 | 0.0025 | 0.0005 | 0.0014 | 3% | 0.0% | PASS |
| 2 kg tool payload | position PD | 0.1412 | 0.1879 | 0.0187 | 0.0817 | 19% | 0.0% | FAIL |
| 2 kg tool payload | PD + velocity FF | 0.0169 | 0.0202 | 0.0193 | 0.0141 | 20% | 0.0% | PASS |
| 2 kg tool payload | PD + velocity FF + nominal bias | 0.0085 | 0.0094 | 0.0079 | 0.0061 | 11% | 0.0% | PASS |
| 2 kg tool payload | nominal bias + integral residual | 0.0027 | 0.0048 | 0.0008 | 0.0032 | 11% | 0.0% | PASS |
| 65% actuator gains | position PD | 0.1413 | 0.1888 | 0.0171 | 0.0799 | 12% | 0.0% | FAIL |
| 65% actuator gains | PD + velocity FF | 0.0144 | 0.0187 | 0.0180 | 0.0134 | 13% | 0.0% | PASS |
| 65% actuator gains | PD + velocity FF + nominal bias | 0.0016 | 0.0026 | 0.0001 | 0.0020 | 3% | 0.0% | PASS |
| 65% actuator gains | nominal bias + integral residual | 0.0020 | 0.0028 | 0.0007 | 0.0019 | 3% | 0.0% | PASS |
| +2 Nms/rad joint damping | position PD | 0.1423 | 0.1924 | 0.0112 | 0.0775 | 12% | 0.0% | FAIL |
| +2 Nms/rad joint damping | PD + velocity FF | 0.0093 | 0.0121 | 0.0117 | 0.0086 | 13% | 0.0% | PASS |
| +2 Nms/rad joint damping | PD + velocity FF + nominal bias | 0.0013 | 0.0017 | 0.0001 | 0.0011 | 5% | 0.0% | PASS |
| +2 Nms/rad joint damping | nominal bias + integral residual | 0.0016 | 0.0025 | 0.0009 | 0.0015 | 5% | 0.0% | PASS |
| 20 ms command latency | position PD | 0.1547 | 0.2093 | 0.0112 | 0.0843 | 12% | 0.0% | FAIL |
| 20 ms command latency | PD + velocity FF | 0.0167 | 0.0232 | 0.0117 | 0.0142 | 13% | 0.0% | PASS |
| 20 ms command latency | PD + velocity FF + nominal bias | 0.0136 | 0.0183 | 0.0001 | 0.0067 | 3% | 0.0% | PASS |
| 20 ms command latency | nominal bias + integral residual | 0.0099 | 0.0181 | 0.0039 | 0.0074 | 3% | 0.0% | PASS |
| 20 Nm / 120 ms torque pulse | position PD | 0.1405 | 0.1899 | 0.0111 | 0.0770 | 15% | 0.0% | FAIL |
| 20 Nm / 120 ms torque pulse | PD + velocity FF | 0.0094 | 0.0122 | 0.0117 | 0.0086 | 16% | 0.0% | PASS |
| 20 Nm / 120 ms torque pulse | PD + velocity FF + nominal bias | 0.0019 | 0.0057 | 0.0001 | 0.0042 | 16% | 0.0% | PASS |
| 20 Nm / 120 ms torque pulse | nominal bias + integral residual | 0.0022 | 0.0053 | 0.0005 | 0.0039 | 16% | 0.0% | PASS |
| combined moderate shift | position PD | 0.1494 | 0.1994 | 0.0187 | 0.0852 | 16% | 0.0% | FAIL |
| combined moderate shift | PD + velocity FF | 0.0180 | 0.0241 | 0.0194 | 0.0168 | 17% | 0.0% | PASS |
| combined moderate shift | PD + velocity FF + nominal bias | 0.0090 | 0.0117 | 0.0050 | 0.0069 | 7% | 0.0% | PASS |
| combined moderate shift | nominal bias + integral residual | 0.0059 | 0.0109 | 0.0022 | 0.0070 | 7% | 0.0% | PASS |

## Plant shifts

- Payload: a rigid inertial body 0.10 m from the wrist-3 frame.
- Actuator shift: proportional, derivative, and command gains scaled together; force limits unchanged.
- Damping: the stated viscous coefficient is added to all six joints.
- Latency: complete actuator commands, including nominal-model bias, are delayed.
- Pulse: deterministic external shoulder-lift torque begins at 45% of the path.
- Combined: 1 kg payload, 80% actuator gains, +1 Nms/rad damping, 10 ms latency, and a 10 Nm / 120 ms pulse.

## Controllers

- `position PD`: the MuJoCo-Menagerie position actuator without trajectory feedforward.
- `PD + velocity FF`: adds `(kv/kp) * qd_des` to the position command.
- `PD + velocity FF + nominal bias`: adds gravity/Coriolis bias computed from an unperturbed model at the measured state.
- `nominal bias + integral residual`: adds a bounded integral correction (maximum +/-0.04 rad of command offset) to reject persistent mismatch.

## Reproduce

```bash
cargo run --release -p arm-lab-demo --bin robustness_bench -- --write
```
