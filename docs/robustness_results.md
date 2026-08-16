# UR5e controller robustness envelope (simulation)

Deterministic MuJoCo stress test over a 3.50 s, 1751-sample collision-free RRT-Connect path timed by the in-repo scalar S-curve. Polyline corners are not blended, so the complete joint trajectory is not globally acceleration- or jerk-bounded. **This is simulation evidence, not hardware validation or a sim-to-real guarantee.**

Pass threshold (declared in code): RMS joint error <= 0.03 rad, maximum joint error <= 0.10 rad, and final joint error <= 0.02 rad. Controller bias forces always come from the unchanged nominal model; perturbed plant parameters are not exposed to the controller.

| Scenario | Controller | RMS joint (rad) | Max joint (rad) | Final (rad) | Max EE pos (m) | Peak force/limit | Saturated steps | Result |
|---|---|---:|---:|---:|---:|---:|---:|:---:|
| nominal | position PD | 0.1409 | 0.1719 | 0.0111 | 0.0777 | 12% | 0.0% | FAIL |
| nominal | PD + velocity FF | 0.0097 | 0.0121 | 0.0117 | 0.0086 | 100% | 0.1% | PASS |
| nominal | PD + velocity FF + nominal bias | 0.0016 | 0.0046 | 0.0001 | 0.0036 | 100% | 0.1% | PASS |
| nominal | nominal bias + integral residual | 0.0020 | 0.0048 | 0.0004 | 0.0037 | 100% | 0.1% | PASS |
| 2 kg tool payload | position PD | 0.1416 | 0.1756 | 0.0187 | 0.0823 | 19% | 0.0% | FAIL |
| 2 kg tool payload | PD + velocity FF | 0.0173 | 0.0201 | 0.0193 | 0.0140 | 100% | 0.2% | PASS |
| 2 kg tool payload | PD + velocity FF + nominal bias | 0.0084 | 0.0092 | 0.0079 | 0.0060 | 100% | 0.2% | PASS |
| 2 kg tool payload | nominal bias + integral residual | 0.0030 | 0.0077 | 0.0007 | 0.0061 | 100% | 0.2% | PASS |
| 65% actuator gains | position PD | 0.1416 | 0.1738 | 0.0172 | 0.0801 | 12% | 0.0% | FAIL |
| 65% actuator gains | PD + velocity FF | 0.0149 | 0.0187 | 0.0180 | 0.0134 | 100% | 0.1% | PASS |
| 65% actuator gains | PD + velocity FF + nominal bias | 0.0019 | 0.0063 | 0.0001 | 0.0049 | 100% | 0.1% | PASS |
| 65% actuator gains | nominal bias + integral residual | 0.0022 | 0.0064 | 0.0006 | 0.0051 | 100% | 0.1% | PASS |
| +2 Nms/rad joint damping | position PD | 0.1426 | 0.1745 | 0.0112 | 0.0781 | 12% | 0.0% | FAIL |
| +2 Nms/rad joint damping | PD + velocity FF | 0.0097 | 0.0121 | 0.0117 | 0.0086 | 100% | 0.1% | PASS |
| +2 Nms/rad joint damping | PD + velocity FF + nominal bias | 0.0014 | 0.0044 | 0.0001 | 0.0032 | 100% | 0.1% | PASS |
| +2 Nms/rad joint damping | nominal bias + integral residual | 0.0018 | 0.0049 | 0.0008 | 0.0038 | 100% | 0.1% | PASS |
| 20 ms command latency | position PD | 0.1551 | 0.1890 | 0.0111 | 0.0851 | 12% | 0.0% | FAIL |
| 20 ms command latency | PD + velocity FF | 0.0174 | 0.0232 | 0.0117 | 0.0142 | 100% | 0.1% | PASS |
| 20 ms command latency | PD + velocity FF + nominal bias | 0.0142 | 0.0194 | 0.0001 | 0.0078 | 100% | 0.1% | PASS |
| 20 ms command latency | nominal bias + integral residual | 0.0116 | 0.0341 | 0.0034 | 0.0150 | 100% | 0.1% | PASS |
| 20 Nm / 120 ms torque pulse | position PD | 0.1409 | 0.1719 | 0.0111 | 0.0776 | 19% | 0.0% | FAIL |
| 20 Nm / 120 ms torque pulse | PD + velocity FF | 0.0098 | 0.0121 | 0.0117 | 0.0086 | 100% | 0.2% | PASS |
| 20 Nm / 120 ms torque pulse | PD + velocity FF + nominal bias | 0.0022 | 0.0085 | 0.0001 | 0.0064 | 100% | 0.2% | PASS |
| 20 Nm / 120 ms torque pulse | nominal bias + integral residual | 0.0026 | 0.0085 | 0.0004 | 0.0064 | 100% | 0.2% | PASS |
| combined moderate shift | position PD | 0.1498 | 0.1852 | 0.0187 | 0.0856 | 16% | 0.0% | FAIL |
| combined moderate shift | PD + velocity FF | 0.0186 | 0.0241 | 0.0194 | 0.0168 | 100% | 0.1% | PASS |
| combined moderate shift | PD + velocity FF + nominal bias | 0.0094 | 0.0117 | 0.0050 | 0.0069 | 100% | 0.1% | PASS |
| combined moderate shift | nominal bias + integral residual | 0.0071 | 0.0224 | 0.0020 | 0.0132 | 100% | 0.1% | PASS |

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
