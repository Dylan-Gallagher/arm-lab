//! Jerk-limited time-parameterization of joint-space paths, from scratch.
//!
//! A rest-to-rest **7-phase S-curve** (bang-bang jerk: trapezoidal
//! acceleration) parameterizes every nonzero edge of a joint-space polyline.
//! Every interior waypoint is a full stop, so position, velocity, and
//! acceleration are continuous even when the polyline tangent changes. Jerk
//! may jump between finite left and right values at a waypoint, but its
//! magnitude remains bounded.
//!
//! Limits are specified per joint. Each edge's path-space `(v, a, j)` limits
//! are the tightest joint limit divided by `|dqᵢ/ds|` on that edge. Edge
//! durations are rounded up to the fixed sample interval by slowing their
//! S-curves, which can only reduce all three derivatives.

/// Per-joint kinematic limits. One value is used for every joint; the path
/// converter then tightens the scalar profile against the steepest joint.
#[derive(Debug, Clone, Copy)]
pub struct TrajLimits {
    pub v_max: f64,
    pub a_max: f64,
    pub j_max: f64,
}

impl Default for TrajLimits {
    fn default() -> Self {
        Self {
            v_max: 1.5,
            a_max: 4.0,
            j_max: 20.0,
        }
    }
}

/// Sampled joint-space trajectory, including the rest endpoints.
#[derive(Debug, Clone)]
pub struct Trajectory {
    pub dt: f64,
    pub duration: f64,
    pub t: Vec<f64>,
    pub q: Vec<Vec<f64>>,
    pub qd: Vec<Vec<f64>>,
    pub qdd: Vec<Vec<f64>>,
    pub qddd: Vec<Vec<f64>>,
}

impl Trajectory {
    pub fn len(&self) -> usize {
        self.q.len()
    }

    pub fn is_empty(&self) -> bool {
        self.q.is_empty()
    }
}

/// Time-parameterize a joint-space polyline with a rest-to-rest S-curve on
/// every nonzero edge.
///
/// Interior waypoints are full stops. Callers should therefore pass geometric
/// shortcut waypoints rather than collision-check densification samples.
pub fn time_parameterize(path: &[Vec<f64>], limits: &TrajLimits, dt: f64) -> Trajectory {
    assert!(dt > 0.0, "dt must be positive");
    assert!(!path.is_empty(), "path must not be empty");
    let n = path[0].len();
    assert!(path.iter().all(|q| q.len() == n), "ragged path");

    if path.windows(2).all(|w| l2(&w[0], &w[1]) < 1e-12) {
        return stationary(path[0].clone(), dt);
    }

    let mut t = Vec::new();
    let mut q = Vec::new();
    let mut qd = Vec::new();
    let mut qdd = Vec::new();
    let mut qddd = Vec::new();
    let mut elapsed_steps = 0usize;
    let mut have_edge = false;

    for edge in path.windows(2) {
        let length = l2(&edge[0], &edge[1]);
        if length < 1e-12 {
            continue;
        }
        let tangent: Vec<f64> = edge[0]
            .iter()
            .zip(edge[1].iter())
            .map(|(a, b)| (b - a) / length)
            .collect();
        let (v_s, a_s, j_s) = scalar_limits(&tangent, limits);
        let curve = SCurve1d::rest_to_rest(length, v_s, a_s, j_s);
        let edge_steps = ((curve.duration / dt).ceil() as usize).max(1);
        let aligned_duration = edge_steps as f64 * dt;
        let time_scale = aligned_duration / curve.duration;

        let first_step = usize::from(have_edge);
        for edge_step in first_step..=edge_steps {
            let curve_t = (edge_step as f64 * dt / time_scale).min(curve.duration);
            let motion = curve.at(curve_t);
            let mut qk: Vec<f64> = edge[0]
                .iter()
                .zip(tangent.iter())
                .map(|(start, direction)| start + direction * motion.s)
                .collect();
            if edge_step == edge_steps {
                qk.clone_from(&edge[1]);
            }
            t.push((elapsed_steps + edge_step) as f64 * dt);
            q.push(qk);
            qd.push(
                tangent
                    .iter()
                    .map(|direction| direction * motion.v / time_scale)
                    .collect(),
            );
            qdd.push(
                tangent
                    .iter()
                    .map(|direction| direction * motion.a / time_scale.powi(2))
                    .collect(),
            );
            qddd.push(
                tangent
                    .iter()
                    .map(|direction| direction * motion.j / time_scale.powi(3))
                    .collect(),
            );
        }
        elapsed_steps += edge_steps;
        have_edge = true;
    }

    Trajectory {
        dt,
        duration: elapsed_steps as f64 * dt,
        t,
        q,
        qd,
        qdd,
        qddd,
    }
}

fn stationary(q0: Vec<f64>, dt: f64) -> Trajectory {
    let n = q0.len();
    Trajectory {
        dt,
        duration: 0.0,
        t: vec![0.0],
        q: vec![q0],
        qd: vec![vec![0.0; n]],
        qdd: vec![vec![0.0; n]],
        qddd: vec![vec![0.0; n]],
    }
}

fn scalar_limits(tangent: &[f64], limits: &TrajLimits) -> (f64, f64, f64) {
    let mut scalar = (f64::INFINITY, f64::INFINITY, f64::INFINITY);
    for slope in tangent.iter().map(|value| value.abs()) {
        if slope >= 1e-12 {
            scalar.0 = scalar.0.min(limits.v_max / slope);
            scalar.1 = scalar.1.min(limits.a_max / slope);
            scalar.2 = scalar.2.min(limits.j_max / slope);
        }
    }
    assert!(
        scalar.0.is_finite() && scalar.1.is_finite() && scalar.2.is_finite(),
        "edge has no moving joint"
    );
    scalar
}

fn l2(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}

/// 1D rest-to-rest jerk-limited profile.
#[derive(Debug, Clone)]
pub struct SCurve1d {
    pub duration: f64,
    pub distance: f64,
    /// `(duration, jerk)` per phase; zero-length phases are omitted.
    phases: Vec<(f64, f64)>,
}

#[derive(Debug, Clone, Copy)]
pub struct Motion1d {
    pub s: f64,
    pub v: f64,
    pub a: f64,
    pub j: f64,
}

impl SCurve1d {
    /// Rest-to-rest S-curve covering `distance` (sign preserved).
    pub fn rest_to_rest(distance: f64, v_max: f64, a_max: f64, j_max: f64) -> Self {
        assert!(v_max > 0.0 && a_max > 0.0 && j_max > 0.0);
        let sign = distance.signum();
        let d = distance.abs();
        if d < 1e-15 {
            return Self {
                duration: 0.0,
                distance: 0.0,
                phases: Vec::new(),
            };
        }

        // Peak accel reachable without exceeding v_max (triangular v-profile).
        let a_max = a_max.min((v_max * j_max).sqrt());
        let tj_a = a_max / j_max;
        let ta_v = (v_max / a_max - tj_a).max(0.0);
        let s_acc_v = accel_distance(a_max, tj_a, ta_v);

        let (tj, ta, tv, _a_used) = if 2.0 * s_acc_v <= d + 1e-15 {
            let tv = (d - 2.0 * s_acc_v).max(0.0) / v_max;
            (tj_a, ta_v, tv, a_max)
        } else {
            let disc = tj_a * tj_a + 4.0 * d / a_max;
            let ta = 0.5 * (-3.0 * tj_a + disc.sqrt());
            if ta > 1e-14 {
                (tj_a, ta, 0.0, a_max)
            } else {
                let tj = (d / (2.0 * j_max)).cbrt();
                (tj, 0.0, 0.0, j_max * tj)
            }
        };

        let j = j_max * sign;
        let mut phases = Vec::new();
        push_phase(&mut phases, tj, j);
        push_phase(&mut phases, ta, 0.0);
        push_phase(&mut phases, tj, -j);
        push_phase(&mut phases, tv, 0.0);
        push_phase(&mut phases, tj, -j);
        push_phase(&mut phases, ta, 0.0);
        push_phase(&mut phases, tj, j);

        let duration: f64 = phases.iter().map(|(dt, _)| dt).sum();
        Self {
            duration,
            distance,
            phases,
        }
    }

    pub fn at(&self, t: f64) -> Motion1d {
        if self.duration == 0.0 {
            return Motion1d {
                s: 0.0,
                v: 0.0,
                a: 0.0,
                j: 0.0,
            };
        }
        let t = t.clamp(0.0, self.duration);
        if t >= self.duration {
            return Motion1d {
                s: self.distance,
                v: 0.0,
                a: 0.0,
                j: 0.0,
            };
        }
        let mut s = 0.0;
        let mut v = 0.0;
        let mut a = 0.0;
        let mut t_left = t;
        let mut j_now = 0.0;
        for &(dt, j) in &self.phases {
            if t_left <= dt + 1e-18 {
                s += v * t_left + 0.5 * a * t_left * t_left + (1.0 / 6.0) * j * t_left.powi(3);
                v += a * t_left + 0.5 * j * t_left * t_left;
                a += j * t_left;
                j_now = j;
                return Motion1d { s, v, a, j: j_now };
            }
            s += v * dt + 0.5 * a * dt * dt + (1.0 / 6.0) * j * dt.powi(3);
            v += a * dt + 0.5 * j * dt * dt;
            a += j * dt;
            t_left -= dt;
            j_now = j;
        }
        Motion1d {
            s: self.distance,
            v: 0.0,
            a: 0.0,
            j: j_now,
        }
    }
}

fn push_phase(phases: &mut Vec<(f64, f64)>, dt: f64, j: f64) {
    if dt > 1e-14 {
        phases.push((dt, j));
    }
}

/// Distance covered while accelerating from rest to `a_max * (ta + tj)`
/// (end of the three accel phases), with `a_max = j_max * tj`.
fn accel_distance(a_max: f64, tj: f64, ta: f64) -> f64 {
    a_max * tj * tj + 1.5 * a_max * tj * ta + 0.5 * a_max * ta * ta
}

#[cfg(test)]
mod unit {
    use super::*;

    fn almost(a: f64, b: f64, eps: f64) {
        assert!(
            (a - b).abs() <= eps,
            "{a} ≉ {b} (eps {eps}, err {})",
            (a - b).abs()
        );
    }

    #[test]
    fn rest_to_rest_reaches_distance_at_rest() {
        let c = SCurve1d::rest_to_rest(1.2, 0.8, 2.0, 10.0);
        let start = c.at(0.0);
        almost(start.s, 0.0, 1e-12);
        almost(start.v, 0.0, 1e-12);
        almost(start.a, 0.0, 1e-9);
        let end = c.at(c.duration);
        almost(end.s, 1.2, 1e-9);
        almost(end.v, 0.0, 1e-8);
        almost(end.a, 0.0, 1e-6);
        let past = c.at(c.duration + 1.0);
        almost(past.s, 1.2, 1e-9);
    }

    #[test]
    fn respects_bounds() {
        let (vmax, amax, jmax) = (0.5, 1.5, 8.0);
        let c = SCurve1d::rest_to_rest(2.0, vmax, amax, jmax);
        let n = 400;
        let (mut mv, mut ma, mut mj) = (0.0f64, 0.0f64, 0.0f64);
        for k in 0..=n {
            let m = c.at(c.duration * k as f64 / n as f64);
            mv = mv.max(m.v.abs());
            ma = ma.max(m.a.abs());
            mj = mj.max(m.j.abs());
        }
        assert!(mv <= vmax + 1e-9, "v {mv} > vmax {vmax}");
        assert!(ma <= amax + 1e-9, "a {ma} > amax {amax}");
        assert!(mj <= jmax + 1e-9, "j {mj} > jmax {jmax}");
    }

    #[test]
    fn short_move_skips_velocity_cruise() {
        // Tiny D: triangular accel, never hits vmax.
        let c = SCurve1d::rest_to_rest(0.01, 2.0, 5.0, 40.0);
        let mut max_v = 0.0f64;
        for k in 0..=200 {
            max_v = max_v.max(c.at(c.duration * k as f64 / 200.0).v.abs());
        }
        assert!(max_v < 2.0 - 1e-6, "short move should not reach vmax");
        almost(c.at(c.duration).s, 0.01, 1e-9);
    }

    #[test]
    fn negative_distance() {
        let c = SCurve1d::rest_to_rest(-0.4, 1.0, 3.0, 12.0);
        almost(c.at(c.duration).s, -0.4, 1e-9);
        almost(c.at(c.duration).v, 0.0, 1e-8);
        assert!(c.at(c.duration * 0.25).s < 0.0);
    }

    #[test]
    fn jerk_is_bang_bang() {
        let jmax = 9.0;
        let c = SCurve1d::rest_to_rest(0.8, 1.0, 2.5, jmax);
        for &(dt, j) in &c.phases {
            assert!(dt > 0.0);
            assert!(
                (j.abs() - jmax).abs() < 1e-12 || j.abs() < 1e-12,
                "jerk {j} is not 0 or ±{jmax}"
            );
        }
    }
}
