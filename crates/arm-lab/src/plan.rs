//! Joint-space RRT-Connect with shortcutting, from scratch.
//!
//! The planner is deliberately physics-agnostic: it takes a `collides`
//! predicate `FnMut(&[f64]) -> bool`. The MuJoCo checker in
//! [`crate::collision`] is the predicate used in the tests and demos;
//! a synthetic predicate is enough to unit-test the algorithm itself.
//!
//! Algorithm: Kuffner & LaValle, *RRT-Connect: An Efficient Approach to
//! Single-Query Path Planning* (ICRA 2000). Two trees grow toward each
//! other; `EXTEND` takes one step of length `step`, `CONNECT` greedily
//! repeats `EXTEND` until it reaches the other tree or hits an obstacle.
//! After a feasible path is found, greedy then random shortcutting
//! removes redundant waypoints, and the path is densified to `resolution`
//! so a downstream time-parameterizer can treat it as a polyline.

use std::time::Instant;

use crate::chain::Chain;
use crate::rng::Rng;

/// Planner tunables. Defaults are sized for a 6-DOF arm in a mildly
/// cluttered MuJoCo scene.
#[derive(Debug, Clone)]
pub struct PlanConfig {
    /// Seed for sampling and random shortcutting. Same seed + scene →
    /// bit-identical path.
    pub seed: u64,
    /// Maximum RRT-Connect iterations (each iteration grows one tree).
    pub max_iters: usize,
    /// Maximum extension length per `EXTEND`, Euclidean in joint space (rad).
    pub step: f64,
    /// Collision-check spacing along edges, Euclidean in joint space (rad).
    pub resolution: f64,
    /// Probability of sampling the other tree's root instead of a uniform
    /// configuration (classic RRT goal bias).
    pub goal_bias: f64,
    /// Random shortcut attempts after the greedy pass.
    pub shortcut_iters: usize,
    /// If true, densify the returned path to `resolution`.
    pub densify: bool,
}

impl Default for PlanConfig {
    fn default() -> Self {
        Self {
            seed: 1,
            max_iters: 2000,
            step: 0.25,
            resolution: 0.05,
            goal_bias: 0.05,
            shortcut_iters: 40,
            densify: true,
        }
    }
}

/// Outcome of a planning query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanStatus {
    Success,
    StartCollision,
    GoalCollision,
    Unconnected,
}

/// Result of [`rrt_connect`]. `path` is empty unless `status` is `Success`.
#[derive(Debug, Clone)]
pub struct PlanResult {
    pub status: PlanStatus,
    /// Collision-free waypoints, start to goal. Densified when configured.
    pub path: Vec<Vec<f64>>,
    /// Waypoints after shortcutting, before densify. Useful for tests.
    pub waypoints: Vec<Vec<f64>>,
    pub nodes: usize,
    pub iterations: usize,
    pub shortcut_removed: usize,
    /// Sum of joint-space L2 segments of `path`.
    pub cost: f64,
    pub elapsed_s: f64,
}

#[derive(Clone)]
struct Node {
    q: Vec<f64>,
    parent: Option<usize>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Status {
    Reached,
    Advanced,
    Trapped,
}

/// Plan a collision-free joint-space path from `start` to `goal`.
pub fn rrt_connect(
    chain: &Chain,
    start: &[f64],
    goal: &[f64],
    mut collides: impl FnMut(&[f64]) -> bool,
    cfg: &PlanConfig,
) -> PlanResult {
    let t0 = Instant::now();
    let n = chain.dof();
    assert_eq!(start.len(), n);
    assert_eq!(goal.len(), n);

    if collides(start) {
        return failed(PlanStatus::StartCollision, t0);
    }
    if collides(goal) {
        return failed(PlanStatus::GoalCollision, t0);
    }
    if l2(start, goal) < 1e-12 {
        let path = vec![start.to_vec()];
        return PlanResult {
            status: PlanStatus::Success,
            path: path.clone(),
            waypoints: path,
            nodes: 1,
            iterations: 0,
            shortcut_removed: 0,
            cost: 0.0,
            elapsed_s: t0.elapsed().as_secs_f64(),
        };
    }

    let mut rng = Rng::new(cfg.seed);
    let mut scratch = vec![0.0; n];

    // Direct connection: if the straight line is free, skip RRT entirely.
    if edge_free(start, goal, cfg.resolution, &mut collides, &mut scratch) {
        let waypoints = vec![start.to_vec(), goal.to_vec()];
        return finish(waypoints, 2, 0, 0, cfg, t0);
    }

    let mut start_tree = vec![Node {
        q: start.to_vec(),
        parent: None,
    }];
    let mut goal_tree = vec![Node {
        q: goal.to_vec(),
        parent: None,
    }];

    let mut grow_start = true;
    let mut iterations = 0usize;

    for it in 0..cfg.max_iters {
        iterations = it + 1;
        let q_rand = if rng.next_f64() < cfg.goal_bias {
            if grow_start {
                goal.to_vec()
            } else {
                start.to_vec()
            }
        } else {
            chain.sample_uniform(&mut rng)
        };

        let connected = {
            let (ta, tb) = if grow_start {
                (&mut start_tree, &mut goal_tree)
            } else {
                (&mut goal_tree, &mut start_tree)
            };
            if extend(ta, &q_rand, chain, cfg, &mut collides, &mut scratch) == Status::Trapped {
                false
            } else {
                let q_new = ta.last().expect("EXTEND added a node").q.clone();
                connect(tb, &q_new, chain, cfg, &mut collides, &mut scratch) == Status::Reached
            }
        };

        if connected {
            let mut raw = trace(&start_tree, start_tree.len() - 1);
            let mut tail = trace(&goal_tree, goal_tree.len() - 1);
            tail.reverse();
            raw.extend(tail.into_iter().skip(1));
            let nodes = start_tree.len() + goal_tree.len();
            let before = raw.len();
            let waypoints = shortcut(&raw, cfg, &mut rng, &mut collides, &mut scratch);
            let removed = before.saturating_sub(waypoints.len());
            return finish(waypoints, nodes, iterations, removed, cfg, t0);
        }
        grow_start = !grow_start;
    }

    let nodes = start_tree.len() + goal_tree.len();
    PlanResult {
        status: PlanStatus::Unconnected,
        path: Vec::new(),
        waypoints: Vec::new(),
        nodes,
        iterations,
        shortcut_removed: 0,
        cost: 0.0,
        elapsed_s: t0.elapsed().as_secs_f64(),
    }
}

/// Euclidean length of a joint-space polyline.
pub fn path_length(path: &[Vec<f64>]) -> f64 {
    path.windows(2).map(|w| l2(&w[0], &w[1])).sum()
}

/// Linearly interpolate a path so consecutive waypoints are at most
/// `resolution` apart (L2 in joint space).
pub fn densify(path: &[Vec<f64>], resolution: f64) -> Vec<Vec<f64>> {
    if path.is_empty() {
        return Vec::new();
    }
    let mut out = vec![path[0].clone()];
    for w in path.windows(2) {
        let d = l2(&w[0], &w[1]);
        if d < 1e-12 {
            continue;
        }
        let n = ((d / resolution).ceil() as usize).max(1);
        for i in 1..=n {
            let t = i as f64 / n as f64;
            out.push(lerp(&w[0], &w[1], t));
        }
    }
    out
}

/// True iff the straight-line interpolant from `a` to `b` is collision-free
/// at spacing `resolution`. The start point `a` is assumed already free
/// and is not re-checked.
pub fn edge_free(
    a: &[f64],
    b: &[f64],
    resolution: f64,
    collides: &mut impl FnMut(&[f64]) -> bool,
    scratch: &mut Vec<f64>,
) -> bool {
    let d = l2(a, b);
    let n = ((d / resolution).ceil() as usize).max(1);
    scratch.resize(a.len(), 0.0);
    for i in 1..=n {
        let t = i as f64 / n as f64;
        for k in 0..a.len() {
            scratch[k] = a[k] + t * (b[k] - a[k]);
        }
        if collides(scratch) {
            return false;
        }
    }
    true
}

pub(crate) fn l2(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn lerp(a: &[f64], b: &[f64], t: f64) -> Vec<f64> {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| x + t * (y - x))
        .collect()
}

fn failed(status: PlanStatus, t0: Instant) -> PlanResult {
    PlanResult {
        status,
        path: Vec::new(),
        waypoints: Vec::new(),
        nodes: 0,
        iterations: 0,
        shortcut_removed: 0,
        cost: 0.0,
        elapsed_s: t0.elapsed().as_secs_f64(),
    }
}

fn finish(
    waypoints: Vec<Vec<f64>>,
    nodes: usize,
    iterations: usize,
    shortcut_removed: usize,
    cfg: &PlanConfig,
    t0: Instant,
) -> PlanResult {
    let path = if cfg.densify {
        densify(&waypoints, cfg.resolution)
    } else {
        waypoints.clone()
    };
    let cost = path_length(&path);
    PlanResult {
        status: PlanStatus::Success,
        path,
        waypoints,
        nodes,
        iterations,
        shortcut_removed,
        cost,
        elapsed_s: t0.elapsed().as_secs_f64(),
    }
}

fn nearest(tree: &[Node], q: &[f64]) -> usize {
    let mut best = 0usize;
    let mut best_d = f64::INFINITY;
    for (i, node) in tree.iter().enumerate() {
        let d = l2(&node.q, q);
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    best
}

fn steer(from: &[f64], toward: &[f64], step: f64, chain: &Chain) -> Vec<f64> {
    let d = l2(from, toward);
    let mut q = if d <= step || d < 1e-12 {
        toward.to_vec()
    } else {
        lerp(from, toward, step / d)
    };
    chain.clamp_to_limits(&mut q);
    q
}

fn extend(
    tree: &mut Vec<Node>,
    q_target: &[f64],
    chain: &Chain,
    cfg: &PlanConfig,
    collides: &mut impl FnMut(&[f64]) -> bool,
    scratch: &mut Vec<f64>,
) -> Status {
    let i_near = nearest(tree, q_target);
    let q_new = steer(&tree[i_near].q, q_target, cfg.step, chain);
    if l2(&tree[i_near].q, &q_new) < 1e-12 {
        return Status::Trapped;
    }
    if !edge_free(&tree[i_near].q, &q_new, cfg.resolution, collides, scratch) {
        return Status::Trapped;
    }
    let reached = l2(&q_new, q_target) < 1e-9;
    tree.push(Node {
        q: q_new,
        parent: Some(i_near),
    });
    if reached {
        Status::Reached
    } else {
        Status::Advanced
    }
}

fn connect(
    tree: &mut Vec<Node>,
    q_target: &[f64],
    chain: &Chain,
    cfg: &PlanConfig,
    collides: &mut impl FnMut(&[f64]) -> bool,
    scratch: &mut Vec<f64>,
) -> Status {
    loop {
        match extend(tree, q_target, chain, cfg, collides, scratch) {
            Status::Advanced => continue,
            other => return other,
        }
    }
}

fn trace(tree: &[Node], idx: usize) -> Vec<Vec<f64>> {
    let mut path = Vec::new();
    let mut i = Some(idx);
    while let Some(k) = i {
        path.push(tree[k].q.clone());
        i = tree[k].parent;
    }
    path.reverse();
    path
}

/// Greedy shortcutting (always take the farthest visible waypoint) followed
/// by random shortcutting.
fn shortcut(
    path: &[Vec<f64>],
    cfg: &PlanConfig,
    rng: &mut Rng,
    collides: &mut impl FnMut(&[f64]) -> bool,
    scratch: &mut Vec<f64>,
) -> Vec<Vec<f64>> {
    let mut out = greedy_shortcut(path, cfg.resolution, collides, scratch);
    for _ in 0..cfg.shortcut_iters {
        if out.len() < 3 {
            break;
        }
        let i = rng.uniform_usize(0, out.len() - 2);
        let j = rng.uniform_usize(i + 2, out.len());
        if edge_free(&out[i], &out[j], cfg.resolution, collides, scratch) {
            out.drain(i + 1..j);
        }
    }
    out
}

fn greedy_shortcut(
    path: &[Vec<f64>],
    resolution: f64,
    collides: &mut impl FnMut(&[f64]) -> bool,
    scratch: &mut Vec<f64>,
) -> Vec<Vec<f64>> {
    if path.len() <= 2 {
        return path.to_vec();
    }
    let mut out = vec![path[0].clone()];
    let mut i = 0usize;
    while i < path.len() - 1 {
        let mut j = path.len() - 1;
        while j > i + 1 {
            if edge_free(&path[i], &path[j], resolution, collides, scratch) {
                break;
            }
            j -= 1;
        }
        out.push(path[j].clone());
        i = j;
    }
    out
}

#[cfg(test)]
mod unit {
    use super::*;

    #[test]
    fn lerp_endpoints() {
        let a = vec![0.0, 0.0];
        let b = vec![2.0, 0.0];
        assert_eq!(lerp(&a, &b, 0.0), a);
        assert_eq!(lerp(&a, &b, 1.0), b);
        assert_eq!(lerp(&a, &b, 0.5), vec![1.0, 0.0]);
    }

    #[test]
    fn densify_spacing() {
        let path = vec![vec![0.0, 0.0], vec![1.0, 0.0]];
        let d = densify(&path, 0.25);
        assert!(d.len() >= 5);
        for w in d.windows(2) {
            assert!(l2(&w[0], &w[1]) <= 0.25 + 1e-12);
        }
        assert_eq!(d.first(), path.first());
        assert_eq!(d.last(), path.last());
    }
}
