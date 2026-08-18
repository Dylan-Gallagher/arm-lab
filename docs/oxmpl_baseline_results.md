# Independent OxMPL RRT-Connect comparison

This result follows the outcome-blind `docs/oxmpl_baseline_protocol.md`. It compares the in-repository planner with independently maintained OxMPL 0.6.0 on all **217 frozen blocked-direct queries**, five paired seeds each (**1,085 trials**). Every nominal OxMPL solution was independently revalidated at 0.05-rad edge spacing before counting as success.

Pooled seed-trial success: **271/1085 (24.98%)** in-repository and **290/1085 (26.73%)** OxMPL.
Paired outcomes: **271 both**, **0 in-repository only**, **19 OxMPL only**, and **795 neither**; the predeclared exact two-sided McNemar/binomial calculation is **0.00000381**. Because five seeds share each query, this calculation is descriptive only and does not satisfy independent-pair assumptions.

## Results by scene

| Scene | Trials | In-repository success | OxMPL success | Both | In-repository only | OxMPL only | Neither |
|---|---:|---:|---:|---:|---:|---:|---:|
| open_floor | 335 | 67 | 70 | 67 | 0 | 3 | 265 |
| offset_pillar | 400 | 129 | 145 | 129 | 0 | 16 | 255 |
| tabletop_pillar | 350 | 75 | 75 | 75 | 0 | 0 | 275 |

## Per-query repeat distribution

| Successful seeds out of five | In-repository queries | OxMPL queries |
|---:|---:|---:|
| 0 | 161 | 159 |
| 1 | 1 | 0 |
| 2 | 1 | 0 |
| 3 | 1 | 0 |
| 4 | 0 | 0 |
| 5 | 53 | 58 |

At least one of five seeds succeeded on **56/217** blocked queries in-repository and **58/217** with OxMPL. OxMPL returned **0 invalid nominal paths** after independent checking.

## Frozen configuration and integrity

- Inputs: query CSV `c07b622...b5452`; in-repository planning CSV `b739009...13055`.
- OxMPL: crates.io `oxmpl = 0.6.0`, crate archive `11988e3...980dc`, archive VCS commit `6ee31f4...e13`.
- Both planners use 0.25-rad extensions, 0.05 goal bias, the frozen paired seeds, and the same MuJoCo emitted-contact state predicate.
- OxMPL 0.6.0 motion sampling was configured to at most 0.05 rad; every returned segment was then rechecked independently at 0.05 rad.
- OxMPL received a fixed 250-ms wall-time budget; the in-repository artifact used 2,000 iterations. Runtime is intentionally not compared.
- OxMPL consumes eight-decimal serialized query coordinates; the original in-repository rows used pre-serialization values, a maximum representational difference of `5e-9` rad per coordinate.

The exact host, two-run resource measurements, artifact hashes, and deterministic comparison gate are recorded in [`docs/oxmpl_baseline_validation.md`](oxmpl_baseline_validation.md).

## Interpretation boundaries

- This is a cross-implementation comparison on one frozen cohort, not evidence that either planner is universally better.
- Five seeds for one query are repeated algorithm trials, not independent task draws. The retained seed-level McNemar/binomial calculation is descriptive, not a valid population significance test; no population confidence interval is attached.
- Path lengths are retained in the CSV only as diagnostics. The in-repository path is shortcut and densified; OxMPL returns a raw tree path, so cost is not a fair quality comparison.
- OxMPL's timeout is wall-clock-bound. Two consecutive runs on the recorded host must agree, but materially different hardware can change borderline timeout outcomes.
- Seed integers and replicate indices are paired, but arm-lab and OxMPL use different PRNGs; they do not receive identical random samples.
- Collision checks sample configurations and MuJoCo emitted contacts. They do not certify continuous avoidance or positive clearance.
- This is deterministic simulation evidence: no hardware, grasp-success, uncertainty, or sim-to-real claim is introduced.

## Reproduce

```bash
cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --write
cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --check
cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --verify-artifacts
```

`oxmpl_elapsed_ms` is observational wall time and the only field normalized by `--check`.
