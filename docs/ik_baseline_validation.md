# Independent IK baseline validation

Validated on 2026-08-18 after the protocol and evaluator were committed, with
all result rows retained.

## Frozen lineage

- base: `a5321fe6e0c4851bf5d5f5a964268b4f0212ea95`
- protocol before evaluator/outcomes:
  `d9078f1b95a159cc09acf2995f895cd91c23aa6e`
- evaluator before first new outcome:
  `6b3320a5e73406eaa9c497eeba3bc96960322e62`
- post-outcome corrective commit:
  `d301e1eff45bf06392dced9840d2d71374d554d7` changes only the report's tiny
  paired-test probability from fixed decimal to scientific notation; it does
  not change a target, solver field, success rule, result row, or analysis.

The pre-existing 97.8% arm-lab restart-enabled result was known at protocol
freeze and is explicitly labeled non-blind. No cold-start, independent-solver,
paired-cell, or union result was known before the frozen evaluator ran.

## Environment

- Fedora Linux 43, kernel `7.1.4-104.fc43.x86_64`, x86-64
- AMD Ryzen 5 3600 (6 cores / 12 logical CPUs), 31 GiB RAM
- rustc `1.97.1 (8bab26f4f 2026-07-14)`, LLVM 22.1.6
- Cargo `1.97.1 (c980f4866 2026-06-30)`
- `mujoco-rs 5.0.0+mj-3.9.0`, MuJoCo 3.9.0
- `k 0.32.0`, `urdf-rs 0.9.0`

## Result-bearing commands

```bash
export MUJOCO_DOWNLOAD_DIR="$PWD/.mujoco-cache"
export LD_LIBRARY_PATH="$MUJOCO_DOWNLOAD_DIR/mujoco-3.9.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cargo run --release -p arm-lab --example ik_baseline -- --write
cargo run --release -p arm-lab --example ik_baseline -- --check
```

The first full write completed normally. After the report-only scientific-
notation correction, a second write completed in 4.40 s including a rebuild
(291,648 KiB maximum RSS), and the immediate warm `--check` independently
regenerated all 4,000 outcomes in 3.36 s (173,892 KiB maximum RSS) and matched
every artifact byte. Wall time is recorded only as operational evidence and is
not a solver metric.

## Artifact integrity

| Artifact | Physical lines | SHA-256 |
|---|---:|---|
| `ik_baseline_targets.csv` | 1,001 | `904834cb95cf9269436e06fa7508797e3d73a4f390edfbbfc777b0851bf7b557` |
| `ik_baseline_results.csv` | 4,001 | `1e8011a4a84dba0da8d573e51eb9bb370d39a725f716e9eec4ed7f86ae88a136` |
| `ik_baseline_results.md` | 43 | `3dd60ac8593b0e8187e42d85eddfea5adb2640e5a3f77e6ee8fd5d5786553a4e` |

The generator asserts exact target/result cardinality and keys, compiled joint
limits, and equality between every stored success flag and a fresh arm-lab FK
pose check. Its unit test mutates one artifact byte and proves the comparator
rejects it. The independent read-only CSV audit separately recomputed:

- success counts `713 / 978 / 328 / 718` in the frozen variant order;
- arm-lab cold/restart cells `713 / 0 / 265 / 22`;
- `k` cold/restart cells `328 / 0 / 390 / 282`;
- restart-enabled cross cells `714 / 264 / 4 / 18`;
- exact two-sided sign probability `8.997736342055427e-73`; and
- every successful stored error below the common strict thresholds.

## Verification results

The final candidate tree passed:

- workspace formatting;
- strict Clippy across every target and feature;
- **68/68** release tests: 42 library/integration tests, five IK evaluator
  tests, and 21 existing benchmark/evaluator tests;
- the full multi-scene replay with every deterministic planning/tracking field
  and outcome unchanged;
- the full 300-query randomized replay with every deterministic field and
  outcome unchanged;
- the independent OxMPL frozen-input/schema/report audit; and
- a further full IK regeneration with every artifact byte unchanged.

Exact commands:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --release --workspace --all-targets
cargo run --release -p arm-lab-demo --bin multi_query_bench -- --check
cargo run --release -p arm-lab-demo --bin randomized_eval -- --check
cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --verify-artifacts
cargo run --release -p arm-lab --example ik_baseline -- --check
```

## Claim boundaries

- The 1,000 poses come from FK of in-limit joint samples. They are reachable
  under the exact arm-lab model, not Cartesian-uniform or task-representative.
- IK evaluation is collision-unaware. It says nothing about a collision-free
  approach, grasping, perception, localization, or control execution.
- The independent solver consumes arm-lab's generated URDF; the existing
  separate FK cross-check agrees below `1e-9`, and every success is still
  re-evaluated through arm-lab FK.
- Maximum attempts and iterations are matched, but algorithms and per-iteration
  work are not. No compute, runtime, or universal solver ranking is claimed.
- The exact paired calculation is descriptive for one deterministic cohort.
- This is simulation-only evidence, not hardware or sim-to-real validation.
