# OxMPL baseline validation record

Validation date: 2026-08-18 (Europe/Dublin). This is a simulation-only
execution record for the predeclared
[`oxmpl_baseline_protocol.md`](oxmpl_baseline_protocol.md).

## Exact environment

- OS: Fedora Linux 43 Workstation, x86-64;
- kernel: `7.1.4-104.fc43.x86_64`;
- CPU: AMD Ryzen 5 3600, 6 cores / 12 threads;
- Rust: `rustc 1.97.1 (8bab26f4f 2026-07-14)`;
- Cargo: `cargo 1.97.1 (c980f4866 2026-06-30)`;
- MuJoCo: 3.9.0 through `mujoco-rs 5.0.0+mj-3.9.0`; and
- external planner: crates.io `oxmpl 0.6.0`, archive SHA-256
  `11988e3c34bfcacbc4d96265f1da76f9bd3a30fcc92ad62e4046cc6315e980dc`.

No other CPU-heavy evaluation was intentionally run concurrently.

## Two consecutive outcome runs

The protocol requires one result write followed immediately by a full
regeneration/check. Both commands used the release profile and the same
MuJoCo runtime:

```bash
cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --write
cargo run --release -p arm-lab-demo --bin oxmpl_baseline -- --check
```

| Run | Exit | Wall | User CPU | Max RSS | Deterministic result |
|---|---:|---:|---:|---:|---|
| write | 0 | 3:26.12 | 205.70 s | 254,184 KiB | 1,085 rows written |
| immediate check | 0 | 3:25.27 | 204.23 s | 256,772 KiB | exact match after wall-time normalization |

The check compared every status, failure detail, waypoint count, raw path
cost, and SHA-256 digest of the exact returned floating-point path. Only
`oxmpl_elapsed_ms` was normalized. There were no status flips or path-content
mismatches. The slowest successful first-run solve was 175.554487 ms, leaving
74.445513 ms before the predeclared 250-ms boundary; timeout cases remain
wall-clock-dependent on materially different hardware.

## Retained outcomes and artifacts

- in-repository success: 271/1,085;
- OxMPL success: 290/1,085;
- paired cells (both / in-repository only / OxMPL only / neither):
  271 / 0 / 19 / 795;
- independently invalid OxMPL paths: 0;
- `oxmpl_baseline_results.csv` SHA-256:
  `3b44b2a05283b5da752ab1464091d03c03702afda8323f8718df55927c37d840`;
- `oxmpl_baseline_results.md` final SHA-256:
  `31d894331e4b9f1d72d9d6f8f2df67826ccb48e174820efeeb5f2a5a68668c56`.

The fast `--verify-artifacts` mode checks frozen input hashes, the 1,085-row
schema and paired keys, re-renders the CSV, and derives the complete Markdown
report from the committed rows without rerunning the wall-time-bound planner.

## Source and chronology audit

The result-bearing runs used evaluator commit
`8bfe273f13f151a8adf0b6f5bf32503c20538bbd`. Its first parent chain contains
the protocol freeze (`ed48da6`), the matched-limit clarification (`10ee6fc`),
and the serialized-coordinate disclosure (`9a793e8`), all committed before the
evaluator dependency and implementation. The working tree was clean before
the first `--write` run.

A fresh source audit mapped the evaluator to every frozen requirement:
cohort/input hashes, 217 blocked-query filter, 1,085 paired keys and seeds,
0.25-rad extension, 0.05 goal bias, 250-ms solve budget, 0.05-rad internal and
independent edge sampling, endpoint/bounds/finite checks, retained status
categories, exact-path digests, and wall-time-only normalization. Formatting,
strict workspace Clippy, all workspace release tests, both existing artifact
regenerations, the new artifact verifier, and an independent CSV invariant
script passed.

Post-run evaluator changes are confined to generated-report wording: they add
the validation-record link and explicitly downgrade the seed-level McNemar
calculation because seeds share queries. Post-run protocol edits correct the
public OxMPL type path and clarify that the paired integer seeds feed different
PRNGs. No planning, collision, timeout, validation, parsing, aggregation, or
raw-CSV logic changed after results were observed.

## Claim boundary

This validates exact-host repeatability and independent sampled-path checking.
It does not establish cross-hardware timeout invariance, equal compute,
continuous collision safety, positive clearance, hardware behavior, or
sim-to-real performance.
