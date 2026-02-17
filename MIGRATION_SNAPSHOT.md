# Migration Snapshot

This repository was initialized by copying selected packages from:

- Source workspace: `/Users/moon/code/ipp_ws/src`
- Target workspace: `/Users/moon/code/ia-tigris/src`
- Snapshot date: 2026-02-17

## Included Packages

| Package Dir | Source Branch | Source Commit | Commit Date | Source Remote | Import Mode |
| --- | --- | --- | --- | --- | --- |
| `core_ompl` | `ipp` | `7247d187` | 2023-02-27 | `https://bitbucket.org/castacks/core_ompl.git` | vendored copy (trimmed release footprint) |
| `core_planning_state_space` | `master` | `bafb64a` | 2022-07-06 | `https://bitbucket.org/castacks/core_planning_state_space.git` | vendored copy |
| `core_planning_utils` | `master` | `f75fac1` | 2020-07-03 | `https://bitbucket.org/castacks/core_planning_utils.git` | vendored copy |
| `cpu_monitor` | `master` | `2fe712a` | 2022-12-01 | `git@github.com:castacks/cpu_monitor.git` | git submodule (pinned SHA) |
| `ipp_beliefs` | `develop` | `86bee2b` | 2025-03-03 | `git@github.com:castacks/ipp_beliefs.git` | vendored copy |
| `ipp_metrics` | `develop` | `076b942` | 2025-02-05 | `git@github.com:castacks/ipp_metrics.git` | vendored copy |
| `ipp_planners` | `develop` | `d71f0d3` | 2025-09-03 | `git@github.com:castacks/ipp_planners.git` | vendored copy + local working-tree edits |
| `ipp_simple_sim` | `develop` | `d447247` | 2025-02-13 | `git@github.com:castacks/ipp_simple_sim.git` | vendored copy |
| `math_utils` | `master` | `f857e75` | 2023-05-03 | `https://bitbucket.org/castacks/math_utils.git` | vendored copy |
| `planner_map_interfaces` | `develop` | `38ae59c` | 2025-03-03 | `git@github.com:castacks/planner_map_interfaces.git` | vendored copy from committed `HEAD` snapshot |
| `simulated_perception` | `develop` | `7fe8a24` | 2023-05-17 | `git@github.com:castacks/simulated_perception.git` | vendored copy |
| `trochoids` | `straight-on-end` | `ab644c0` | 2024-04-26 | `git@github.com:castacks/trochoids.git` | git submodule |

## Notes

- `ipp_planners` intentionally includes local changes from the source workspace for Docker/Foxglove support.
- `planner_map_interfaces` was copied from committed `HEAD` to avoid carrying over a local uncommitted file clear.
- `cpu_monitor` submodule is pinned at commit `2fe712a1b55c3307206371995a435926bb087a84` (tag `v1.12.1`).
- `core_ompl` was reduced by removing non-essential directories: `demos/`, `doc/`, `py-bindings/`, `tests/`.
