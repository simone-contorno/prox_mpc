# Contributing to ProxMPC

Thanks for your interest in contributing to ProxMPC. This is a ROS 2 Jazzy
package set, and contributions are expected to match the conventions already
established in this codebase rather than introduce new ones. When in doubt,
grep for how an existing package already solved the same problem and follow
that pattern.

## Table of Contents

- [Code style](#code-style)
- [Commit conventions](#commit-conventions)
- [Testing standards](#testing-standards)
- [Security](#security)
- [Documentation standards](#documentation-standards)
- [License](#license)

## Code style

- **Language defaults.** C++17 is the primary language across every package
  (`prox_mpc_core`, `prox_mpc_controller`, `prox_mpc_obstacle_tracker`,
  `prox_mpc_msgs`, `prox_mpc_test_models`, `prox_mpc_demo`,
  `prox_mpc_benchmark`). Python is used only where the repo already uses it:
  the orchestration/analysis scripts under `prox_mpc_benchmark/scripts/`,
  targeting Python 3.12. Those scripts live in an `ament_cmake` package
  (`prox_mpc_benchmark/package.xml` declares `<build_type>ament_cmake</build_type>`)
  and are installed, not built as an `ament_python` package - follow that
  pattern rather than converting a package to `ament_python`.
- **CMake.** `cmake_minimum_required(VERSION 3.28)` is the floor in every
  package's `CMakeLists.txt`; do not lower it.
- **Formatting is `ament_uncrustify`-only.** As stated in the package READMEs
  (e.g. `prox_mpc_core/README.md`, `prox_mpc_controller/README.md`,
  `prox_mpc_obstacle_tracker/README.md`): `cpplint` and `ament_copyright` are
  disabled - uncrustify is the single enforced C++ formatter, and files carry
  a short SPDX header with the full text in [LICENSE](LICENSE), for example:

  ```cpp
  // Copyright 2026 Simone Contorno
  // SPDX-License-Identifier: Apache-2.0
  ```

  Match this two-line header (adapted for `#` comments in Python) at the top
  of every new source file; do not add a full license block per file.
- **Lint runs through `colcon test`, not standalone.** Every package's
  `CMakeLists.txt` calls `find_package(ament_lint_auto REQUIRED)` and
  `ament_lint_auto_find_test_dependencies()` under `BUILD_TESTING`, and
  `.github/workflows/ci.yaml` invokes `colcon test --return-code-on-test-failure`
  after the build. That is the whole lint path in this repo - there is no
  separate `ament_uncrustify --reformat` or standalone lint invocation in CI,
  so verify locally the same way: build, then `colcon test` in your overlay.
- **RAII and ownership.** Use `std::unique_ptr` by default for exclusive
  ownership (e.g. `prox_mpc_obstacle_tracker`'s `std::unique_ptr<Tracker>
  tracker_`); reserve `std::shared_ptr` for genuine shared ownership, such as
  the tracker node's shared ROS infrastructure objects
  (`std::shared_ptr<tf2_ros::Buffer>`, the `LifecyclePublisher`). Never store a
  `std::shared_ptr` by reference or use one solely to extend an object's
  lifetime; pass `const std::shared_ptr&` when only observing it.
- **Named constants over magic numbers**, and **explicit narrowing
  conversions** - narrow to a lower-precision type only at a tightly scoped
  boundary, with a comment explaining why (see the existing exceptions called
  out in ROS parameter and Eigen/solver code for precedent).

## Commit conventions

- **Branching follows GitHub Flow**: `main` is the stable branch, `dev` is the
  integration branch, and topic work happens on `feat/*`, `fix/*`, `chore/*`,
  or `test/*` branches merged in via pull request.
- **Commit messages follow Conventional Commits**, scoped to the package or
  area they touch, matching real history in this repo, for example:
  - `feat(controller): predictive dynamic-obstacle avoidance and model speed-cap forwarding`
  - `test(core): cover model input velocity-bound (v_min/v_max) override`
  - `docs(benchmark): matched-cap results table and fair-comparison prose`
  - `chore(release): bump packages to 1.0.0`
- **One logical change per commit.** Keep unrelated refactors, formatting-only
  changes, and behavior changes in separate commits so the history stays
  reviewable and bisectable.
- **No DCO / sign-off is currently required.** This repository has no
  `Signed-off-by` trailer convention in its commit history and no existing
  `CONTRIBUTING`-adjacent policy or `.github/` template requiring one - do not
  add a sign-off trailer unless a maintainer asks for it in review.

## Testing standards

- **Frameworks.** Every test in this repo is a GoogleTest (with GMock
  available) suite registered via `ament_add_gtest` - there is no
  `ament_add_pytest_test`, no `launch_testing`, and no `.py` test file
  anywhere in the tree. If you add Python-facing behavior that needs its own
  test (as opposed to being exercised through a C++ node under test), discuss
  the framework choice in the PR first rather than assuming pytest is already
  wired up.
- **Where tests live.** Tests live in `<package>/test/`, one `.cpp` file per
  suite, registered in that package's `CMakeLists.txt` under
  `if(BUILD_TESTING)`. Current suites, for reference:
  - `prox_mpc_core/test/`: `test_model_interface`, `test_mpc_regression`,
    `test_custom_model`, `test_obstacle_k`, `test_utils`.
  - `prox_mpc_controller/test/test_prox_mpc_controller.cpp`.
  - `prox_mpc_obstacle_tracker/test/`: `test_clustering`, `test_imm_filter`,
    `test_tracker`, `test_obstacle_tracker_node`.
  - `prox_mpc_demo/test/test_simulation_node.cpp`.
  - `prox_mpc_benchmark/test/`: `test_metrics`, `test_obstacle_field`.
  - `prox_mpc_msgs` and `prox_mpc_test_models` currently register no gtest
    suites - only `ament_lint_auto` runs under `BUILD_TESTING` for those two
    packages (they are IDL-only and a pluginlib-loaded model library,
    respectively).
- **Unit vs. integration pattern.** Follow the split this repo already uses
  rather than inventing a new one:
  - ROS-free core logic is tested directly, with no node/executor in the
    loop - e.g. `prox_mpc_obstacle_tracker`'s `test_clustering`,
    `test_imm_filter`, and `test_tracker` link only against
    `${PROJECT_NAME}_core` and exercise clustering, filter, and
    association/lifecycle logic as plain C++; `prox_mpc_core`'s five suites
    test the model/solver interface the same way.
  - ROS-facing surfaces are tested through their public interface, in the
    same process, with the test owning `rclcpp` init/shutdown via its own
    `main` (`SKIP_LINKING_MAIN_LIBRARIES`): `test_prox_mpc_controller` brings
    up a `LifecycleNode`, a `tf2` buffer, and a `Costmap2DROS` and drives
    every `nav2_core::Controller` method and fail-safe branch through the
    plugin's public surface; `test_obstacle_tracker_node` drives the
    lifecycle-node transition ladder and the scan-to-publish path against a
    synthetic scan; `test_simulation_node` drives the demo's control cycle
    directly without a wall timer. There is no `launch_testing`-based
    integration test anywhere in this repo yet - if you need true multi-node,
    multi-process system coverage, that would be new territory here, and per
    the house convention it belongs in a dedicated system-test package, not
    bolted onto one of the existing library packages.
- **Run tests exactly as CI does.** `.github/workflows/ci.yaml` builds once
  with `--symlink-install` and `--coverage` flags, then runs
  `colcon test --return-code-on-test-failure` followed by
  `colcon test-result --verbose` across the whole workspace (no package
  matrix - one `build-and-test` job covers every package). Reproduce that
  locally from your overlay:

  ```bash
  colcon build --symlink-install
  colcon test --packages-select <package> --return-code-on-test-failure
  colcon test-result --all --verbose
  ```

  Drop `--packages-select <package>` to match the CI invocation exactly
  (it tests the whole workspace); use it locally to iterate on one package
  faster. A second CI job, `clang-tsa`, rebuilds the whole workspace with
  Clang and `-Wthread-safety` - it does not run tests, only the
  thread-safety-annotated build, so a change to mutex-guarded state in
  `prox_mpc_controller` or `prox_mpc_obstacle_tracker` should also be checked
  against that build if you touch shared/guarded state.
- **Coverage: aspirational target, not an enforced CI gate.** The house
  convention (see the `ros2-testing-ci` skill) targets >= 95% line coverage.
  In this repo today, CI generates a filtered `lcov` report
  (`coverage_filtered.info`, with `/opt/*`, `/usr/*`, `*/test/*`, and
  `*/build/*` excluded) and uploads it as a build artifact
  (`.github/workflows/ci.yaml`, the `Generate coverage report` and
  `Upload coverage artifact` steps) - but nothing in the workflow reads a
  percentage back out of that report or fails the job on it. There is no
  `fail_under`, no Codecov config, and no other coverage gate anywhere in
  `.github/workflows/` or the package tree. Treat 95% as the target to aim
  for and a number worth checking locally (`genhtml` on the same
  `coverage_filtered.info` shape, per the `ros2-testing-ci` skill's
  `references/coverage.md`), not as something that currently blocks a PR.
- **Ship tests with the change, in the same PR.** Recent history mostly
  follows this: `c854679` (`feat(obstacle_tracker): IMM CV+CTRV tracking...`)
  added the new `test_imm_filter` suite and extended the three existing tracker
  suites (`test_clustering`, `test_tracker`, `test_obstacle_tracker_node`)
  alongside the new filter and clustering code in the same commit; `d78ff1a`
  (`feat(controller): predictive dynamic-obstacle avoidance...`) added 205
  lines to `test_prox_mpc_controller.cpp` alongside the behavior change; and
  `3580c0c` is a dedicated `test(core): ...` commit closing a gap left by a
  prior change. This is not exceptionless - `dae7906`
  (`feat(benchmark): matched-cap fair harness...`) landed a real behavior
  change (a genuine matched speed cap, a sensor-noise model, new scenarios)
  touching 22 files with no test-file update, so it is not a hard gate today.
  Default to adding or updating a test in the same PR as any new feature or
  bug fix; if you skip it, say why in the PR description rather than leaving
  it silent.

## Security

- **Never commit secrets, keys, or credentials.** Nothing in CI will catch a
  leaked key (see [SECURITY.md](SECURITY.md#what-ci-does-and-does-not-check)), so
  the review you do before you commit is the only gate. The tracked tree is
  currently clean: the only credential-shaped strings in it are the public
  maintainer emails in each `package.xml` and in
  `prox_mpc_benchmark/models/ackermann_robot/model.config`, which are contact
  addresses, not secrets. Keep it that way. Before every commit, read your
  staged diff and run a quick sweep, for example:

  ```bash
  git diff --staged
  git grep -niE 'api[_-]?key|secret|password|-----BEGIN' -- $(git diff --staged --name-only)
  ```

  This is a robotics-control package - a Nav2 controller plugin, a lidar
  obstacle tracker, and a benchmarking harness - with no login flow, no API
  tokens, no service credentials, and no config file that should ever hold one.
  If a change appears to need a secret, that is a design smell to raise in the
  PR, not something to inline.
- **Dependency and license review is required for any new dependency.** The
  project runs a **permissive / NVIDIA-TAO-compatible** license policy,
  documented in [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md): only
  Apache-2.0, BSD-2-Clause, BSD-3-Clause, MIT, and header-only *unmodified*
  MPL-2.0 are in policy (Eigen's MPL-2.0 headers are the sole MPL case, used
  unmodified). Copyleft licenses (GPL/LGPL/AGPL, or a modified MPL file) are out
  of policy - do not add one without maintainer sign-off. When you add a
  dependency you must: (1) add a row to the `THIRD_PARTY_LICENSES.md` inventory
  table stating how it is used, its license, and a one-line rationale; (2)
  declare it explicitly in `package.xml` with the correct tag
  (`<depend>`, `<build_depend>`, `<exec_depend>`, or `<test_depend>`) and wire
  it in `CMakeLists.txt`; and (3) prefer a rosdep-provisioned apt key over a
  vendored copy, matching how `ros-jazzy-proxsuite` is pinned
  (`version_gte="0.6.5"`) and `ros-jazzy-vector-pursuit-controller` is declared
  as a runtime `exec_depend`. Bundled *assets* (URDF, world files, RViz configs)
  are inventoried by the package that ships them -
  [prox_mpc_demo](prox_mpc_demo/THIRD_PARTY_LICENSES.md) and
  [prox_mpc_benchmark](prox_mpc_benchmark/THIRD_PARTY_LICENSES.md). CI enforces
  the dependency half of this with
  `.github/scripts/check_dependency_inventory.py`.
- **If you touch the workflows, keep their hardening.**
  `.github/workflows/ci.yaml` declares least-privilege
  `permissions: contents: read` and pins every third-party action by full commit
  SHA with the version tag in a trailing comment
  (`actions/checkout@34e1148...  # v4.3.1`) rather than by a mutable tag. Preserve
  both. What CI does and does not scan for is listed in
  [SECURITY.md](SECURITY.md#what-ci-does-and-does-not-check) - check a new or
  bumped dependency against [OSV](https://osv.dev/) yourself, because nothing
  automated does.
- **Reporting a security concern.** Do not open a public issue or PR that
  describes an exploitable flaw. Follow [SECURITY.md](SECURITY.md), which
  covers the private reporting routes, what to include, response expectations,
  and what is in and out of scope (notably: SROS2 / DDS transport security is
  out of scope for this repository).

## Documentation standards

- **Package README template.** Every package README in this repo, including the
  root [README.md](README.md) (itself brought into this same template),
  follows one shape: `# Title`, an optional badge, an intro paragraph, a
  `## Table of Contents` (a bullet list of anchor links to every `##`
  section), the body sections, and a closing `## License` footer.
  `prox_mpc_core/README.md` and `prox_mpc_benchmark/README.md` are the
  reference examples - match this shape for a new or restructured package
  doc rather than inventing a different outline.
- **Markdown style.** The rules that govern this file govern every Markdown
  file in the repo: GitHub-Flavored Markdown with ATX headers, one sentence
  per line, exactly one blank line before and after every heading and every
  fenced code block, a language identifier on every fenced code block (e.g.
  `cpp`, `bash`, `yaml`, or `xml`), and relative links for anything that lives
  in this repository. Use Mermaid only where a diagram materially helps the
  reader, not by default on every page. [doc/architecture.md](doc/architecture.md)
  and the root [README.md](README.md)'s "Per-cycle control loop" diagram are the
  real, working examples of the conventions already in use here: `flowchart TD`
  / `flowchart LR` for orientation, `<br/>` for a line break inside a node
  label, `[(...)]` for an external system (for example Nav2),
  `subgraph name[...] ... end` to group related nodes, and three edge styles
  used deliberately - a plain `-->` for an unlabeled dependency, a labeled
  `-- text -->` for a named relationship (for example
  `trk -- tracked_obstacles --> ctrl`), and a dashed `-. text .->` for a
  `pluginlib`-style load-by-name relationship rather than a build dependency
  (for example `demo -. launches .-> ctrl`). Keep node and edge labels in
  plain English, and keep LaTeX out of node text.
- **What a package doc should cover.** Where applicable to what is being
  added or changed, a package doc should cover purpose, prerequisites,
  installation, usage/launch flows, parameters (name, type, default, unit,
  meaning), interfaces (topics, services, actions, TF) with their QoS,
  lifecycle behavior, architecture or data flow, troubleshooting, and
  verification. `prox_mpc_core/README.md` and
  `prox_mpc_controller/doc/architecture.md` are the two documents in this
  repo that already do this well end to end - the former for a library
  package's README shape, the latter for a lifecycle-plugin's
  QoS-annotated interface table, parameter tables, and state-diagram
  lifecycle section. Follow one of them as the working model rather than
  restating this as an abstract checklist with no anchor in this repo.
- **Keep docs in sync with the code.** A new or changed parameter, topic,
  service, action, or QoS profile must be reflected in the relevant package
  doc and README table in the same PR that changes it, not as a follow-up.

For repository layout and file templates, follow what already exists in this
codebase.

## License

[Apache-2.0](LICENSE).
