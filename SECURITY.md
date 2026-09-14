# Security Policy

## Supported versions

| Version | Supported |
| --- | --- |
| 2.0.0 | Yes |
| < 2.0.0 | No |

This repository ships seven ROS 2 Jazzy packages from a single source tree and
releases them together, so a fix lands for all of them at once. Only the latest
release is patched; there are no backport branches.

## Reporting a vulnerability

**Do not open a public issue, pull request, or discussion that describes an
exploitable flaw.**

Report privately, by either route:

- Open a [private security advisory](https://docs.github.com/en/code-security/security-advisories/guidance-on-reporting-and-writing-information-about-vulnerabilities/privately-reporting-a-security-vulnerability)
  on this repository (**Security** tab -> **Report a vulnerability**). This is
  preferred: it keeps the report, the fix, and the CVE request in one place.
- Or email the maintainer at `simone.contorno@outlook.it`, the address used
  across the `package.xml` manifests.

Please include:

- the affected package and version (or commit SHA),
- the impact - what an attacker gains, and what access they need to start,
- a minimal reproduction, ideally a launch file or parameter set,
- your ROS 2 distro, DDS vendor, and platform.

**What to expect.** This is a single-maintainer project, so response is
best-effort rather than contractual: acknowledgement within 7 days, an initial
assessment within 14 days, and a fix or a documented mitigation for confirmed
issues before the advisory is published. Reporters are credited in the advisory
unless they ask otherwise. If you have not heard back in 14 days, please send a
follow-up - the report was more likely missed than ignored.

## Scope

These packages are a Nav2 controller plugin, a lidar obstacle tracker, a
simulation demo, and a benchmarking harness. There is no login flow, no API
token, no service credential, and no network-facing service of our own. The
controller runs in-process inside `controller_server` as a pluginlib-loaded
`nav2_core::Controller`; the tracker is a lifecycle node on the local ROS graph.

### In scope

- Memory-safety or undefined-behaviour bugs reachable from message, parameter,
  or costmap input - the solver path, the clustering and IMM filter path, and
  the message conversions.
- A parameter or message value that drives the controller into an unsafe
  command rather than into its documented fail-safes (solver-failure
  deceleration, `nav2_core::NoValidControl` escalation, the non-finite-command
  guard, and the footprint veto).
- Denial of service through unbounded allocation or unbounded runtime in a
  control-rate callback.
- A build-, packaging-, or CI-level supply-chain weakness in this repository.

### Out of scope

- **ROS 2 / DDS transport security.** SROS2 enclaves and DDS-Security
  authentication and encryption are not configured anywhere in this repository:
  there is no `ROS_SECURITY_*` setup, no enclave, and no keystore in the tree.
  Securing the DDS transport is a deployment and system-integration concern for
  whoever fields these packages. If you deploy on a shared or untrusted network,
  apply SROS2 and network isolation at that layer.
- An attacker who already publishes on the robot's ROS graph. The ROS 2 trust
  model assumes graph participants are authorised; a hostile publisher on
  `/scan` or `/tracked_obstacles` is a transport-security problem, addressed by
  the layer above.
- Vulnerabilities in upstream dependencies (Nav2, ProxSuite, Eigen, Gazebo).
  Report those to the upstream project. If an upstream advisory affects how
  *this* repository uses that dependency, we do want to hear about it.
- Simulation-only behaviour in `prox_mpc_demo` and `prox_mpc_benchmark` with no
  bearing on the controller or tracker as deployed.

## What CI does and does not check

Stated plainly so nobody assumes a scan is catching things it is not.

**Runs today** (`.github/workflows/ci.yaml`): least-privilege
`permissions: contents: read`; every third-party action pinned by full commit
SHA; `ament_cppcheck` under `colcon test` with
`AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` set so it does not silently no-op; a
`sanitizers` job that rebuilds the workspace `Debug` under AddressSanitizer and
UndefinedBehaviorSanitizer with `-fno-sanitize-recover=all`, leak detection on,
and Eigen's own bounds assertions live, then runs the full test suite; and a
separate `clang-tsa` job that rebuilds the workspace under
`-Werror=thread-safety`, so an unguarded access to shared state fails the build.

**Does not run:** no secret scanner, no CodeQL or other deep SAST, and no
dependency-CVE audit (`pip-audit` / `trivy` / `grype` / OSV).
`.github/dependabot.yml` exists but covers the `github-actions` ecosystem only:
it bumps the SHA-pinned actions monthly and does not see the ROS/apt
dependencies, which rosdep resolves. Checking a new or bumped dependency against
[OSV](https://osv.dev/) is the contributor's and reviewer's job - see
[CONTRIBUTING.md](CONTRIBUTING.md#security).

## Hardening notes for deployers

- Run `controller_server` with no more privilege than it needs; nothing here
  requires root.
- The tracker and controller trust their input topics. On a shared network,
  enable SROS2 and restrict who may publish `/scan` and `/tracked_obstacles`.
- The parameters that bound resource use on constrained hardware -
  `max_obstacles`, `max_dynamic_obstacles`, `max_obstacle_scan_cells`,
  `max_iter_sqp`, `max_solve_time`, and the tracker's `max_clusters` and
  `max_tracks` - are validated at configure time: out-of-range values are
  clamped to a safe range, except the iteration and track caps (`max_iter_sqp`,
  `max_clusters`, `max_tracks`), which are rejected so the lifecycle transition
  fails. Set them deliberately rather than relying on defaults tuned for the
  shipped demos.
