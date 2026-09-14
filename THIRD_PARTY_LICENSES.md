# Third-party licenses (dependencies)

This file inventories the third-party libraries ProxMPC builds against, links, or
otherwise depends on, and the license each is used under.
The repository's own code and assets are covered by the repository
[LICENSE](LICENSE) (Apache-2.0) and [NOTICE](NOTICE).
Bundled *assets* (URDF, world files, RViz configurations) are inventoried by the
package that ships them:
[prox_mpc_demo/THIRD_PARTY_LICENSES.md](prox_mpc_demo/THIRD_PARTY_LICENSES.md)
and
[prox_mpc_benchmark/THIRD_PARTY_LICENSES.md](prox_mpc_benchmark/THIRD_PARTY_LICENSES.md).

## License policy

The dependency policy is **permissive / NVIDIA-TAO-compatible**: Apache-2.0,
BSD-2-Clause, BSD-3-Clause, MIT, and **header-only, unmodified MPL-2.0**.
Every dependency that is built against, linked, or redistributed falls inside
that set.
There is exactly one documented exception - `ffmpeg`, recorded below - and it is
neither built against nor linked.
MPL-2.0 is file-level (weak) copyleft; it is used here only as unmodified upstream
headers, so the per-file modify-then-redistribute trigger is not hit and the
obligation reduces to preserving the notice (done below).

### Documented exception - ffmpeg (GPL-2+)

The Debian `ffmpeg` binary package is licensed **GPL-2+**
(`/usr/share/doc/ffmpeg/copyright`: "the resulting binaries are licensed under
GPL v2+"), which is outside the permissive set above.
It is retained as a documented exception on the following basis.

- **Separate process, never linked.** `ffmpeg` is invoked only as an external
  executable - `prox_mpc_benchmark/scripts/record_scenarios.py` spawns it as a
  subprocess for x11grab screen capture, and
  `prox_mpc_benchmark/scripts/combine_grid.sh` shells out to it for the xstack
  grid and GIF passes.
  No repository code links, statically or dynamically, against any `libav*`
  library, and no `ffmpeg` source is redistributed here.
  Invoking a GPL program as a separate process does not place this repository's
  Apache-2.0 code under the GPL.
- **Developer-time tooling, not a shipped runtime component.** It is used to
  record and assemble comparison videos; no deployed ProxMPC node or plugin
  depends on it, and it is not on any control path.

The exception is therefore limited to the video-capture tooling.
Extending `ffmpeg` use into linked code, or redistributing `ffmpeg` binaries with
this repository, would change the analysis and must be re-reviewed.

## Inventory

| Library | Used as | License | Notes |
| --- | --- | --- | --- |
| `Eigen3` | C++ headers (`<Eigen/Dense>`, `<Eigen/Sparse>`) | MPL-2.0 (with BSD-3 files) | Header-only, unmodified. Only the Dense and Sparse modules are included. Two files reachable through `<Eigen/Sparse>` (`SparseCholesky/SimplicialCholesky.h`, `OrderingMethods/Amd.h`) are dual-licensed **LGPL-2.1+ or MPL-2.0**; Eigen 3.4.0 ships both with MPL-2.0-only file headers and this project takes the MPL-2.0 arm, so no LGPL obligation attaches. No LGPL-only backend (`SuperLU`, `UmfPack`, `Cholmod`, `Pardiso`, `SPQR`) is included or linked. |
| `ProxSuite` | Linked QP solver (`proxsuite::proxsuite`) | BSD-2-Clause | See the canonical version note in [prox_mpc_core's NMPC doc](prox_mpc_core/doc/nmpc.md) and the proxsuite provisioning note below. |
| `SIMDe` | Transitive apt dependency (`libsimde-dev`, a `Depends` of `ros-jazzy-proxsuite`) | MIT | Not vendored: ProxSuite ships only a CMake finder and includes the headers from `libsimde-dev` in its dense linalg core. Not used directly by this project. |
| ROS 2 client/message/Nav2 libraries (`rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `tf2`, `tf2_ros`, `pluginlib`, `nav2_core`, `nav2_costmap_2d`, `nav2_util`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `lifecycle_msgs`, `visualization_msgs`) | Linked / message generation | Apache-2.0 or BSD-3-Clause | The ROS 2 Jazzy core permissive set. |
| `vector_pursuit_controller` (apt `ros-jazzy-vector-pursuit-controller` v2.0.0, maintained by Black Coffee Robotics) | Installed `nav2_core::Controller` plugin (pluginlib-loaded at runtime, not linked) | Apache-2.0 | `prox_mpc_benchmark`'s single external fair peer for the cross-controller comparison; a runtime `exec_depend`, not a build/link dependency. |
| `nav2_graceful_controller` (apt `ros-jazzy-nav2-graceful-controller`) | Installed `nav2_core::Controller` plugin (pluginlib-loaded at runtime, not linked) | Apache-2.0 | A `prox_mpc_benchmark` cross-controller comparison peer from the Nav2 distribution; a runtime `exec_depend`, not a build/link dependency. |
| `ffmpeg` (apt `ffmpeg`) | Invoked as a separate process by the demo-video tooling (`prox_mpc_benchmark/scripts/record_scenarios.py`, `scripts/combine_grid.sh`); never linked, never redistributed | GPL-2+ (Debian binary build) | **Documented policy exception** - see [Documented exception - ffmpeg](#documented-exception---ffmpeg-gpl-2) above. Developer-time recording utility only; not a shipped runtime component. |

## Coverage of the ROS 2 distribution set

The "ROS 2 client/message/Nav2 libraries" row above is a blanket entry. The keys
it covers are enumerated here so the claim is checkable rather than rhetorical:
every one is Apache-2.0, BSD-3-Clause, or MIT - all inside the permissive policy
set above - consumed as a normal rosdep-provisioned dependency and never vendored
or modified. Most are ROS 2 Jazzy distribution packages; the exceptions to note
are `nav2_mppi_controller`, which declares `MIT` rather than Apache-2.0, and the
two Ubuntu apt keys `python3-yaml` (PyYAML, MIT/Expat) and `python3-numpy`
(BSD-3-Clause, with permissively licensed bundled components), which come from
Ubuntu rather than the ROS distribution.

`.github/scripts/check_dependency_inventory.py` (run in CI) fails if a
`package.xml` declares a rosdep key that appears neither in the inventory table
nor in this list, so a new dependency cannot enter without an attribution
decision.

```text
ament_cmake
ament_cmake_gtest
ament_cmake_pytest
ament_cmake_python
ament_index_python
ament_lint_auto
ament_lint_common
builtin_interfaces
dwb_core
dwb_critics
joint_state_publisher
launch
launch_ros
nav2_behaviors
nav2_bringup
nav2_bt_navigator
nav2_controller
nav2_lifecycle_manager
nav2_map_server
nav2_minimal_tb3_sim
nav2_mppi_controller
nav2_msgs
nav2_navfn_planner
nav2_planner
nav2_regulated_pure_pursuit_controller
nav2_util
python3-numpy
python3-yaml
rcl_interfaces
rclpy
rcpputils
rcutils
robot_state_publisher
ros2bag
ros2launch
ros_gz_bridge
ros_gz_sim
rosbag2_py
rosidl_default_generators
rosidl_default_runtime
rosidl_typesupport_introspection_cpp
rviz2
std_msgs
xacro
```

## ProxSuite - canonical version

The reproducible, rosdep-provisioned dependency is the apt key
`ros-jazzy-proxsuite` (a clean CI runner or a provisioned deployment host installs
it via `rosdep install`).
The packages pin it as the floor with `version_gte="0.6.5"` in their
`package.xml`.
The bit-exact regression baseline in
`prox_mpc_core/test/test_mpc_regression.cpp` must be confirmed against the
provisioned ProxSuite; its 1e-6 tolerance leaves margin for minor numerical
variation, and CI surfaces any divergence rather than reporting a false green.

## License texts

### ProxSuite - BSD-2-Clause

```text
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### SIMDe - MIT

```text
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

### Eigen - MPL-2.0

Eigen is licensed primarily under the Mozilla Public License, Version 2.0; the
full text is at <https://www.mozilla.org/MPL/2.0/>.
The headers carry the standard MPL-2.0 notice:

```text
This Source Code Form is subject to the terms of the Mozilla Public License,
v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
one at https://mozilla.org/MPL/2.0/.
```

### ROS 2 / Nav2

Apache-2.0 (same text as the repository [LICENSE](LICENSE)) and BSD-3-Clause; see
each upstream package for its individual notice.
