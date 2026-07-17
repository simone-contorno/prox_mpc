# Third-party licenses (build/link dependencies)

This file inventories the third-party libraries ProxMPC builds against or links,
and the license each is used under.
The repository's own code and assets are covered by the repository
[LICENSE](LICENSE) (Apache-2.0) and [NOTICE](NOTICE).
Bundled demo *assets* (URDF, world files) are inventoried separately in
[prox_mpc_demo/THIRD_PARTY_LICENSES.md](prox_mpc_demo/THIRD_PARTY_LICENSES.md).

## License policy

The dependency policy is **permissive / NVIDIA-TAO-compatible**: Apache-2.0,
BSD-2-Clause, BSD-3-Clause, MIT, and **header-only, unmodified MPL-2.0**.
Every dependency below falls inside that set.
MPL-2.0 is file-level (weak) copyleft; it is used here only as unmodified upstream
headers, so the per-file modify-then-redistribute trigger is not hit and the
obligation reduces to preserving the notice (done below).

## Inventory

| Library | Used as | License | Notes |
| --- | --- | --- | --- |
| Eigen3 | C++ headers (`<Eigen/Dense>`, `<Eigen/Sparse>`) | MPL-2.0 (with BSD-3 files) | Header-only, unmodified. Only the Dense and Sparse modules are included; **no LGPL module** (`SparseCholesky` non-impl, `SuperLU`, `UmfPack`, `Cholmod`, `Pardiso`, `SPQR`) is included or linked. |
| ProxSuite | Linked QP solver (`proxsuite::proxsuite`) | BSD-2-Clause | See the canonical version note in [prox_mpc_core's NMPC doc](prox_mpc_core/doc/nmpc.md) and the proxsuite provisioning note below. |
| SIMDe | Transitive (vendored inside ProxSuite for portable SIMD) | MIT | Pulled in via ProxSuite headers; not used directly. |
| ROS 2 client/message/Nav2 libraries (`rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `tf2`, `tf2_ros`, `pluginlib`, `nav2_core`, `nav2_costmap_2d`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `lifecycle_msgs`, `visualization_msgs`) | Linked / message generation | Apache-2.0 or BSD-3-Clause | The ROS 2 Jazzy core permissive set. |
| `vector_pursuit_controller` (apt `ros-jazzy-vector-pursuit-controller` v2.0.0, maintained by Black Coffee Robotics) | Installed `nav2_core::Controller` plugin (pluginlib-loaded at runtime, not linked) | Apache-2.0 | `prox_mpc_benchmark`'s single external fair peer for the cross-controller comparison; a runtime `exec_depend`, not a build/link dependency. |

## ProxSuite — canonical version

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

### ProxSuite — BSD-2-Clause

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

### SIMDe — MIT

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

### Eigen — MPL-2.0

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
