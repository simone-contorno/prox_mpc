# Third-party licenses

This file records third-party material redistributed in this repository and the
license under which it is used.
The package's own code and assets are covered by the repository `LICENSE`, and
the build/link dependencies are inventoried in the repository
[THIRD_PARTY_LICENSES.md](../THIRD_PARTY_LICENSES.md).

## Gazebo world files

- Files: `worlds/prox_mpc_world.sdf.xacro`, `worlds/prox_mpc_open.sdf.xacro`
- Source: derived from
  [`nav2_minimal_tb3_sim`](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation/tree/main/nav2_minimal_tb3_sim)
  `worlds/tb3_sandbox.sdf.xacro`
- License: Apache-2.0 (same text as the repository `LICENSE`)
- Modifications: `prox_mpc_world.sdf.xacro` mirrors `tb3_sandbox` (so the stock
  `tb3_sandbox` map still aligns and AMCL localizes) and adds, behind the
  `obstacles` xacro arg, two static boxes and one trajectory-animated dynamic
  actor not present in the static map; `prox_mpc_open.sdf.xacro` reuses the same
  upstream system-plugin / `sun` / `ground_plane` / scene / physics boilerplate
  inside an original open 6x6 m room.
- Runtime reference: `prox_mpc_world.sdf.xacro` references
  `model://turtlebot3_world` by URI, a runtime dependency resolved from the
  installed Gazebo model path; it is not redistributed in this repository.

## RViz configuration

- File: `rviz/nav2_simulation.rviz`
- Source: derived from
  [`nav2_bringup`](https://github.com/ros-navigation/navigation2)
  `rviz/nav2_default_view.rviz`
- License: Apache-2.0 (same text as the repository `LICENSE`)
- Modifications: the Nav2 panel, display tree, tool set, and `nav2_rviz_plugins`
  classes are carried over from the upstream view - the large majority of the
  file is unchanged from it. Added two ProxMPC-specific displays, a `Path` on
  `/prox_mpc_local_plan` ("ProxMPC Local Plan") and "ProxMPC Predicted
  Obstacles" on `/prox_mpc_predicted_obstacles`; added the `SetGoal` tool on
  `/goal_pose`; and replaced the upstream `TopDownOrtho` view controller with
  `Orbit`.

## R2D2 URDF model

- File: `urdf/r2d2.urdf`
- Source: [ros/urdf_tutorial](https://github.com/ros/urdf_tutorial) (ros2 branch), `urdf/06-flexible.urdf`
- License: BSD-3-Clause
- Modifications: trimmed the gripper pole, fingers, and tips (which referenced
  external `package://urdf_tutorial` meshes) to make the file self-contained;
  re-rooted the tree at an empty `base_link` and lifted the body by 0.47 m so the
  wheels rest on the ground plane; renamed the robot to `r2d2`.

The visuals are geometric primitives only, so no meshes from the upstream package
are redistributed and no runtime dependency on `urdf_tutorial` is introduced.

### License text

```text
Software License Agreement (BSD License)

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```
