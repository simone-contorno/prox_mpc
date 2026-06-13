# Third-party licenses

This file records third-party material redistributed in this repository and the
license under which it is used.
The package's own code and assets are covered by the repository `LICENSE`.

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
