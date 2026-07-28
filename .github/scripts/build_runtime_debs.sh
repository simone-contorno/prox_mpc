#!/usr/bin/env bash
# Build Debian packages for the ProxMPC controller runtime.
#
# Runtime closure of the Nav2 controller plugin: prox_mpc_msgs -> prox_mpc_core
# -> prox_mpc_controller. prox_mpc_test_models is built too because it is a
# test_depend of the controller and bloom resolves test dependencies; it lands in
# the controller's Build-Depends, never in its Depends, so installing
# ros-jazzy-prox-mpc-controller does not pull it in.
#
# The demo and benchmark packages are development tooling and are not shipped.
#
# Order matters: each package is installed before the next is built, because
# bloom resolves the intra-repository dependencies through apt and rosdep, not
# through the source tree.
#
# Usage:
#   .github/scripts/build_runtime_debs.sh [output_dir]
#
# Requires: python3-bloom, python3-rosdep, fakeroot, debhelper, dh-python, and
# root or sudo to install each intermediate .deb.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="${1:-${REPO_ROOT}/debs}"

# Dependency order. Do not reorder.
PACKAGES=(prox_mpc_msgs prox_mpc_core prox_mpc_test_models prox_mpc_controller)

if [ "$(id -u)" -eq 0 ]; then
  SUDO=''
elif command -v sudo >/dev/null 2>&1; then
  SUDO='sudo'
else
  echo "error: need root or sudo to install intermediate packages" >&2
  exit 1
fi

for tool in bloom-generate fakeroot dh rosdep; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "error: missing '$tool'. Install:" >&2
    echo "  sudo apt-get install -y python3-bloom python3-rosdep fakeroot debhelper dh-python" >&2
    exit 1
  fi
done

# These packages are not in the ROS distro index yet, so rosdep cannot resolve
# the intra-repository dependencies and bloom aborts on prox_mpc_controller with
# "Could not resolve rosdep key 'prox_mpc_core'". Register the local rules from
# .github/rosdep/ alongside the upstream sources. ROSDEP_SOURCE_PATH replaces the
# default source list rather than extending it, so the upstream list is copied in
# too - without it, keys like 'eigen' stop resolving.
ROSDEP_LOCAL="$(mktemp -d)"
trap 'rm -rf "${ROSDEP_LOCAL}"' EXIT
cp "${REPO_ROOT}/.github/rosdep/prox_mpc.yaml" "${ROSDEP_LOCAL}/"
echo "yaml file://${ROSDEP_LOCAL}/prox_mpc.yaml" > "${ROSDEP_LOCAL}/99-prox-mpc.list"
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  cp /etc/ros/rosdep/sources.list.d/20-default.list "${ROSDEP_LOCAL}/"
else
  echo "error: /etc/ros/rosdep/sources.list.d/20-default.list not found; run 'sudo rosdep init'" >&2
  exit 1
fi
export ROSDEP_SOURCE_PATH="${ROSDEP_LOCAL}"
rosdep update >/dev/null

mkdir -p "$OUT_DIR"
# ROS's setup.bash reads unset variables, which is fatal under `set -u`; relax it
# for the source only.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

for pkg in "${PACKAGES[@]}"; do
  echo "==> Building ${pkg}"
  pushd "${REPO_ROOT}/${pkg}" >/dev/null

  rm -rf debian .obj-* obj-*
  # A failed rosdep resolve makes bloom prompt, which dies with an opaque
  # "Inappropriate ioctl for device" on a non-tty. </dev/null keeps that a clean
  # failure instead of a hang.
  bloom-generate rosdebian --os-name ubuntu --ros-distro "${ROS_DISTRO}" </dev/null
  fakeroot debian/rules binary

  rm -rf debian .obj-* obj-*
  popd >/dev/null

  # bloom writes the .deb next to the package directory. Debug-symbol .ddeb files
  # are not shipped.
  mv "${REPO_ROOT}"/*.deb "${OUT_DIR}/" 2>/dev/null || true
  rm -f "${REPO_ROOT}"/*.ddeb

  deb="$(ls -t "${OUT_DIR}"/ros-"${ROS_DISTRO}"-"${pkg//_/-}"_*.deb 2>/dev/null | head -1)"
  if [ -z "${deb}" ]; then
    echo "error: no .deb produced for ${pkg}" >&2
    exit 1
  fi
  echo "==> Installing ${deb}"
  $SUDO apt-get install -y --allow-downgrades "${deb}"
done

echo
echo "Built packages in ${OUT_DIR}:"
ls -1 "${OUT_DIR}"/*.deb
