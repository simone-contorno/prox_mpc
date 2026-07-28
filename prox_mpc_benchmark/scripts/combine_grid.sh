#!/usr/bin/env bash
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0
#
# Combine four clips into one labelled 2x2 grid mp4 via ffmpeg xstack, plus a poster
# still. Two modes:
#   * Scenario grid (default): four per-scenario clips from one --input-dir; labels
#     come from the scenario names (ProxMPC across the four scenarios).
#   * Explicit grid: pass four clip paths with --inputs and four labels with
#     --labels -- e.g. one scenario across four controllers.
#
# Cell layout (xstack 0_0|w0_0|0_h0|w0_h0): top-left, top-right, bottom-left,
# bottom-right. Clips are produced by record_scenarios.py. Only ffmpeg is required.

set -euo pipefail

# Resolve the source tree even when invoked via a colcon --symlink-install symlink,
# so the default input dir points at prox_mpc_benchmark/results/videos.
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

INPUT_DIR="$PKG_DIR/results/videos"
OUTPUT=""
CELL="960x540"
FRAMERATE="30"
GIF_WIDTH="960"
GIF_FPS="10"
BASENAMES="nav2_open,static_box,dynamic_line_forward,dynamic_circle"
INPUTS_ARG=""
LABELS_ARG=""

declare -A LABELMAP=(
  [nav2_open]="No obstacle"
  [static_box]="Static box"
  [dynamic_line_forward]="Dynamic line"
  [dynamic_circle]="Dynamic circle"
)

usage() {
  cat <<'EOF'
Usage: combine_grid.sh [options]
  -i, --input-dir DIR      directory holding the per-scenario clips
                           (default <pkg>/results/videos)
  -o, --output FILE        grid mp4 path (default <input-dir>/prox_mpc_demo_grid.mp4;
                           required with --inputs)
  -c, --cell WxH           per-cell size (default 960x540 -> 1920x1080 grid)
  -r, --framerate N        output frame rate (default 30)
      --gif-width PX        inline-GIF width (default 960; height auto)
      --gif-fps N           inline-GIF frame rate (default 10)
  -b, --basenames a,b,c,d  clip basenames in <input-dir> (scenario-grid mode)
      --inputs p1,p2,p3,p4 four explicit clip paths (explicit-grid mode)
      --labels A,B,C,D     four cell labels (with --inputs)
  -h, --help               this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -i|--input-dir) INPUT_DIR="$2"; shift 2 ;;
    -o|--output) OUTPUT="$2"; shift 2 ;;
    -c|--cell) CELL="$2"; shift 2 ;;
    -r|--framerate) FRAMERATE="$2"; shift 2 ;;
    --gif-width) GIF_WIDTH="$2"; shift 2 ;;
    --gif-fps) GIF_FPS="$2"; shift 2 ;;
    -b|--basenames) BASENAMES="$2"; shift 2 ;;
    --inputs) INPUTS_ARG="$2"; shift 2 ;;
    --labels) LABELS_ARG="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

CELL_W="${CELL%x*}"
CELL_H="${CELL#*x}"

# Resolve the four clip PATHS and LABELS from whichever mode was requested.
PATHS=()
LABELS=()
if [[ -n "$INPUTS_ARG" ]]; then
  IFS=',' read -r -a PATHS <<<"$INPUTS_ARG"
  IFS=',' read -r -a LABELS <<<"$LABELS_ARG"
  if [[ ${#PATHS[@]} -ne 4 || ${#LABELS[@]} -ne 4 ]]; then
    echo "error: --inputs and --labels each need exactly 4 comma-separated values" >&2
    exit 2
  fi
  if [[ -z "$OUTPUT" ]]; then
    echo "error: --output is required with --inputs" >&2
    exit 2
  fi
else
  IFS=',' read -r -a NAMES <<<"$BASENAMES"
  if [[ ${#NAMES[@]} -ne 4 ]]; then
    echo "error: --basenames needs exactly 4 comma-separated names" >&2
    exit 2
  fi
  for name in "${NAMES[@]}"; do
    PATHS+=("$INPUT_DIR/$name.mp4")
    LABELS+=("${LABELMAP[$name]:-$name}")
  done
  OUTPUT="${OUTPUT:-$INPUT_DIR/prox_mpc_demo_grid.mp4}"
fi
GIF="${OUTPUT%.mp4}.gif"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "error: ffmpeg not found (install: sudo apt-get install -y ffmpeg)" >&2
  exit 3
fi

# Build the input list and the labelled/scaled filtergraph.
INPUTS=()
FILTER=""
for i in 0 1 2 3; do
  clip="${PATHS[$i]}"
  if [[ ! -f "$clip" ]]; then
    echo "error: missing clip $clip (run record_scenarios.py first)" >&2
    exit 4
  fi
  INPUTS+=(-i "$clip")
  label="${LABELS[$i]}"
  FILTER+="[$i:v]scale=${CELL_W}:${CELL_H},drawtext=text='${label}':"
  FILTER+="fontcolor=white:fontsize=28:box=1:boxcolor=black@0.5:boxborderw=8:"
  FILTER+="x=20:y=20[v$i];"
done
FILTER+="[v0][v1][v2][v3]xstack=inputs=4:layout=0_0|w0_0|0_h0|w0_h0[out]"

GRID_CMD=(ffmpeg -y "${INPUTS[@]}"
  -filter_complex "$FILTER"
  -map "[out]" -r "$FRAMERATE"
  -c:v libx264 -preset veryfast -pix_fmt yuv420p -movflags +faststart
  "$OUTPUT")

echo "+ ${GRID_CMD[*]}"
"${GRID_CMD[@]}"

# Inline GIF (palette two-pass for quality) so the grid renders directly in
# GitHub-flavoured Markdown and loops, without a click-to-download attachment.
PAL="$(mktemp --suffix=.png)"
trap 'rm -f "$PAL"' EXIT
GIF_VF="fps=${GIF_FPS},scale=${GIF_WIDTH}:-1:flags=lanczos"
echo "+ ffmpeg -i $OUTPUT -vf ${GIF_VF},palettegen ... | paletteuse -> $GIF"
ffmpeg -y -hide_banner -loglevel error -i "$OUTPUT" \
  -vf "${GIF_VF},palettegen=stats_mode=diff" "$PAL"
ffmpeg -y -hide_banner -loglevel error -i "$OUTPUT" -i "$PAL" \
  -lavfi "${GIF_VF},paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle" "$GIF"

echo "grid: $OUTPUT"
echo "gif:  $GIF"
