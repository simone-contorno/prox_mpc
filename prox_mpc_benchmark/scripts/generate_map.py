#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Generate a Nav2 occupancy map (PGM + YAML) from a room extent and obstacles.

The world and the map are produced from one source and cannot drift out of sync.
The matching world xacro takes the same room_x / room_y; presets are 7 / 15 / 30 m.

Occupancy convention (map_server default, negate=0): 0 = occupied (walls,
obstacles), 254 = free, 205 = unknown.
"""

import argparse
from pathlib import Path

import numpy as np

OCC, FREE = 0, 254


def world_to_cell(x, y, origin, res, height):
    cx = int((x - origin[0]) / res)
    cy = height - 1 - int((y - origin[1]) / res)
    return cx, cy


def fill_box(grid, origin, res, cx, cy, sx, sy):
    h, w = grid.shape
    x0, y0 = world_to_cell(cx - sx / 2.0, cy + sy / 2.0, origin, res, h)
    x1, y1 = world_to_cell(cx + sx / 2.0, cy - sy / 2.0, origin, res, h)
    x0, x1 = max(0, min(x0, x1)), min(w, max(x0, x1) + 1)
    y0, y1 = max(0, min(y0, y1)), min(h, max(y0, y1) + 1)
    grid[y0:y1, x0:x1] = OCC


def generate(room_x, room_y, res, wall, obstacles, out_stem):
    w = int(round(room_x / res))
    h = int(round(room_y / res))
    origin = [-room_x / 2.0, -room_y / 2.0, 0.0]
    grid = np.full((h, w), FREE, dtype=np.uint8)

    t = max(1, int(round(wall / res)))  # wall thickness in cells
    grid[:t, :] = OCC
    grid[-t:, :] = OCC
    grid[:, :t] = OCC
    grid[:, -t:] = OCC

    for o in obstacles:
        fill_box(grid, origin, res, o['x'], o['y'], o['sx'], o['sy'])

    pgm = Path(f'{out_stem}.pgm')
    with open(pgm, 'wb') as fh:
        fh.write(f'P5\n{w} {h}\n255\n'.encode())
        fh.write(grid.tobytes())

    yaml_path = Path(f'{out_stem}.yaml')
    yaml_path.write_text(
        f'image: {pgm.name}\n'
        f'mode: trinary\n'
        f'resolution: {res}\n'
        f'origin: [{origin[0]}, {origin[1]}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.25\n')
    print(f'[generate_map] {w}x{h} cells @ {res} m -> {pgm.name}, {yaml_path.name} '
          f'(extent {room_x}x{room_y} m, origin {origin[:2]})')
    return pgm, yaml_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--room-x', type=float, default=7.0)
    ap.add_argument('--room-y', type=float, default=7.0)
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--wall', type=float, default=0.1, help='wall thickness [m]')
    ap.add_argument('--out', required=True, help='output stem (without extension)')
    ap.add_argument('--obstacle', action='append', default=[],
                    help="static box 'x,y,sx,sy' (repeatable)")
    args = ap.parse_args()

    obstacles = []
    for spec in args.obstacle:
        x, y, sx, sy = (float(v) for v in spec.split(','))
        obstacles.append({'x': x, 'y': y, 'sx': sx, 'sy': sy})

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    generate(args.room_x, args.room_y, args.resolution, args.wall, obstacles, args.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
