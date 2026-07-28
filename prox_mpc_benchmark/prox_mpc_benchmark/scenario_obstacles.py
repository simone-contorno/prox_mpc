# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Single source of truth for mapping scenario obstacles to parallel arrays.

Modes a/b1 (run_matrix.py, benchmark.launch.py) and mode b2 (run_nav2.py) must
drive byte-identical obstacle geometry or the comparison is not fair. This module
mirrors the C++ obstacle_field.hpp motion law; keep the two in sync.
"""


def obstacle_arrays(scn):
    """
    Map a scenario's obstacles to the parallel-array parameter form.

    Returns eight lists: motion, cx, cy, ex, ey, radius, speed, clearance.
    For circle motion (cx, cy) is the orbit centre and radius its radius; for
    line motion (cx, cy) and (ex, ey) are the patrol endpoints; for static
    obstacles (cx, cy) is the pose and radius is unused.
    """
    motion, cx, cy, ex, ey, radius, speed, clearance = ([] for _ in range(8))
    for o in scn.get('obstacles', []) or []:
        otype = o.get('type', 'static')
        m = 'static' if otype == 'static' else o.get('motion', 'static')
        motion.append(m)
        if m == 'circle':
            c = o.get('center', {})
            cx.append(float(c.get('x', 0.0)))
            cy.append(float(c.get('y', 0.0)))
            ex.append(0.0)
            ey.append(0.0)
            radius.append(float(o.get('radius', 1.0)))
            speed.append(float(o.get('speed', 0.0)))
        elif m == 'line':
            a = o.get('from', {})
            b = o.get('to', {})
            cx.append(float(a.get('x', 0.0)))
            cy.append(float(a.get('y', 0.0)))
            ex.append(float(b.get('x', 0.0)))
            ey.append(float(b.get('y', 0.0)))
            radius.append(1.0)
            speed.append(float(o.get('speed', 0.0)))
        else:  # static
            p = o.get('pose', {})
            cx.append(float(p.get('x', 0.0)))
            cy.append(float(p.get('y', 0.0)))
            ex.append(0.0)
            ey.append(0.0)
            radius.append(1.0)
            speed.append(0.0)
        clearance.append(float(o.get('clearance', 0.7)))
    return motion, cx, cy, ex, ey, radius, speed, clearance


def body_radii(clearance, robot_radius, safety_margin):
    """
    Physical body radius per obstacle, for costmap marking and the clearance metric.

    The scenario states the required clearance between robot and obstacle centres;
    the drawn/marked body is that minus the robot's own footprint allowance, with a
    floor so an obstacle never degenerates to a point.
    """
    return [max(0.05, c - (robot_radius + safety_margin)) for c in clearance]
