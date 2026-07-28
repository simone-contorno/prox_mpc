#!/usr/bin/env python3
# Copyright 2026 Simone Contorno
# SPDX-License-Identifier: Apache-2.0

"""
Sample a controller process's CPU / RAM and the achieved control rate.

For the mode (b2) cross-controller comparison this measures the embedded-relevant
resource cost of one Nav2 controller: the CPU% and resident memory (RSS) of its
`controller_server` process (read from /proc), and the achieved control frequency
(the rate of commands on /cmd_vel while the robot is moving). Every controller runs
in its own controller_server with identical surrounding config, so the differences
are attributable to the controller plugin.

CPU% is reported as a fraction of one core, so a multi-threaded controller (e.g.
MPPI parallelising its batch) can exceed 100%. Stats are accumulated only while the
robot is commanded (from the first /cmd_vel to shutdown), excluding idle bring-up.
The per-run JSON is merged into the run record by run_nav2.py.
"""

import argparse
import json
import math
import os
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

CLK_TCK = os.sysconf('SC_CLK_TCK')


def percentile(values, q):
    """Linear-interpolated q-quantile (0..1) of values, or None if empty."""
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    idx = q * (len(s) - 1)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (idx - lo)


def read_cpu_rss(pid):
    """Return (cpu_jiffies, rss_kb) for pid, or (None, None) if it is gone."""
    try:
        with open(f'/proc/{pid}/stat') as fh:
            stat = fh.read()
        # The comm field is parenthesised and may contain spaces; everything after
        # the last ')' is the space-separated remainder starting at field 3.
        rest = stat[stat.rfind(')') + 2:].split()
        jiffies = int(rest[11]) + int(rest[12])  # utime + stime
        rss_kb = 0
        with open(f'/proc/{pid}/status') as fh:
            for line in fh:
                if line.startswith('VmRSS:'):
                    rss_kb = int(line.split()[1])
                    break
        return jiffies, rss_kb
    except (FileNotFoundError, ProcessLookupError, IndexError, ValueError):
        return None, None


class ResourceSampler(Node):
    """Poll a PID's CPU/RSS and count /cmd_vel messages until shutdown."""

    def __init__(self, pid, out_path, hz, cmd_topic, compute_topic,
                 node_name='prox_mpc_resource_sampler'):
        super().__init__(node_name)
        self._pid = pid
        self._out = out_path
        self._cmd_count = 0
        self._first_cmd = None
        self._last_cmd = None
        self._prev_jiffies = None
        self._prev_t = None
        self._cpu_jiffies = 0
        self._cpu_wall = 0.0
        self._cpu_peak = 0.0
        self._rss_peak_kb = 0
        self._rss_sum_kb = 0
        self._rss_n = 0
        self._compute_ms = []
        self._compute_dropped = 0
        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        if compute_topic:
            self.create_subscription(Float64, compute_topic, self._on_compute, 50)
        self.create_timer(1.0 / hz, self._sample)

    def _on_compute(self, msg):
        # Per-cycle controller compute time [ms] from the timing decorator. A
        # non-finite sample sorts into an arbitrary position and would corrupt
        # the reported percentiles, so it is rejected here rather than inside
        # percentile().
        if self._first_cmd is None:
            return
        if math.isfinite(msg.data):
            self._compute_ms.append(msg.data)
        else:
            self._compute_dropped += 1

    def _on_cmd(self, _msg):
        now = time.monotonic()
        if self._first_cmd is None:
            self._first_cmd = now
        self._last_cmd = now
        self._cmd_count += 1

    def _sample(self):
        jiffies, rss_kb = read_cpu_rss(self._pid)
        if jiffies is None:
            return
        now = time.monotonic()
        active = self._first_cmd is not None
        if self._prev_jiffies is not None and active:
            dt = now - self._prev_t
            if dt > 0.0:
                dj = jiffies - self._prev_jiffies
                self._cpu_jiffies += dj
                self._cpu_wall += dt
                self._cpu_peak = max(self._cpu_peak, (dj / CLK_TCK) / dt * 100.0)
            self._rss_peak_kb = max(self._rss_peak_kb, rss_kb)
            self._rss_sum_kb += rss_kb
            self._rss_n += 1
        self._prev_jiffies = jiffies
        self._prev_t = now

    def write(self):
        """Write the accumulated resource summary as JSON."""
        if self._compute_dropped:
            self.get_logger().warn(
                f'{self._compute_dropped} non-finite compute-time sample(s) dropped; '
                'excluded from compute_ms_p50/p95/max')
        cpu_mean = ((self._cpu_jiffies / CLK_TCK) / self._cpu_wall * 100.0
                    if self._cpu_wall > 0.0 else None)
        freq = None
        if (self._first_cmd is not None and self._last_cmd is not None and
                self._last_cmd > self._first_cmd and self._cmd_count > 1):
            freq = (self._cmd_count - 1) / (self._last_cmd - self._first_cmd)
        rec = {
            'cpu_mean_pct': cpu_mean,
            'cpu_peak_pct': self._cpu_peak if self._cpu_wall > 0.0 else None,
            'rss_peak_mb': self._rss_peak_kb / 1024.0 if self._rss_peak_kb else None,
            'rss_mean_mb': (self._rss_sum_kb / self._rss_n / 1024.0
                            if self._rss_n else None),
            'control_rate_hz': freq,
            'cmd_count': self._cmd_count,
            'compute_ms_p50': percentile(self._compute_ms, 0.50),
            'compute_ms_p95': percentile(self._compute_ms, 0.95),
            'compute_ms_max': max(self._compute_ms) if self._compute_ms else None,
            'compute_count': len(self._compute_ms),
            'sampled_pid': self._pid,
        }
        with open(self._out, 'w') as fh:
            json.dump(rec, fh, indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pid', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--cmd-topic', default='cmd_vel')
    ap.add_argument('--compute-topic', default='')
    ap.add_argument('--node-name', default='prox_mpc_resource_sampler')
    args = ap.parse_args()

    rclpy.init()
    node = ResourceSampler(args.pid, args.out, args.hz, args.cmd_topic,
                           args.compute_topic, args.node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # a tool process: still write what we have
        pass
    finally:
        node.write()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
