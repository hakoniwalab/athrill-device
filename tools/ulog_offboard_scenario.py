#!/usr/bin/env python3
"""Write a scenario for athrill,mavlink-scenario from a PX4 ULog flown in Offboard (position setpoints).

  python3 tools/ulog_offboard_scenario.py <flight.ulg> <scenario.csv> [--start-ms 50000] [--end-s T]

The log's trajectory_setpoint (position x, y, z and yaw, local NED) becomes the scenario channels x, y, z, yaw:
- Time: log time t (from the log start, which is the arming for SDLOG_MODE 0) is replayed at start-ms + t.
- Frame: positions are taken relative to the vehicle's position at the log start and turned by minus its heading
  then, so the replay starts at the origin heading north (the emulated vehicle's start); yaw is turned the same way.
- Rate: samples closer than --join-s (0.3 s) apart are joined by straight lines, re-sampled every --step-ms (50 ms);
  longer gaps hold the last value (PX4 logs the topic when it changes). NaN fields hold the last finite value.
- Sequence: setpoints stream from start-ms - 5 s at the first setpoint; Offboard (mode 6) at start-ms - 3 s; arm at
  start-ms; the log's setpoints until --end-s (default: the end of the log's first Offboard stretch); then AUTO_LAND
  (mode 4, submode 6).
Needs pyulog.
"""
import argparse
import math
import sys

import numpy as np
from pyulog import ULog


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog")
    ap.add_argument("scenario")
    ap.add_argument("--start-ms", type=float, default=50000.0)
    ap.add_argument("--end-s", type=float, default=None)
    ap.add_argument("--step-ms", type=float, default=50.0)
    ap.add_argument("--join-s", type=float, default=0.3)
    a = ap.parse_args()

    u = ULog(a.ulog, ["trajectory_setpoint", "vehicle_local_position", "vehicle_status"])
    get = lambda n: [d for d in u.data_list if d.name == n][0].data  # noqa: E731
    sp, lp, vs = get("trajectory_setpoint"), get("vehicle_local_position"), get("vehicle_status")
    t0 = u.start_timestamp
    if a.end_s is None:
        ns, tv = vs["nav_state"], (vs["timestamp"] - t0) / 1e6
        off = np.flatnonzero(ns != 14)
        off = off[tv[off] > 0.5]
        a.end_s = float(tv[off[0]]) if len(off) else float(tv[-1])
    ts = (sp["timestamp"] - t0) / 1e6
    keep = ts <= a.end_s
    ts = ts[keep]
    cols = []
    for key in ("position[0]", "position[1]", "position[2]", "yaw"):
        v = sp[key][keep].astype(float)
        last = np.nan
        for i in range(len(v)):          # hold the last finite value
            if np.isfinite(v[i]):
                last = v[i]
            else:
                v[i] = last
        cols.append(v)
    first = np.flatnonzero(np.all(np.isfinite(np.vstack(cols)), axis=0))
    if len(first) == 0:
        sys.exit("no finite position setpoint")
    i0 = first[0]
    ts, cols = ts[i0:], [c[i0:] for c in cols]
    x0, y0, z0, psi0 = float(lp["x"][0]), float(lp["y"][0]), float(lp["z"][0]), float(lp["heading"][0])
    c, s = math.cos(-psi0), math.sin(-psi0)
    dx, dy = cols[0] - x0, cols[1] - y0
    x, y, z = c * dx - s * dy, s * dx + c * dy, cols[2] - z0
    yaw = np.unwrap(cols[3] - psi0)

    step = a.step_ms / 1000.0
    grid = np.arange(0.0, a.end_s + 1e-9, step)
    out = np.empty((len(grid), 4))
    for k, tg in enumerate(grid):
        j = np.searchsorted(ts, tg, side="right") - 1
        if j < 0:
            j = 0
        if j + 1 < len(ts) and ts[j + 1] - ts[j] <= a.join_s and tg >= ts[j]:
            w = (tg - ts[j]) / (ts[j + 1] - ts[j])
            out[k] = [(1 - w) * v[j] + w * v[j + 1] for v in (x, y, z, yaw)]
        else:
            out[k] = [v[j] for v in (x, y, z, yaw)]
    out[:, 3] = [wrap(v) for v in out[:, 3]]

    S = a.start_ms
    with open(a.scenario, "w") as f:
        f.write(f"# from {a.ulog.split('/')[-1]}: trajectory_setpoint 0..{a.end_s:.1f} s, relative to the start "
                f"position ({x0:.3f}, {y0:.3f}, {z0:.3f}) and turned by -{psi0:.4f} rad\n")
        f.write("time_ms,x,y,z,yaw,arm,mode,submode\n")
        p = out[0]
        f.write(f"{S - 5000:.0f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},0,0,0\n")
        f.write(f"{S - 3000:.0f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},0,6,0\n")
        for tg, p in zip(grid, out):
            f.write(f"{S + tg * 1000:.0f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},1,6,0\n")
        p = out[-1]
        f.write(f"{S + a.end_s * 1000 + step * 1000:.0f},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},1,4,6\n")
    print(f"{a.scenario}: {len(grid)} setpoints, 0..{a.end_s:.1f} s of the log at {S / 1000:.1f} s; start heading "
          f"{psi0:.3f} rad removed")


if __name__ == "__main__":
    main()
