#!/usr/bin/env python3
"""Plot a wing CSV log — the graphical side of docs/test-campaign.md.

    python3 plot.py bench_T0.csv --out bench_T0.png

Panels (top to bottom):
  1. pilot sticks (rc_p/rc_r/rc_t) + mode/arm steps   — T0: mix sanity, expo
  2. surface commands l_us/r_us + throttle out        — T0: D1 signs, endpoints
  3. attitude roll/pitch/yaw + env flags (stall/bank…) — envelope behaviour
  4. speeds gps_v vs vest, altitude on right axis     — T2: stall campaign
  5. accel az [g] + gyro pitch rate                   — buffet / stall signature

The stall event detector (T2) uses panel 4+5: speed minimum, then nose-drop
(pitch rate) and sink. See the doc for the exact thresholds.
"""
import argparse
import csv
import math
import sys


def load(path):
    """Return {column: [floats]} keeping only rows with a valid t_ms."""
    cols = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["t_ms"])
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isfinite(t):
                continue
            for k, v in row.items():
                if k is None:
                    continue
                try:
                    x = float(v)
                except (TypeError, ValueError):
                    x = float("nan")
                cols.setdefault(k, []).append(x)
            n = len(cols["t_ms"])
            for k in cols:                       # ragged rows -> pad
                if len(cols[k]) < n:
                    cols[k].append(float("nan"))
    if "t_ms" not in cols or not cols["t_ms"]:
        sys.exit(f"{path}: no valid t_ms column — is this a logger CSV?")
    t0 = cols["t_ms"][0]
    cols["t_s"] = [(x - t0) / 1000.0 for x in cols["t_ms"]]
    return cols


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("csv_file")
    p.add_argument("--out", help="write PNG instead of opening a window")
    args = p.parse_args()

    try:
        import matplotlib
        if args.out:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib missing: pip3 install matplotlib")

    c = load(args.csv_file)
    t = c["t_s"]

    def get(name, default=0.0):
        return c.get(name, [default] * len(t))

    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 11))
    fig.suptitle(args.csv_file, fontsize=12)

    # 1 — sticks + modes
    a = axes[0]
    a.plot(t, get("rc_p"), label="rc pitch")
    a.plot(t, get("rc_r"), label="rc roll")
    a.plot(t, get("rc_t"), label="rc throttle")
    a.step(t, get("mode"), where="post", alpha=0.6, label="mode (1=passthru)")
    a.step(t, get("armed"), where="post", alpha=0.6, label="armed")
    a.set_ylabel("sticks [-1,1]")
    a.legend(loc="upper right", fontsize=8, ncol=3)

    # 2 — surfaces
    a = axes[1]
    a.plot(t, get("l_us"), label="elevon L [us]")
    a.plot(t, get("r_us"), label="elevon R [us]")
    a.set_ylabel("servos [us]")
    ax2 = a.twinx()
    ax2.plot(t, get("thr_out"), color="k", alpha=0.5, label="throttle")
    ax2.set_ylabel("throttle [0,1]")
    a.legend(loc="upper left", fontsize=8)
    ax2.legend(loc="upper right", fontsize=8)

    # 3 — attitude + env flags
    a = axes[2]
    a.plot(t, get("roll"), label="roll [deg]")
    a.plot(t, get("pitch"), label="pitch [deg]")
    a.plot(t, get("yaw"), label="yaw [deg]")
    env = get("env")
    a.step(t, [v * 10 for v in env], where="post", color="red",
           alpha=0.5, label="env x10 (1=stall 2=bank 4=vne 8=g)")
    a.set_ylabel("attitude [deg]")
    a.legend(loc="upper right", fontsize=8, ncol=3)

    # 4 — speeds + altitude
    a = axes[3]
    a.plot(t, get("gps_v"), label="GPS ground speed [m/s]")
    a.plot(t, get("vest"), label="airspeed est [m/s]")
    a.plot(t, get("rc_t"), alpha=0.3, label="throttle (scaled)")
    a.set_ylabel("speed [m/s]")
    ax4 = a.twinx()
    ax4.plot(t, get("gps_alt"), color="gray", alpha=0.6, label="GPS alt [m]")
    ax4.set_ylabel("altitude [m]")
    a.legend(loc="upper left", fontsize=8)
    ax4.legend(loc="upper right", fontsize=8)

    # 5 — IMU signature (stall buffet lives here)
    a = axes[4]
    a.plot(t, get("az"), label="accel z [g]")
    a.plot(t, get("gy"), label="gyro pitch [rad/s]")
    a.plot(t, get("gx"), alpha=0.5, label="gyro roll [rad/s]")
    a.set_ylabel("IMU")
    a.set_xlabel("time [s]")
    a.legend(loc="upper right", fontsize=8, ncol=3)

    # wind class (F6) — printed once so old logs (0=unknown) are honest
    winds = sorted({int(v) for v in get("wind")})
    if winds and winds != [0]:
        names = {0: "unknown", 1: "calm", 2: "soft", 3: "strong"}
        print("wind classes present: "
              + ", ".join(names.get(w, str(w)) for w in winds))

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if args.out:
        fig.savefig(args.out, dpi=110)
        print(f"wrote {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
