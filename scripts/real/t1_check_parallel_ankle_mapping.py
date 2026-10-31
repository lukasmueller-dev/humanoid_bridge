#!/usr/bin/env python3
"""Check which crank order / sign makes the bridge's parallel ankle mapping agree with the Booster firmware.

Read-only: it only subscribes to /low_state and never commands the robot. Run it with the robot in damping or
prepare mode (or hanging) while you move both feet by hand through pitch and roll, then press Ctrl+C.

/low_state carries both motor_state_parallel (crank motors in slots 15/16 and 21/22) and motor_state_serial (the
firmware's ankle pitch/roll). For every hypothesis (crank b in the pitch slot or not, sign of each crank) the script
computes f^-1(cranks) with t1_parallel_ankle.py and compares it with the firmware's serial angles. The best
hypothesis gives parallel_ankle_swap_cranks / parallel_ankle_crank_sign for robot_bridge/params/T1_config.yaml;
its error should be small (a few mrad) over the whole motion before parallel_ankle_mapping is turned on.

    python3 scripts/real/t1_check_parallel_ankle_mapping.py [--seconds 30] [--save samples.npz]
"""

import argparse
import itertools
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import t1_parallel_ankle as pa

SERIAL_IDX = {"left": (15, 16), "right": (21, 22)}  # (pitch, roll) = the two crank slots in parallel mode
HYPOTHESES = [dict(swap=swap, sign=(sa, sb)) for swap, sa, sb in itertools.product((False, True), (1.0, -1.0), (1.0, -1.0))]


def crank_angles_from_slots(parallel_q, side, swap, sign):
    """XML-convention crank angles (a, b) from the two motor_state_parallel slots of one ankle."""
    i_pitch, i_roll = SERIAL_IDX[side]
    slot_a, slot_b = (i_roll, i_pitch) if swap else (i_pitch, i_roll)
    return np.array([sign[0] * parallel_q[slot_a], sign[1] * parallel_q[slot_b]])


def score_hypotheses(parallel_q, serial_q):
    """parallel_q, serial_q: (N, 23). Returns one dict per hypothesis with per-side max / median error [rad]."""
    results = []
    for hyp in HYPOTHESES:
        errors = {}
        for side in ("left", "right"):
            err = []
            for qp, qs in zip(parallel_q, serial_q):
                q_fw = qs[list(SERIAL_IDX[side])]
                q_est, residual = pa.serial_angles(crank_angles_from_slots(qp, side, hyp["swap"], hyp["sign"]), side, q_fw)
                err.append(np.abs(q_est - q_fw).max() if residual < 1e-6 else np.inf)
            err = np.asarray(err)
            errors[side] = dict(max=float(err.max()), median=float(np.median(err)))
        results.append(dict(hyp, errors=errors, worst=max(e["max"] for e in errors.values())))
    return sorted(results, key=lambda r: r["worst"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=30.0, help="Stop after this long (Ctrl+C also stops).")
    parser.add_argument("--rate", type=float, default=50.0, help="Samples per second kept from /low_state.")
    parser.add_argument("--save", type=Path, help="Save the raw samples to this .npz.")
    parser.add_argument("--load", type=Path, help="Score samples saved earlier with --save instead of recording.")
    args = parser.parse_args(argv)

    if args.load:
        data = np.load(args.load)
        parallel_q, serial_q = data["parallel_q"], data["serial_q"]
    else:
        import rclpy
        from booster_interface.msg import LowState

        rclpy.init()
        node = rclpy.create_node("t1_check_parallel_ankle_mapping")
        samples = dict(parallel=[], serial=[], last=0.0)

        def on_state(msg):
            now = node.get_clock().now().nanoseconds * 1e-9
            if now - samples["last"] < 1.0 / args.rate or len(msg.motor_state_parallel) < 23:
                return
            samples["last"] = now
            samples["parallel"].append([m.q for m in msg.motor_state_parallel])
            samples["serial"].append([m.q for m in msg.motor_state_serial])

        node.create_subscription(LowState, "/low_state", on_state, 1)
        print(f"Recording /low_state for {args.seconds:.0f} s: move both feet through pitch and roll (Ctrl+C to stop).")
        end = node.get_clock().now().nanoseconds * 1e-9 + args.seconds
        try:
            while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        parallel_q, serial_q = np.asarray(samples["parallel"]), np.asarray(samples["serial"])
        if args.save:
            np.savez(args.save, parallel_q=parallel_q, serial_q=serial_q)
            print(f"saved {args.save}")

    if len(parallel_q) == 0:
        raise SystemExit("no /low_state samples with motor_state_parallel received")
    for side in ("left", "right"):
        q = serial_q[:, list(SERIAL_IDX[side])]
        print(f"{side} ankle range covered: pitch [{q[:, 0].min():+.2f}, {q[:, 0].max():+.2f}], "
              f"roll [{q[:, 1].min():+.2f}, {q[:, 1].max():+.2f}] rad")

    results = score_hypotheses(parallel_q, serial_q)
    print(f"\n{len(parallel_q)} samples. |f^-1(cranks) - firmware serial| per hypothesis (max / median, rad):")
    for r in results:
        e = r["errors"]
        print(f"  swap_cranks={str(r['swap']).lower():5s} crank_sign=[{r['sign'][0]:+.1f}, {r['sign'][1]:+.1f}]  "
              f"left {e['left']['max']:.4f} / {e['left']['median']:.4f}   right {e['right']['max']:.4f} / {e['right']['median']:.4f}")
    best = results[0]
    print("\nBest: parallel_ankle_swap_cranks: {}  parallel_ankle_crank_sign: [{:.1f}, {:.1f}]  (max error {:.4f} rad)"
          .format(str(best["swap"]).lower(), *best["sign"], best["worst"]))
    if best["worst"] > 0.02:
        print("WARNING: even the best hypothesis is off by more than 0.02 rad; the crank zero offsets or the geometry "
              "do not match T1_23dof_parallel.xml. Do not enable parallel_ankle_mapping.")


if __name__ == "__main__":
    main()
