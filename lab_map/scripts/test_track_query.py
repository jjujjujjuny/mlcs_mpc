#!/usr/bin/env python3

import sys
from pathlib import Path

CODE_DIR = Path("/home/user/Desktop/turtlebot_data/code")
MAIN_PC_DIR = CODE_DIR / "main_pc"

for p in [CODE_DIR, MAIN_PC_DIR]:
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import argparse
from track_path_api import TrackPath


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--v", type=float, default=0.12)
    parser.add_argument("--lookahead", type=float, default=0.60)
    args = parser.parse_args()

    track = TrackPath()

    print("===== Track summary =====")
    for k, v in track.summary().items():
        print(f"{k}: {v}")

    print("")
    print("===== Query pose =====")
    print(f"x={args.x}, y={args.y}, yaw={args.yaw}")

    near = track.nearest(args.x, args.y, args.yaw)

    print("")
    print("===== Nearest centerline =====")
    for k in [
        "x_ref",
        "y_ref",
        "s",
        "theta",
        "idx",
        "distance",
        "lateral_error",
        "yaw_ref",
        "heading_error",
        "kappa",
        "dkappa_ds",
    ]:
        if k in near:
            print(f"{k}: {near[k]}")

    pp = track.pure_pursuit(
        x=args.x,
        y=args.y,
        yaw=args.yaw,
        v=args.v,
        lookahead=args.lookahead,
    )

    print("")
    print("===== Pure pursuit =====")
    print("target_x:", pp["target_x"])
    print("target_y:", pp["target_y"])
    print("target_s:", pp["target_s"])
    print("target_theta:", pp["target_theta"])
    print("target_kappa:", pp["target_kappa"])
    print("alpha:", pp["alpha"])
    print("kappa_cmd:", pp["kappa_cmd"])
    print("omega:", pp["omega"])


if __name__ == "__main__":
    main()
