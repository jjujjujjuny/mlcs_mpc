#!/usr/bin/env python3

import os
import glob
import argparse
import pandas as pd
import matplotlib.pyplot as plt


MANUAL_ROOT = "/home/user/Desktop/turtlebot_data/experiments/lab_boundary_manual"
MAP_ROOT = "/home/user/Desktop/map_data"


def latest_manual_csv():
    files = glob.glob(os.path.join(MANUAL_ROOT, "session_*", "manual_lab_boundary_motion.csv"))

    if not files:
        raise FileNotFoundError("manual_lab_boundary_motion.csv 파일을 찾지 못했어.")

    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default=None)
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args()

    csv_path = args.csv if args.csv is not None else latest_manual_csv()

    df = pd.read_csv(csv_path)

    if "model_x" not in df.columns or "model_y" not in df.columns:
        raise RuntimeError("CSV에 model_x, model_y 컬럼이 없어.")

    d = df.iloc[::args.stride].copy()

    x = d["model_x"]
    y = d["model_y"]

    x_min = x.min()
    x_max = x.max()
    y_min = y.min()
    y_max = y.max()

    width_x = x_max - x_min
    height_y = y_max - y_min

    session_dir = os.path.dirname(csv_path)
    out_png = os.path.join(session_dir, "manual_boundary_trace.png")
    out_csv = os.path.join(session_dir, "manual_boundary_trace_points.csv")

    os.makedirs(MAP_ROOT, exist_ok=True)
    map_preview_png = os.path.join(MAP_ROOT, "latest_manual_boundary_trace.png")

    d[["time", "model_x", "model_y", "model_speed"]].to_csv(out_csv, index=False)

    plt.figure(figsize=(8, 8))
    plt.plot(x, y, linewidth=1.0, label="manual trace")
    plt.scatter([x.iloc[0]], [y.iloc[0]], s=60, label="start")
    plt.scatter([x.iloc[-1]], [y.iloc[-1]], s=60, label="end")

    plt.xlabel("model_x [m]")
    plt.ylabel("model_y [m]")
    plt.title("Manual Boundary Trace: all visited model-center positions")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    margin = 0.2
    plt.xlim(x_min - margin, x_max + margin)
    plt.ylim(y_min - margin, y_max + margin)

    plt.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.savefig(map_preview_png, dpi=200, bbox_inches="tight")

    print("source csv:", csv_path)
    print("rows:", len(df))
    print("plotted rows:", len(d))
    print("")
    print("bounds from all visited points:")
    print("  x_min:", x_min)
    print("  x_max:", x_max)
    print("  y_min:", y_min)
    print("  y_max:", y_max)
    print("  width_x:", width_x)
    print("  height_y:", height_y)
    print("")
    print("saved plot:", out_png)
    print("saved latest preview:", map_preview_png)
    print("saved points:", out_csv)


if __name__ == "__main__":
    main()
