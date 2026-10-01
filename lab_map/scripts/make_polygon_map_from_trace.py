#!/usr/bin/env python3

import os
import glob
import json
import time
import argparse

import numpy as np
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


def moving_average(arr, window):
    if window <= 1:
        return arr.copy()
    kernel = np.ones(window) / window
    pad = window // 2
    arr_pad = np.pad(arr, (pad, pad), mode="edge")
    out = np.convolve(arr_pad, kernel, mode="valid")
    return out[:len(arr)]


def remove_near_duplicates(points, min_dist=0.03):
    if len(points) == 0:
        return points
    kept = [points[0]]
    for p in points[1:]:
        if np.linalg.norm(p - kept[-1]) >= min_dist:
            kept.append(p)
    return np.array(kept)


def point_line_distance(point, start, end):
    if np.allclose(start, end):
        return np.linalg.norm(point - start)
    line = end - start
    t = np.dot(point - start, line) / np.dot(line, line)
    proj = start + t * line
    return np.linalg.norm(point - proj)


def rdp(points, epsilon):
    if len(points) < 3:
        return points

    start = points[0]
    end = points[-1]

    dmax = -1.0
    index = -1
    for i in range(1, len(points) - 1):
        d = point_line_distance(points[i], start, end)
        if d > dmax:
            dmax = d
            index = i

    if dmax > epsilon:
        left = rdp(points[:index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return np.vstack((left[:-1], right))
    else:
        return np.array([start, end])


def close_ring(points):
    if len(points) < 2:
        return points
    if np.linalg.norm(points[0] - points[-1]) < 1e-9:
        return points
    return np.vstack([points, points[0]])


def polygon_area(points_closed):
    x = points_closed[:, 0]
    y = points_closed[:, 1]
    return 0.5 * abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))


def polygon_perimeter(points_closed):
    return np.sum(np.linalg.norm(points_closed[1:] - points_closed[:-1], axis=1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default=None)
    parser.add_argument("--use_rot180", action="store_true")
    parser.add_argument("--stride", type=int, default=10)
    parser.add_argument("--smooth_window", type=int, default=9)
    parser.add_argument("--min_dist", type=float, default=0.05)
    parser.add_argument("--epsilon", type=float, default=0.20)
    parser.add_argument("--trim_head", type=int, default=0)
    parser.add_argument("--trim_tail", type=int, default=0)
    args = parser.parse_args()

    csv_path = args.csv if args.csv else latest_manual_csv()
    df = pd.read_csv(csv_path)

    if "model_x" not in df.columns or "model_y" not in df.columns:
        raise RuntimeError("CSV에 model_x, model_y 컬럼이 없어.")

    raw_x = df["model_x"].to_numpy(dtype=float)
    raw_y = df["model_y"].to_numpy(dtype=float)

    if args.use_rot180:
        raw_x = -raw_x
        raw_y = -raw_y

    if args.trim_head > 0:
        raw_x = raw_x[args.trim_head:]
        raw_y = raw_y[args.trim_head:]

    if args.trim_tail > 0:
        raw_x = raw_x[:-args.trim_tail]
        raw_y = raw_y[:-args.trim_tail]

    trace_points = np.column_stack([raw_x, raw_y])

    x = raw_x.copy()
    y = raw_y.copy()

    if args.stride > 1:
        x = x[::args.stride]
        y = y[::args.stride]

    x = moving_average(x, args.smooth_window)
    y = moving_average(y, args.smooth_window)

    processed_points = np.column_stack([x, y])
    filtered_points = remove_near_duplicates(processed_points, min_dist=args.min_dist)

    if len(filtered_points) < 3:
        raise RuntimeError("유효 점이 너무 적어 polygon을 만들 수 없어.")

    simplified = rdp(filtered_points, args.epsilon)

    if len(simplified) < 3:
        raise RuntimeError("단순화 후 점이 너무 적어 polygon을 만들 수 없어. epsilon을 줄여봐.")

    polygon_closed = close_ring(simplified)

    area = polygon_area(polygon_closed)
    perimeter = polygon_perimeter(polygon_closed)

    ts = int(time.time())
    out_dir = os.path.join(MAP_ROOT, f"polygon_map_{ts}")
    os.makedirs(out_dir, exist_ok=True)

    vertices_csv = os.path.join(out_dir, "polygon_vertices.csv")
    json_path = os.path.join(out_dir, "polygon_map.json")
    preview_png = os.path.join(out_dir, "polygon_preview.png")
    compare_png = os.path.join(out_dir, "polygon_compare.png")

    latest_preview_png = os.path.join(MAP_ROOT, "latest_polygon_preview.png")
    latest_compare_png = os.path.join(MAP_ROOT, "latest_polygon_compare.png")

    vertices_df = pd.DataFrame({
        "vertex_id": list(range(len(simplified))),
        "x": simplified[:, 0],
        "y": simplified[:, 1],
    })
    vertices_df.to_csv(vertices_csv, index=False)

    json_data = {
        "source_csv": csv_path,
        "use_rot180": args.use_rot180,
        "parameters": {
            "stride": args.stride,
            "smooth_window": args.smooth_window,
            "min_dist": args.min_dist,
            "epsilon": args.epsilon,
            "trim_head": args.trim_head,
            "trim_tail": args.trim_tail,
        },
        "polygon": {
            "num_vertices": int(len(simplified)),
            "area_m2": float(area),
            "perimeter_m": float(perimeter),
            "vertices": [
                {"id": int(i), "x": float(p[0]), "y": float(p[1])}
                for i, p in enumerate(simplified)
            ]
        }
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(json_data, f, ensure_ascii=False, indent=2)

    # -------------------------
    # 1) 기본 preview
    # -------------------------
    plt.figure(figsize=(9, 9))
    plt.plot(trace_points[:, 0], trace_points[:, 1], linewidth=1.0, alpha=0.35, label="raw trace")
    plt.plot(filtered_points[:, 0], filtered_points[:, 1], linewidth=1.2, alpha=0.8, label="filtered trace")
    plt.plot(polygon_closed[:, 0], polygon_closed[:, 1], "-o", linewidth=2.5, markersize=5, label="optimized polygon")

    plt.scatter([trace_points[0, 0]], [trace_points[0, 1]], s=80, label="start")
    plt.scatter([trace_points[-1, 0]], [trace_points[-1, 1]], s=80, label="end")

    for i, p in enumerate(simplified):
        plt.text(p[0], p[1], str(i), fontsize=9)

    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Polygon map from manual boundary trace")
    plt.legend()
    plt.savefig(preview_png, dpi=200, bbox_inches="tight")
    plt.savefig(latest_preview_png, dpi=200, bbox_inches="tight")
    plt.close()

    # -------------------------
    # 2) 비교용 figure
    # -------------------------
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    # 왼쪽: 실제 trace만
    axes[0].plot(trace_points[:, 0], trace_points[:, 1], linewidth=1.0, alpha=0.9, label="actual trace")
    axes[0].scatter([trace_points[0, 0]], [trace_points[0, 1]], s=80, label="start")
    axes[0].scatter([trace_points[-1, 0]], [trace_points[-1, 1]], s=80, label="end")
    axes[0].set_title("Actual measured trace")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend()

    # 오른쪽: 실제 trace + polygon overlay
    axes[1].plot(trace_points[:, 0], trace_points[:, 1], linewidth=1.0, alpha=0.35, label="actual trace")
    axes[1].plot(filtered_points[:, 0], filtered_points[:, 1], linewidth=1.0, alpha=0.8, label="filtered trace")
    axes[1].plot(polygon_closed[:, 0], polygon_closed[:, 1], "-o", linewidth=2.8, markersize=5, label="optimized polygon")
    axes[1].scatter([trace_points[0, 0]], [trace_points[0, 1]], s=80, label="start")
    axes[1].scatter([trace_points[-1, 0]], [trace_points[-1, 1]], s=80, label="end")

    for i, p in enumerate(simplified):
        axes[1].text(p[0], p[1], str(i), fontsize=9)

    info_text = (
        f"vertices: {len(simplified)}\n"
        f"area: {area:.3f} m^2\n"
        f"perimeter: {perimeter:.3f} m\n"
        f"rot180: {args.use_rot180}\n"
        f"stride: {args.stride}, smooth: {args.smooth_window}\n"
        f"min_dist: {args.min_dist}, epsilon: {args.epsilon}"
    )
    axes[1].text(
        0.02, 0.98, info_text,
        transform=axes[1].transAxes,
        verticalalignment="top",
        bbox=dict(boxstyle="round", alpha=0.2)
    )

    axes[1].set_title("Actual trace vs optimized polygon")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    axes[1].axis("equal")
    axes[1].grid(True)
    axes[1].legend()

    plt.tight_layout()
    plt.savefig(compare_png, dpi=200, bbox_inches="tight")
    plt.savefig(latest_compare_png, dpi=200, bbox_inches="tight")
    plt.close()

    print("source csv:", csv_path)
    print("out dir:", out_dir)
    print("vertices csv:", vertices_csv)
    print("json:", json_path)
    print("preview:", preview_png)
    print("compare:", compare_png)
    print("latest preview:", latest_preview_png)
    print("latest compare:", latest_compare_png)
    print("")
    print("num_vertices:", len(simplified))
    print("area_m2:", area)
    print("perimeter_m:", perimeter)


if __name__ == "__main__":
    main()
