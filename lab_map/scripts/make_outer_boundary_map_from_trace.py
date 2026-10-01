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


def moving_average_closed(points, window):
    if window <= 1:
        return points.copy()

    n = len(points)
    out = []

    half = window // 2

    for i in range(n):
        idxs = [(i + k) % n for k in range(-half, half + 1)]
        out.append(np.mean(points[idxs], axis=0))

    return np.array(out)


def point_line_distance(point, start, end):
    if np.allclose(start, end):
        return np.linalg.norm(point - start)

    line = end - start
    t = np.dot(point - start, line) / np.dot(line, line)
    proj = start + t * line

    return np.linalg.norm(point - proj)


def rdp_closed(points, epsilon):
    # closed polygon을 위해 마지막에 첫 점을 붙여서 RDP 적용
    closed = np.vstack([points, points[0]])
    simplified = rdp_open(closed, epsilon)

    if np.linalg.norm(simplified[0] - simplified[-1]) < 1e-9:
        simplified = simplified[:-1]

    return simplified


def rdp_open(points, epsilon):
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
        left = rdp_open(points[:index + 1], epsilon)
        right = rdp_open(points[index:], epsilon)
        return np.vstack([left[:-1], right])
    else:
        return np.array([start, end])


def close_ring(points):
    return np.vstack([points, points[0]])


def polygon_area(points):
    p = close_ring(points)
    x = p[:, 0]
    y = p[:, 1]
    return 0.5 * abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))


def polygon_perimeter(points):
    p = close_ring(points)
    return np.sum(np.linalg.norm(p[1:] - p[:-1], axis=1))


def make_radial_outer_boundary(points, n_bins=180, percentile=95.0):
    """
    중심 기준 각도 bin마다 바깥쪽 점을 하나 선택한다.
    percentile=100이면 가장 먼 점.
    percentile=95이면 약간 outlier에 덜 민감.
    """

    center = np.mean(points, axis=0)
    rel = points - center

    angles = np.arctan2(rel[:, 1], rel[:, 0])
    angles = (angles + 2.0 * np.pi) % (2.0 * np.pi)

    radius = np.linalg.norm(rel, axis=1)

    bins = np.linspace(0.0, 2.0 * np.pi, n_bins + 1)

    selected = []

    for i in range(n_bins):
        lo = bins[i]
        hi = bins[i + 1]

        mask = (angles >= lo) & (angles < hi)

        if not np.any(mask):
            continue

        idxs = np.where(mask)[0]
        r = radius[idxs]

        if len(r) == 1:
            chosen = idxs[0]
        else:
            target_r = np.percentile(r, percentile)
            chosen = idxs[np.argmin(np.abs(r - target_r))]

        selected.append(points[chosen])

    selected = np.array(selected)

    # 같은 점 중복 제거
    unique = []
    for p in selected:
        if len(unique) == 0:
            unique.append(p)
        else:
            if np.linalg.norm(p - unique[-1]) > 1e-6:
                unique.append(p)

    if len(unique) > 1 and np.linalg.norm(unique[0] - unique[-1]) < 1e-6:
        unique = unique[:-1]

    return np.array(unique), center


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default=None)
    parser.add_argument("--use_rot180", action="store_true")

    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--n_bins", type=int, default=180)
    parser.add_argument("--percentile", type=float, default=97.0)
    parser.add_argument("--smooth_window", type=int, default=5)
    parser.add_argument("--epsilon", type=float, default=0.12)

    parser.add_argument("--trim_head", type=int, default=0)
    parser.add_argument("--trim_tail", type=int, default=0)

    args = parser.parse_args()

    csv_path = args.csv if args.csv else latest_manual_csv()
    df = pd.read_csv(csv_path)

    x = df["model_x"].to_numpy(dtype=float)
    y = df["model_y"].to_numpy(dtype=float)

    if args.use_rot180:
        x = -x
        y = -y

    if args.trim_head > 0:
        x = x[args.trim_head:]
        y = y[args.trim_head:]

    if args.trim_tail > 0:
        x = x[:-args.trim_tail]
        y = y[:-args.trim_tail]

    if args.stride > 1:
        x = x[::args.stride]
        y = y[::args.stride]

    points = np.column_stack([x, y])

    boundary, center = make_radial_outer_boundary(
        points,
        n_bins=args.n_bins,
        percentile=args.percentile,
    )

    if len(boundary) < 3:
        raise RuntimeError("boundary 후보점이 너무 적어.")

    boundary_smooth = moving_average_closed(boundary, args.smooth_window)

    polygon = rdp_closed(boundary_smooth, args.epsilon)

    if len(polygon) < 3:
        raise RuntimeError("polygon 꼭짓점이 너무 적어. epsilon을 줄여봐.")

    polygon_closed = close_ring(polygon)

    area = polygon_area(polygon)
    perimeter = polygon_perimeter(polygon)

    ts = int(time.time())
    out_dir = os.path.join(MAP_ROOT, f"outer_boundary_map_{ts}")
    os.makedirs(out_dir, exist_ok=True)

    json_path = os.path.join(out_dir, "outer_boundary_map.json")
    vertices_csv = os.path.join(out_dir, "outer_boundary_vertices.csv")
    preview_png = os.path.join(out_dir, "outer_boundary_preview.png")
    compare_png = os.path.join(out_dir, "outer_boundary_compare.png")

    latest_preview = os.path.join(MAP_ROOT, "latest_outer_boundary_preview.png")
    latest_compare = os.path.join(MAP_ROOT, "latest_outer_boundary_compare.png")

    pd.DataFrame({
        "vertex_id": list(range(len(polygon))),
        "x": polygon[:, 0],
        "y": polygon[:, 1],
    }).to_csv(vertices_csv, index=False)

    result = {
        "source_csv": csv_path,
        "method": "radial_outer_boundary_from_trace",
        "use_rot180": args.use_rot180,
        "parameters": {
            "stride": args.stride,
            "n_bins": args.n_bins,
            "percentile": args.percentile,
            "smooth_window": args.smooth_window,
            "epsilon": args.epsilon,
            "trim_head": args.trim_head,
            "trim_tail": args.trim_tail,
        },
        "center_used_for_radial_boundary": {
            "x": float(center[0]),
            "y": float(center[1]),
        },
        "polygon": {
            "num_vertices": int(len(polygon)),
            "area_m2": float(area),
            "perimeter_m": float(perimeter),
            "vertices": [
                {
                    "id": int(i),
                    "x": float(p[0]),
                    "y": float(p[1]),
                }
                for i, p in enumerate(polygon)
            ],
        },
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    # preview
    plt.figure(figsize=(9, 9))
    plt.plot(points[:, 0], points[:, 1], linewidth=0.8, alpha=0.35, label="actual trace")
    plt.plot(boundary[:, 0], boundary[:, 1], ".", markersize=3, alpha=0.7, label="outer selected points")
    plt.plot(polygon_closed[:, 0], polygon_closed[:, 1], "-o", linewidth=2.5, markersize=5, label="outer boundary polygon")

    for i, p in enumerate(polygon):
        plt.text(p[0], p[1], str(i), fontsize=9)

    plt.scatter([points[0, 0]], [points[0, 1]], s=80, label="start")
    plt.scatter([points[-1, 0]], [points[-1, 1]], s=80, label="end")
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Outer boundary polygon from manual trace")
    plt.legend()
    plt.savefig(preview_png, dpi=200, bbox_inches="tight")
    plt.savefig(latest_preview, dpi=200, bbox_inches="tight")
    plt.close()

    # compare
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    axes[0].plot(points[:, 0], points[:, 1], linewidth=0.9)
    axes[0].set_title("Actual trace")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")

    axes[1].plot(points[:, 0], points[:, 1], linewidth=0.8, alpha=0.25, label="actual trace")
    axes[1].plot(boundary[:, 0], boundary[:, 1], ".", markersize=3, alpha=0.7, label="outer selected points")
    axes[1].plot(polygon_closed[:, 0], polygon_closed[:, 1], "-o", linewidth=2.5, markersize=5, label="outer boundary polygon")

    for i, p in enumerate(polygon):
        axes[1].text(p[0], p[1], str(i), fontsize=9)

    info = (
        f"vertices: {len(polygon)}\n"
        f"area: {area:.3f} m^2\n"
        f"perimeter: {perimeter:.3f} m\n"
        f"bins: {args.n_bins}\n"
        f"percentile: {args.percentile}\n"
        f"epsilon: {args.epsilon}"
    )
    axes[1].text(
        0.02,
        0.98,
        info,
        transform=axes[1].transAxes,
        va="top",
        bbox=dict(boxstyle="round", alpha=0.2),
    )

    axes[1].set_title("Actual trace vs outer boundary polygon")
    axes[1].axis("equal")
    axes[1].grid(True)
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    axes[1].legend()

    plt.tight_layout()
    plt.savefig(compare_png, dpi=200, bbox_inches="tight")
    plt.savefig(latest_compare, dpi=200, bbox_inches="tight")
    plt.close()

    print("source csv:", csv_path)
    print("out dir:", out_dir)
    print("json:", json_path)
    print("vertices:", vertices_csv)
    print("preview:", preview_png)
    print("compare:", compare_png)
    print("latest preview:", latest_preview)
    print("latest compare:", latest_compare)
    print("")
    print("num_vertices:", len(polygon))
    print("area_m2:", area)
    print("perimeter_m:", perimeter)


if __name__ == "__main__":
    main()
