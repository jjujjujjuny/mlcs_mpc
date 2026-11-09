#!/usr/bin/env python3

import os
import json
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


WAYPOINT_JSON = "/home/user/Desktop/map_data/track_data/latest_track_waypoints.json"
BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
SAVE_ROOT = "/home/user/Desktop/map_data/track_smooth_data"


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def close_ring(points):
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return pts
    if np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
        return pts
    return np.vstack([pts, pts[0]])


def remove_duplicate_last(points):
    pts = np.asarray(points, dtype=float)
    if len(pts) >= 2 and np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
        return pts[:-1]
    return pts


def load_waypoints(path):
    data = load_json(path)
    wps = data["track"]["waypoints"]
    return np.array([[float(p["x"]), float(p["y"])] for p in wps], dtype=float)


def load_boundary(path):
    data = load_json(path)
    verts = data["boundary"]["vertices"]
    return np.array([[float(p["x"]), float(p["y"])] for p in verts], dtype=float)


def remove_near_consecutive(points, min_dist):
    pts = remove_duplicate_last(points)
    kept = [pts[0]]

    for p in pts[1:]:
        if np.linalg.norm(p - kept[-1]) >= min_dist:
            kept.append(p)

    if len(kept) >= 2 and np.linalg.norm(kept[0] - kept[-1]) < min_dist:
        kept = kept[:-1]

    return np.array(kept, dtype=float)


def point_line_distance(p, a, b):
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < 1e-12:
        return np.linalg.norm(p - a)
    t = np.dot(p - a, ab) / denom
    t = max(0.0, min(1.0, t))
    proj = a + t * ab
    return np.linalg.norm(p - proj)


def rdp_open(points, eps):
    pts = np.asarray(points, dtype=float)

    if len(pts) < 3:
        return pts

    a = pts[0]
    b = pts[-1]

    dmax = -1.0
    idx = -1

    for i in range(1, len(pts) - 1):
        d = point_line_distance(pts[i], a, b)
        if d > dmax:
            dmax = d
            idx = i

    if dmax > eps:
        left = rdp_open(pts[:idx + 1], eps)
        right = rdp_open(pts[idx:], eps)
        return np.vstack([left[:-1], right])
    else:
        return np.array([a, b])


def rdp_closed(points, eps):
    pts = remove_duplicate_last(points)
    closed = np.vstack([pts, pts[0]])
    simp = rdp_open(closed, eps)

    if len(simp) >= 2 and np.linalg.norm(simp[0] - simp[-1]) < 1e-9:
        simp = simp[:-1]

    return simp


def limit_num_points(points, max_points):
    pts = np.asarray(points, dtype=float)
    n = len(pts)

    if n <= max_points:
        return pts

    idx = np.linspace(0, n, max_points, endpoint=False).astype(int)
    return pts[idx]


def catmull_rom_closed(points, samples_per_seg):
    pts = remove_duplicate_last(points)
    n = len(pts)

    if n < 3:
        return pts.copy()

    curve = []

    for i in range(n):
        p0 = pts[(i - 1) % n]
        p1 = pts[i]
        p2 = pts[(i + 1) % n]
        p3 = pts[(i + 2) % n]

        for j in range(samples_per_seg):
            t = j / float(samples_per_seg)
            t2 = t * t
            t3 = t2 * t

            p = 0.5 * (
                2.0 * p1
                + (-p0 + p2) * t
                + (2.0*p0 - 5.0*p1 + 4.0*p2 - p3) * t2
                + (-p0 + 3.0*p1 - 3.0*p2 + p3) * t3
            )
            curve.append(p)

    return np.array(curve, dtype=float)


def smooth_closed(points, iterations, alpha):
    pts = remove_duplicate_last(points).copy()
    n = len(pts)

    for _ in range(iterations):
        nxt = pts.copy()
        for i in range(n):
            prev_p = pts[(i - 1) % n]
            curr_p = pts[i]
            next_p = pts[(i + 1) % n]
            nxt[i] = curr_p + alpha * (0.5 * (prev_p + next_p) - curr_p)
        pts = nxt

    return pts


def resample_closed(points, spacing):
    pts = remove_duplicate_last(points)
    pts2 = close_ring(pts)

    seg = np.diff(pts2, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    total = float(np.sum(seg_len))

    n_samples = max(int(total / spacing), 40)
    targets = np.linspace(0.0, total, n_samples, endpoint=False)

    cum = np.concatenate([[0.0], np.cumsum(seg_len)])

    out = []
    k = 0

    for s in targets:
        while k < len(seg_len) - 1 and cum[k + 1] < s:
            k += 1

        s0 = cum[k]
        s1 = cum[k + 1]
        p0 = pts2[k]
        p1 = pts2[k + 1]

        if s1 - s0 < 1e-12:
            out.append(p0)
        else:
            a = (s - s0) / (s1 - s0)
            out.append((1.0 - a) * p0 + a * p1)

    return np.array(out, dtype=float)


def compute_normals(points):
    pts = remove_duplicate_last(points)
    n = len(pts)

    normals = np.zeros_like(pts)

    for i in range(n):
        p_prev = pts[(i - 1) % n]
        p_next = pts[(i + 1) % n]

        t = p_next - p_prev
        nt = np.linalg.norm(t)

        if nt < 1e-12:
            t = np.array([1.0, 0.0])
        else:
            t = t / nt

        normals[i] = np.array([-t[1], t[0]])

    return normals


def offset_curve(points, offset):
    pts = remove_duplicate_last(points)
    normals = compute_normals(pts)
    return pts + offset * normals


def expand_about_centroid(points, scale):
    pts = np.asarray(points, dtype=float)
    c = np.mean(pts, axis=0)
    return c + scale * (pts - c)


def save_csv(path, points):
    df = pd.DataFrame({
        "idx": np.arange(len(points)),
        "x": points[:, 0],
        "y": points[:, 1],
    })
    df.to_csv(path, index=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=float, default=0.60)
    parser.add_argument("--spacing", type=float, default=0.04)
    parser.add_argument("--min_dist", type=float, default=0.18)
    parser.add_argument("--simplify_epsilon", type=float, default=0.22)
    parser.add_argument("--max_waypoints", type=int, default=28)
    parser.add_argument("--samples_per_seg", type=int, default=30)
    parser.add_argument("--smooth_iter", type=int, default=10)
    parser.add_argument("--smooth_alpha", type=float, default=0.28)
    parser.add_argument("--scale", type=float, default=1.00)
    parser.add_argument("--label_every", type=int, default=3)
    args = parser.parse_args()

    os.makedirs(SAVE_ROOT, exist_ok=True)

    raw_wp = load_waypoints(WAYPOINT_JSON)
    boundary = load_boundary(BOUNDARY_JSON)

    # 1. 너무 촘촘한 clicked waypoint 줄이기
    wp1 = remove_near_consecutive(raw_wp, args.min_dist)
    wp2 = rdp_closed(wp1, args.simplify_epsilon)
    wp3 = limit_num_points(wp2, args.max_waypoints)

    # 2. 대표 waypoint 기반 곡선 생성
    curve0 = catmull_rom_closed(wp3, args.samples_per_seg)
    curve1 = smooth_closed(curve0, args.smooth_iter, args.smooth_alpha)
    curve2 = expand_about_centroid(curve1, args.scale)
    centerline = resample_closed(curve2, args.spacing)

    # 3. 곡선 boundary 생성
    half = args.width * 0.5
    left = offset_curve(centerline, +half)
    right = offset_curve(centerline, -half)

    left = smooth_closed(left, 4, 0.20)
    right = smooth_closed(right, 4, 0.20)

    left = resample_closed(left, args.spacing)
    right = resample_closed(right, args.spacing)

    # 4. 저장
    save_csv(os.path.join(SAVE_ROOT, "smooth_track_reduced_waypoints.csv"), wp3)
    save_csv(os.path.join(SAVE_ROOT, "smooth_track_centerline.csv"), centerline)
    save_csv(os.path.join(SAVE_ROOT, "smooth_track_left_boundary.csv"), left)
    save_csv(os.path.join(SAVE_ROOT, "smooth_track_right_boundary.csv"), right)

    latest_json = os.path.join(SAVE_ROOT, "latest_smooth_track_data.json")
    latest_png = os.path.join(SAVE_ROOT, "latest_smooth_track_preview.png")

    result = {
        "source_waypoint_json": WAYPOINT_JSON,
        "source_boundary_json": BOUNDARY_JSON,
        "method": "reduce_dense_waypoints_then_generate_smooth_curved_track",
        "parameters": vars(args),
        "counts": {
            "raw_waypoints": int(len(raw_wp)),
            "after_min_dist": int(len(wp1)),
            "after_rdp": int(len(wp2)),
            "used_waypoints": int(len(wp3)),
            "centerline_points": int(len(centerline)),
            "left_boundary_points": int(len(left)),
            "right_boundary_points": int(len(right)),
        },
        "track": {
            "width_m": float(args.width),
            "closed": True,
            "reduced_waypoints": [{"id": i, "x": float(p[0]), "y": float(p[1])} for i, p in enumerate(wp3)],
            "centerline": [{"id": i, "x": float(p[0]), "y": float(p[1])} for i, p in enumerate(centerline)],
            "left_boundary": [{"id": i, "x": float(p[0]), "y": float(p[1])} for i, p in enumerate(left)],
            "right_boundary": [{"id": i, "x": float(p[0]), "y": float(p[1])} for i, p in enumerate(right)],
        },
    }

    with open(latest_json, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    # 5. plot
    fig, ax = plt.subplots(figsize=(10, 10))

    bd = close_ring(boundary)
    raw_closed = close_ring(raw_wp)
    wp3_closed = close_ring(wp3)
    center_closed = close_ring(centerline)
    left_closed = close_ring(left)
    right_closed = close_ring(right)

    ax.plot(bd[:, 0], bd[:, 1], linewidth=2.8, label="map boundary")

    ax.plot(
        raw_closed[:, 0], raw_closed[:, 1],
        "--",
        linewidth=0.8,
        alpha=0.18,
        label="original dense clicked waypoints"
    )

    ax.plot(
        wp3_closed[:, 0], wp3_closed[:, 1],
        "o--",
        linewidth=1.3,
        markersize=4,
        alpha=0.65,
        label="reduced waypoints"
    )

    ax.plot(
        center_closed[:, 0], center_closed[:, 1],
        linewidth=3.0,
        label="smooth centerline"
    )

    ax.plot(
        left_closed[:, 0], left_closed[:, 1],
        linewidth=2.4,
        label="left boundary"
    )

    ax.plot(
        right_closed[:, 0], right_closed[:, 1],
        linewidth=2.4,
        label="right boundary"
    )

    for i, p in enumerate(wp3):
        if i % args.label_every == 0:
            ax.text(p[0], p[1], f"W{i}", fontsize=8)

    ax.set_title(
        f"Smooth curved track | width={args.width:.2f} m | used waypoints={len(wp3)} / raw={len(raw_wp)}"
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="best")

    fig.savefig(latest_png, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("Smooth curved track generated.")
    print("raw waypoints       :", len(raw_wp))
    print("used waypoints      :", len(wp3))
    print("centerline points   :", len(centerline))
    print("track width [m]     :", args.width)
    print("")
    print("json:", latest_json)
    print("png :", latest_png)
    print("")
    print("csv:")
    print("  /home/user/Desktop/map_data/track_smooth_data/smooth_track_reduced_waypoints.csv")
    print("  /home/user/Desktop/map_data/track_smooth_data/smooth_track_centerline.csv")
    print("  /home/user/Desktop/map_data/track_smooth_data/smooth_track_left_boundary.csv")
    print("  /home/user/Desktop/map_data/track_smooth_data/smooth_track_right_boundary.csv")


if __name__ == "__main__":
    main()
