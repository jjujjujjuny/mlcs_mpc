#!/usr/bin/env python3

import os
import json
import csv
import math
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


DEFAULT_TRACK_JSON = "/home/user/Desktop/map_data/track_data/latest_track_waypoints.json"
DEFAULT_BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
SAVE_DIR = "/home/user/Desktop/map_data/track_linear_data"


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_points_csv(path, points, header=("idx", "x", "y")):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i, p in enumerate(points):
            writer.writerow([i, float(p[0]), float(p[1])])


def close_if_needed(points, closed):
    pts = np.asarray(points, dtype=float)
    if not closed:
        return pts
    if len(pts) < 2:
        return pts
    if np.linalg.norm(pts[0] - pts[-1]) < 1e-12:
        return pts
    return np.vstack([pts, pts[0]])


def polygon_area(points):
    pts = np.asarray(points, dtype=float)
    pts2 = close_if_needed(pts, True)
    x = pts2[:, 0]
    y = pts2[:, 1]
    return 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])


def ensure_ccw(points):
    pts = np.asarray(points, dtype=float)
    if polygon_area(pts) < 0:
        return pts[::-1].copy()
    return pts.copy()


def chaikin_closed(points, iterations=3):
    pts = np.asarray(points, dtype=float)
    pts = ensure_ccw(pts)

    for _ in range(iterations):
        new_pts = []
        n = len(pts)
        for i in range(n):
            p0 = pts[i]
            p1 = pts[(i + 1) % n]
            q = 0.75 * p0 + 0.25 * p1
            r = 0.25 * p0 + 0.75 * p1
            new_pts.append(q)
            new_pts.append(r)
        pts = np.asarray(new_pts, dtype=float)
    return pts


def chaikin_open(points, iterations=3):
    pts = np.asarray(points, dtype=float)
    for _ in range(iterations):
        new_pts = [pts[0]]
        for i in range(len(pts) - 1):
            p0 = pts[i]
            p1 = pts[i + 1]
            q = 0.75 * p0 + 0.25 * p1
            r = 0.25 * p0 + 0.75 * p1
            new_pts.append(q)
            new_pts.append(r)
        new_pts.append(pts[-1])
        pts = np.asarray(new_pts, dtype=float)
    return pts


def resample_polyline(points, spacing=0.05, closed=False):
    pts = np.asarray(points, dtype=float)
    pts_ext = close_if_needed(pts, closed)

    seg_vec = np.diff(pts_ext, axis=0)
    seg_len = np.linalg.norm(seg_vec, axis=1)
    total_len = float(np.sum(seg_len))

    if total_len < 1e-12:
        return pts.copy()

    s = np.concatenate([[0.0], np.cumsum(seg_len)])

    if closed:
        n_samples = max(int(total_len / spacing), 20)
        s_new = np.linspace(0.0, total_len, n_samples, endpoint=False)
    else:
        n_samples = max(int(total_len / spacing) + 1, 2)
        s_new = np.linspace(0.0, total_len, n_samples, endpoint=True)

    out = []
    seg_idx = 0
    for sv in s_new:
        while seg_idx < len(seg_len) - 1 and s[seg_idx + 1] < sv:
            seg_idx += 1

        s0 = s[seg_idx]
        s1 = s[seg_idx + 1]
        p0 = pts_ext[seg_idx]
        p1 = pts_ext[seg_idx + 1]

        if abs(s1 - s0) < 1e-12:
            out.append(p0)
        else:
            a = (sv - s0) / (s1 - s0)
            out.append((1.0 - a) * p0 + a * p1)

    return np.asarray(out, dtype=float)


def compute_tangent(points, closed=False):
    pts = np.asarray(points, dtype=float)
    n = len(pts)
    tangents = np.zeros_like(pts)

    for i in range(n):
        if closed:
            p_prev = pts[(i - 1) % n]
            p_next = pts[(i + 1) % n]
        else:
            if i == 0:
                p_prev = pts[i]
                p_next = pts[i + 1]
            elif i == n - 1:
                p_prev = pts[i - 1]
                p_next = pts[i]
            else:
                p_prev = pts[i - 1]
                p_next = pts[i + 1]

        v = p_next - p_prev
        nv = np.linalg.norm(v)

        if nv < 1e-12:
            tangents[i] = np.array([1.0, 0.0])
        else:
            tangents[i] = v / nv

    return tangents


def make_boundaries(centerline, half_width, closed=False):
    tangents = compute_tangent(centerline, closed=closed)

    normals = np.zeros_like(tangents)
    normals[:, 0] = -tangents[:, 1]
    normals[:, 1] = tangents[:, 0]

    left = centerline + half_width * normals
    right = centerline - half_width * normals
    return left, right


def point_in_polygon(x, y, polygon):
    inside = False
    n = len(polygon)
    j = n - 1

    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        intersect = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
        )
        if intersect:
            inside = not inside
        j = i

    return inside


def points_inside_polygon(points, polygon):
    polygon = np.asarray(polygon, dtype=float)
    return np.array([point_in_polygon(p[0], p[1], polygon) for p in points], dtype=bool)


def scale_about_centroid(points, scale):
    pts = np.asarray(points, dtype=float)
    c = np.mean(pts, axis=0)
    return c + scale * (pts - c)


def track_is_valid(centerline, half_width, map_boundary, closed):
    left, right = make_boundaries(centerline, half_width=half_width, closed=closed)

    inside_c = points_inside_polygon(centerline, map_boundary).all()
    inside_l = points_inside_polygon(left, map_boundary).all()
    inside_r = points_inside_polygon(right, map_boundary).all()

    return inside_c and inside_l and inside_r, left, right


def find_max_safe_scale(base_centerline, half_width, map_boundary, closed, max_scale=1.8, steps=25):
    valid, _, _ = track_is_valid(base_centerline, half_width, map_boundary, closed)
    if not valid:
        return 1.0

    low = 1.0
    high = max_scale

    scaled_high = scale_about_centroid(base_centerline, high)
    valid_high, _, _ = track_is_valid(scaled_high, half_width, map_boundary, closed)
    if valid_high:
        return high

    for _ in range(steps):
        mid = 0.5 * (low + high)
        scaled_mid = scale_about_centroid(base_centerline, mid)
        valid_mid, _, _ = track_is_valid(scaled_mid, half_width, map_boundary, closed)

        if valid_mid:
            low = mid
        else:
            high = mid

    return low


def path_length(points, closed=False):
    pts = close_if_needed(points, closed)
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def load_clicked_waypoints(track_json):
    data = load_json(track_json)
    waypoints = data["track"]["waypoints"]
    pts = np.array([[float(w["x"]), float(w["y"])] for w in waypoints], dtype=float)
    return pts, data


def load_boundary(boundary_json):
    data = load_json(boundary_json)
    vertices = data["boundary"]["vertices"]
    pts = np.array([[float(v["x"]), float(v["y"])] for v in vertices], dtype=float)
    return ensure_ccw(pts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--track_json", default=DEFAULT_TRACK_JSON)
    parser.add_argument("--boundary_json", default=DEFAULT_BOUNDARY_JSON)
    parser.add_argument("--width", type=float, default=0.60)
    parser.add_argument("--spacing", type=float, default=0.05)
    parser.add_argument("--curve_iter", type=int, default=3)
    parser.add_argument("--max_scale", type=float, default=1.8)
    parser.add_argument("--closed", action="store_true")
    args = parser.parse_args()

    os.makedirs(SAVE_DIR, exist_ok=True)

    clicked_pts, track_data = load_clicked_waypoints(args.track_json)
    map_boundary = load_boundary(args.boundary_json)

    if len(clicked_pts) < 2:
        raise RuntimeError("waypoint가 너무 적어. 최소 2개 이상 필요.")

    half_width = args.width * 0.5

    # 1) clicked waypoints -> smooth curve
    if args.closed:
        smooth_pts = chaikin_closed(clicked_pts, iterations=args.curve_iter)
    else:
        smooth_pts = chaikin_open(clicked_pts, iterations=args.curve_iter)

    smooth_centerline = resample_polyline(
        smooth_pts,
        spacing=args.spacing,
        closed=args.closed
    )

    # 2) outward expansion
    best_scale = find_max_safe_scale(
        smooth_centerline,
        half_width=half_width,
        map_boundary=map_boundary,
        closed=args.closed,
        max_scale=args.max_scale,
        steps=30
    )

    expanded_centerline = scale_about_centroid(smooth_centerline, best_scale)

    valid, left_boundary, right_boundary = track_is_valid(
        expanded_centerline,
        half_width=half_width,
        map_boundary=map_boundary,
        closed=args.closed
    )

    if not valid:
        raise RuntimeError("확장 후 트랙이 map boundary를 벗어남. 파라미터를 줄여야 함.")

    clicked_csv = Path(SAVE_DIR) / "track_clicked_waypoints.csv"
    smooth_csv = Path(SAVE_DIR) / "track_smooth_centerline.csv"
    center_csv = Path(SAVE_DIR) / "track_centerline.csv"
    left_csv = Path(SAVE_DIR) / "track_left_boundary.csv"
    right_csv = Path(SAVE_DIR) / "track_right_boundary.csv"
    json_path = Path(SAVE_DIR) / "track_linear_data.json"
    preview_png = Path(SAVE_DIR) / "track_linear_preview.png"

    save_points_csv(clicked_csv, clicked_pts, header=("waypoint_id", "x", "y"))
    save_points_csv(smooth_csv, smooth_centerline)
    save_points_csv(center_csv, expanded_centerline)
    save_points_csv(left_csv, left_boundary)
    save_points_csv(right_csv, right_boundary)

    result = {
        "source_track_json": args.track_json,
        "source_boundary_json": args.boundary_json,
        "method": "smooth_curve_then_expand_within_map_boundary",
        "track": {
            "closed": bool(args.closed),
            "width_m": float(args.width),
            "half_width_m": float(half_width),
            "resample_spacing_m": float(args.spacing),
            "curve_iter": int(args.curve_iter),
            "max_scale_requested": float(args.max_scale),
            "scale_used": float(best_scale),
            "clicked_waypoint_count": int(len(clicked_pts)),
            "smooth_centerline_count": int(len(smooth_centerline)),
            "centerline_point_count": int(len(expanded_centerline)),
            "centerline_length_m": path_length(expanded_centerline, closed=args.closed),
        },
        "files": {
            "clicked_waypoints_csv": str(clicked_csv),
            "smooth_centerline_csv": str(smooth_csv),
            "centerline_csv": str(center_csv),
            "left_boundary_csv": str(left_csv),
            "right_boundary_csv": str(right_csv),
            "preview_png": str(preview_png),
        },
        "notes": [
            "Clicked waypoints are first smoothed into a curve.",
            "Then the curved centerline is expanded outward about its centroid.",
            "The final scale is chosen automatically so that the centerline and both width boundaries stay inside the map boundary."
        ]
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    latest_json = Path(SAVE_DIR) / "latest_track_linear_data.json"
    latest_center = Path(SAVE_DIR) / "latest_track_centerline.csv"
    latest_left = Path(SAVE_DIR) / "latest_track_left_boundary.csv"
    latest_right = Path(SAVE_DIR) / "latest_track_right_boundary.csv"
    latest_preview = Path(SAVE_DIR) / "latest_track_linear_preview.png"

    with open(latest_json, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    save_points_csv(latest_center, expanded_centerline)
    save_points_csv(latest_left, left_boundary)
    save_points_csv(latest_right, right_boundary)

    fig, ax = plt.subplots(figsize=(10, 10))

    bd_plot = close_if_needed(map_boundary, True)
    ax.plot(bd_plot[:, 0], bd_plot[:, 1], linewidth=2.5, label="map boundary")

    clicked_plot = close_if_needed(clicked_pts, args.closed)
    smooth_plot = close_if_needed(smooth_centerline, args.closed)
    center_plot = close_if_needed(expanded_centerline, args.closed)
    left_plot = close_if_needed(left_boundary, args.closed)
    right_plot = close_if_needed(right_boundary, args.closed)

    ax.plot(
        clicked_plot[:, 0], clicked_plot[:, 1],
        "--o", linewidth=1.5, markersize=5,
        label="clicked waypoints"
    )

    ax.plot(
        smooth_plot[:, 0], smooth_plot[:, 1],
        linewidth=2.0,
        label="smooth centerline (before expand)"
    )

    ax.plot(
        center_plot[:, 0], center_plot[:, 1],
        linewidth=3.0,
        label="generated centerline"
    )

    ax.plot(
        left_plot[:, 0], left_plot[:, 1],
        linewidth=2.0,
        label="left boundary"
    )

    ax.plot(
        right_plot[:, 0], right_plot[:, 1],
        linewidth=2.0,
        label="right boundary"
    )

    for i, p in enumerate(clicked_pts):
        ax.text(p[0], p[1], f"W{i}", fontsize=8)

    ax.set_title(
        f"Curved track from clicked waypoints | width={args.width:.2f} m | scale={best_scale:.3f}"
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="best")

    fig.savefig(preview_png, dpi=200, bbox_inches="tight")
    fig.savefig(latest_preview, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("Curved track created.")
    print("clicked waypoint count :", len(clicked_pts))
    print("curve_iter             :", args.curve_iter)
    print("track width [m]        :", args.width)
    print("scale used             :", best_scale)
    print("closed                 :", args.closed)
    print("")
    print("json                   :", json_path)
    print("centerline csv         :", center_csv)
    print("left boundary csv      :", left_csv)
    print("right boundary csv     :", right_csv)
    print("preview                :", preview_png)


if __name__ == "__main__":
    main()
