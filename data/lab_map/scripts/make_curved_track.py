#!/usr/bin/env python3
import os
import json
import time
import math
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


DEFAULT_BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
DEFAULT_WAYPOINT_JSON = "/home/user/Desktop/map_data/track_data/latest_track_waypoints.json"
DEFAULT_SAVE_ROOT = "/home/user/Desktop/map_data/track_curved_data"


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def normalize_waypoints(obj):
    if "track" in obj and "waypoints" in obj["track"]:
        pts = obj["track"]["waypoints"]
    elif "waypoints" in obj:
        pts = obj["waypoints"]
    else:
        raise ValueError("waypoints json 구조를 찾지 못함")

    out = []
    for p in pts:
        if isinstance(p, dict):
            out.append([float(p["x"]), float(p["y"])])
        else:
            out.append([float(p[0]), float(p[1])])
    return np.array(out, dtype=float)


def close_points(points):
    if len(points) < 2:
        return points
    if np.linalg.norm(points[0] - points[-1]) < 1e-9:
        return points
    return np.vstack([points, points[0]])


def catmull_rom_closed(points, samples_per_seg=20):
    """
    Closed Catmull-Rom spline through control points.
    """
    pts = np.array(points, dtype=float)
    n = len(pts)
    if n < 3:
        return pts.copy()

    curve = []
    for i in range(n):
        p0 = pts[(i - 1) % n]
        p1 = pts[i % n]
        p2 = pts[(i + 1) % n]
        p3 = pts[(i + 2) % n]

        for j in range(samples_per_seg):
            t = j / float(samples_per_seg)
            t2 = t * t
            t3 = t2 * t

            # Catmull-Rom basis
            point = 0.5 * (
                (2.0 * p1) +
                (-p0 + p2) * t +
                (2.0*p0 - 5.0*p1 + 4.0*p2 - p3) * t2 +
                (-p0 + 3.0*p1 - 3.0*p2 + p3) * t3
            )
            curve.append(point)

    return np.array(curve, dtype=float)


def cumulative_lengths(points, closed=True):
    pts = np.array(points, dtype=float)
    if closed:
        pts2 = np.vstack([pts, pts[0]])
    else:
        pts2 = pts.copy()

    seg = np.diff(pts2, axis=0)
    d = np.linalg.norm(seg, axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    return s, d.sum()


def resample_closed_curve(points, spacing=0.03):
    pts = np.array(points, dtype=float)
    pts2 = np.vstack([pts, pts[0]])
    seg = np.diff(pts2, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total_len = cum[-1]

    if total_len < 1e-9:
        return pts.copy()

    n_samples = max(20, int(total_len / spacing))
    targets = np.linspace(0.0, total_len, n_samples, endpoint=False)

    new_pts = []
    for t in targets:
        idx = np.searchsorted(cum, t, side="right") - 1
        idx = min(max(idx, 0), len(seg_len) - 1)

        l0 = cum[idx]
        l1 = cum[idx + 1]
        if l1 - l0 < 1e-12:
            alpha = 0.0
        else:
            alpha = (t - l0) / (l1 - l0)

        p = (1.0 - alpha) * pts2[idx] + alpha * pts2[idx + 1]
        new_pts.append(p)

    return np.array(new_pts, dtype=float)


def smooth_closed_curve(points, iterations=5, alpha=0.25):
    pts = np.array(points, dtype=float)
    n = len(pts)
    out = pts.copy()

    for _ in range(iterations):
        new_out = out.copy()
        for i in range(n):
            prev_p = out[(i - 1) % n]
            curr_p = out[i]
            next_p = out[(i + 1) % n]
            lap = 0.5 * (prev_p + next_p) - curr_p
            new_out[i] = curr_p + alpha * lap
        out = new_out

    return out


def expand_about_centroid(points, scale=1.0):
    pts = np.array(points, dtype=float)
    c = pts.mean(axis=0)
    return c + scale * (pts - c)


def compute_tangent_normal(points):
    pts = np.array(points, dtype=float)
    n = len(pts)
    tangents = np.zeros_like(pts)
    normals = np.zeros_like(pts)

    for i in range(n):
        prev_p = pts[(i - 1) % n]
        next_p = pts[(i + 1) % n]
        t = next_p - prev_p
        norm = np.linalg.norm(t)
        if norm < 1e-12:
            t = np.array([1.0, 0.0])
            norm = 1.0
        t = t / norm
        tangents[i] = t
        normals[i] = np.array([-t[1], t[0]])

    return tangents, normals


def offset_curve(points, offset):
    pts = np.array(points, dtype=float)
    _, normals = compute_tangent_normal(pts)
    return pts + offset * normals


def polygon_area(points):
    pts = np.array(points, dtype=float)
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1))


def ensure_ccw(points):
    pts = np.array(points, dtype=float)
    if polygon_area(pts) < 0:
        pts = pts[::-1]
    return pts


def to_list_of_dict(points):
    return [{"x": float(p[0]), "y": float(p[1])} for p in points]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--boundary_json", type=str, default=DEFAULT_BOUNDARY_JSON)
    parser.add_argument("--waypoint_json", type=str, default=DEFAULT_WAYPOINT_JSON)
    parser.add_argument("--save_root", type=str, default=DEFAULT_SAVE_ROOT)

    parser.add_argument("--width", type=float, default=0.60)
    parser.add_argument("--spacing", type=float, default=0.03)
    parser.add_argument("--samples_per_seg", type=int, default=25)
    parser.add_argument("--smooth_iter", type=int, default=6)
    parser.add_argument("--smooth_alpha", type=float, default=0.22)
    parser.add_argument("--scale", type=float, default=1.00)

    args = parser.parse_args()

    boundary_data = load_json(args.boundary_json)
    boundary_vertices = boundary_data["boundary"]["vertices"]
    map_boundary = np.array([[float(v["x"]), float(v["y"])] for v in boundary_vertices], dtype=float)
    map_boundary = ensure_ccw(map_boundary)

    waypoint_data = load_json(args.waypoint_json)
    clicked = normalize_waypoints(waypoint_data)
    clicked = ensure_ccw(clicked)

    # 1) smooth centerline from clicked waypoints
    smooth0 = catmull_rom_closed(clicked, samples_per_seg=args.samples_per_seg)
    smooth1 = resample_closed_curve(smooth0, spacing=args.spacing)
    smooth2 = smooth_closed_curve(smooth1, iterations=args.smooth_iter, alpha=args.smooth_alpha)
    smooth3 = resample_closed_curve(smooth2, spacing=args.spacing)

    # 2) expand if desired
    centerline = expand_about_centroid(smooth3, scale=args.scale)
    centerline = resample_closed_curve(centerline, spacing=args.spacing)

    # 3) curved offset boundaries from curved centerline
    half_w = args.width / 2.0
    left_boundary = offset_curve(centerline, +half_w)
    right_boundary = offset_curve(centerline, -half_w)

    # 4) additional smoothing on boundaries too
    left_boundary = smooth_closed_curve(left_boundary, iterations=2, alpha=0.15)
    right_boundary = smooth_closed_curve(right_boundary, iterations=2, alpha=0.15)

    left_boundary = resample_closed_curve(left_boundary, spacing=args.spacing)
    right_boundary = resample_closed_curve(right_boundary, spacing=args.spacing)

    stamp = int(time.time())
    save_root = Path(args.save_root)
    save_root.mkdir(parents=True, exist_ok=True)

    session_dir = save_root / f"track_{stamp}"
    session_dir.mkdir(parents=True, exist_ok=True)

    json_path = session_dir / "track_curved_data.json"
    preview_path = session_dir / "track_curved_preview.png"

    latest_json = save_root / "latest_track_curved_data.json"
    latest_png = save_root / "latest_track_curved_preview.png"

    out = {
        "created_unix_time": stamp,
        "source_boundary_json": args.boundary_json,
        "source_waypoint_json": args.waypoint_json,
        "coordinate_frame": boundary_data.get("coordinate_frame", {}),
        "track": {
            "type": "curved_closed_track",
            "closed": True,
            "width": float(args.width),
            "spacing": float(args.spacing),
            "scale": float(args.scale),
            "samples_per_seg": int(args.samples_per_seg),
            "smooth_iter": int(args.smooth_iter),
            "smooth_alpha": float(args.smooth_alpha),
            "clicked_waypoints": to_list_of_dict(clicked),
            "smooth_centerline_before_expand": to_list_of_dict(smooth3),
            "generated_centerline": to_list_of_dict(centerline),
            "left_boundary": to_list_of_dict(left_boundary),
            "right_boundary": to_list_of_dict(right_boundary),
        }
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    with open(latest_json, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    # plot
    fig, ax = plt.subplots(figsize=(10, 10))

    mb = np.vstack([map_boundary, map_boundary[0]])
    ax.plot(mb[:, 0], mb[:, 1], linewidth=3, label="map boundary")

    clicked_closed = np.vstack([clicked, clicked[0]])
    ax.plot(
        clicked_closed[:, 0], clicked_closed[:, 1],
        "o--", linewidth=1.4, markersize=5,
        label="clicked waypoints"
    )

    smooth3_closed = np.vstack([smooth3, smooth3[0]])
    ax.plot(
        smooth3_closed[:, 0], smooth3_closed[:, 1],
        "-", linewidth=2.2,
        label="smooth centerline (before expand)"
    )

    center_closed = np.vstack([centerline, centerline[0]])
    left_closed = np.vstack([left_boundary, left_boundary[0]])
    right_closed = np.vstack([right_boundary, right_boundary[0]])

    ax.plot(center_closed[:, 0], center_closed[:, 1], linewidth=3.2, label="generated centerline")
    ax.plot(left_closed[:, 0], left_closed[:, 1], linewidth=2.6, label="left boundary (curved)")
    ax.plot(right_closed[:, 0], right_closed[:, 1], linewidth=2.6, label="right boundary (curved)")

    for i, p in enumerate(clicked):
        ax.text(p[0], p[1], f"W{i}", fontsize=8)

    ax.set_title(
        f"Curved track from clicked waypoints | width={args.width:.2f} m | scale={args.scale:.3f}"
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="best")

    fig.savefig(preview_path, dpi=200, bbox_inches="tight")
    fig.savefig(latest_png, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("Curved track generation finished.")
    print("session dir :", session_dir)
    print("json        :", json_path)
    print("preview png :", preview_path)
    print("latest json :", latest_json)
    print("latest png  :", latest_png)


if __name__ == "__main__":
    main()
