#!/usr/bin/env python3

import os
import json
import math
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


CENTERLINE_JSON_CANDIDATES = [
    "/home/user/Desktop/map_data/track_reclick_data/latest_reclicked_centerline.json",
    "/home/user/Desktop/map_data/track_data/latest_track_waypoints.json",
]

BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
SAVE_ROOT = "/home/user/Desktop/map_data/track_curvature_data"


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def close_ring(points):
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return pts
    if np.linalg.norm(pts[0] - pts[-1]) < 1e-12:
        return pts
    return np.vstack([pts, pts[0]])


def remove_duplicate_last(points):
    pts = np.asarray(points, dtype=float)
    if len(pts) >= 2 and np.linalg.norm(pts[0] - pts[-1]) < 1e-12:
        return pts[:-1]
    return pts


def load_centerline_source():
    for path in CENTERLINE_JSON_CANDIDATES:
        if not os.path.exists(path):
            continue

        data = load_json(path)

        if "track" in data and "waypoints" in data["track"]:
            pts = np.array(
                [[float(p["x"]), float(p["y"])] for p in data["track"]["waypoints"]],
                dtype=float
            )
            return remove_duplicate_last(pts), path, "waypoints"

        if "track" in data and "centerline" in data["track"]:
            pts = np.array(
                [[float(p["x"]), float(p["y"])] for p in data["track"]["centerline"]],
                dtype=float
            )
            return remove_duplicate_last(pts), path, "centerline"

    raise FileNotFoundError("사용 가능한 centerline json을 찾지 못했어.")


def load_map_boundary():
    if not os.path.exists(BOUNDARY_JSON):
        return None
    data = load_json(BOUNDARY_JSON)
    verts = data["boundary"]["vertices"]
    return np.array([[float(p["x"]), float(p["y"])] for p in verts], dtype=float)


def polygon_area(points):
    p = close_ring(points)
    x = p[:, 0]
    y = p[:, 1]
    return 0.5 * np.sum(x[:-1] * y[1:] - x[1:] * y[:-1])


def ensure_ccw(points):
    pts = remove_duplicate_last(points)
    if polygon_area(pts) < 0:
        return pts[::-1].copy()
    return pts.copy()


def catmull_rom_closed(points, samples_per_seg=25):
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
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
            )

            curve.append(p)

    return np.array(curve, dtype=float)


def resample_closed(points, spacing=0.03):
    pts = remove_duplicate_last(points)
    pts2 = close_ring(pts)

    seg = np.diff(pts2, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)

    total = float(np.sum(seg_len))
    if total < 1e-12:
        return pts.copy()

    n_samples = max(int(total / spacing), 80)
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

    return np.asarray(out, dtype=float)


def laplacian_smooth_preserve_size(points, alpha=0.18):
    pts = remove_duplicate_last(points)

    c0 = np.mean(pts, axis=0)
    r0 = np.sqrt(np.mean(np.sum((pts - c0) ** 2, axis=1)))

    new_pts = pts.copy()
    n = len(pts)

    for i in range(n):
        prev_p = pts[(i - 1) % n]
        curr_p = pts[i]
        next_p = pts[(i + 1) % n]

        lap = 0.5 * (prev_p + next_p) - curr_p
        new_pts[i] = curr_p + alpha * lap

    c1 = np.mean(new_pts, axis=0)
    r1 = np.sqrt(np.mean(np.sum((new_pts - c1) ** 2, axis=1)))

    if r1 > 1e-12:
        new_pts = c0 + (new_pts - c1) * (r0 / r1)

    return new_pts


def angle_between(v1, v2):
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    return math.atan2(cross, dot)


def compute_curvature(points):
    pts = remove_duplicate_last(points)
    n = len(pts)

    kappa = np.zeros(n)
    s_step = np.zeros(n)

    for i in range(n):
        p_prev = pts[(i - 1) % n]
        p = pts[i]
        p_next = pts[(i + 1) % n]

        v1 = p - p_prev
        v2 = p_next - p

        l1 = np.linalg.norm(v1)
        l2 = np.linalg.norm(v2)

        ds = 0.5 * (l1 + l2)
        s_step[i] = ds

        if ds < 1e-12 or l1 < 1e-12 or l2 < 1e-12:
            kappa[i] = 0.0
        else:
            dtheta = angle_between(v1, v2)
            kappa[i] = dtheta / ds

    dkappa_ds = np.zeros(n)

    for i in range(n):
        kp = kappa[(i + 1) % n]
        km = kappa[(i - 1) % n]
        ds = s_step[(i + 1) % n] + s_step[i]
        if ds < 1e-12:
            dkappa_ds[i] = 0.0
        else:
            dkappa_ds[i] = (kp - km) / ds

    return kappa, dkappa_ds


def compute_tangents_normals(points):
    pts = remove_duplicate_last(points)
    n = len(pts)

    tangents = np.zeros_like(pts)
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

        tangents[i] = t
        normals[i] = np.array([-t[1], t[0]])

    return tangents, normals


def offset_curve(points, half_width):
    pts = remove_duplicate_last(points)
    _, normals = compute_tangents_normals(pts)

    left = pts + half_width * normals
    right = pts - half_width * normals

    return left, right


def path_length(points):
    p = close_ring(points)
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def save_points_csv(path, points):
    pts = remove_duplicate_last(points)
    pd.DataFrame({
        "idx": np.arange(len(pts)),
        "x": pts[:, 0],
        "y": pts[:, 1],
    }).to_csv(path, index=False)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--width", type=float, default=0.60)
    parser.add_argument("--spacing", type=float, default=0.03)

    # 곡률 제한
    # kappa_max = 1 / min_turn_radius
    # 예: R_min=0.8m 이면 kappa_max=1.25
    parser.add_argument("--kappa_max", type=float, default=1.20)
    parser.add_argument("--dkappa_max", type=float, default=8.0)

    parser.add_argument("--samples_per_seg", type=int, default=25)
    parser.add_argument("--smooth_alpha", type=float, default=0.18)
    parser.add_argument("--max_iter", type=int, default=250)
    parser.add_argument("--scale", type=float, default=1.00)

    args = parser.parse_args()

    os.makedirs(SAVE_ROOT, exist_ok=True)

    raw_points, source_json, source_kind = load_centerline_source()
    raw_points = ensure_ccw(raw_points)

    map_boundary = load_map_boundary()
    if map_boundary is not None:
        map_boundary = ensure_ccw(map_boundary)

    # 1. 초기 곡선 생성
    curve = catmull_rom_closed(raw_points, samples_per_seg=args.samples_per_seg)
    curve = resample_closed(curve, spacing=args.spacing)

    if args.scale != 1.0:
        c = np.mean(curve, axis=0)
        curve = c + args.scale * (curve - c)

    # 2. 곡률 제한까지 smoothing 반복
    converged = False

    for it in range(args.max_iter + 1):
        kappa, dkappa = compute_curvature(curve)
        max_kappa = float(np.max(np.abs(kappa)))
        max_dkappa = float(np.max(np.abs(dkappa)))

        if max_kappa <= args.kappa_max and max_dkappa <= args.dkappa_max:
            converged = True
            break

        curve = laplacian_smooth_preserve_size(curve, alpha=args.smooth_alpha)
        curve = resample_closed(curve, spacing=args.spacing)

    centerline = curve
    kappa, dkappa = compute_curvature(centerline)

    half_width = args.width * 0.5
    left_boundary, right_boundary = offset_curve(centerline, half_width)

    centerline = resample_closed(centerline, spacing=args.spacing)
    left_boundary = resample_closed(left_boundary, spacing=args.spacing)
    right_boundary = resample_closed(right_boundary, spacing=args.spacing)

    # 3. 저장
    latest_json = os.path.join(SAVE_ROOT, "latest_curvature_limited_track.json")
    center_csv = os.path.join(SAVE_ROOT, "latest_centerline.csv")
    left_csv = os.path.join(SAVE_ROOT, "latest_left_boundary.csv")
    right_csv = os.path.join(SAVE_ROOT, "latest_right_boundary.csv")
    curvature_csv = os.path.join(SAVE_ROOT, "latest_curvature_profile.csv")
    preview_png = os.path.join(SAVE_ROOT, "latest_curvature_limited_preview.png")
    curvature_png = os.path.join(SAVE_ROOT, "latest_curvature_profile.png")

    save_points_csv(center_csv, centerline)
    save_points_csv(left_csv, left_boundary)
    save_points_csv(right_csv, right_boundary)

    kappa, dkappa = compute_curvature(centerline)
    n = len(centerline)
    s = np.linspace(0.0, path_length(centerline), n, endpoint=False)

    pd.DataFrame({
        "idx": np.arange(n),
        "s": s,
        "x": centerline[:, 0],
        "y": centerline[:, 1],
        "kappa": kappa,
        "abs_kappa": np.abs(kappa),
        "dkappa_ds": dkappa,
        "abs_dkappa_ds": np.abs(dkappa),
        "turn_radius": 1.0 / np.maximum(np.abs(kappa), 1e-9),
    }).to_csv(curvature_csv, index=False)

    result = {
        "source_centerline_json": source_json,
        "source_kind": source_kind,
        "method": "curvature_limited_closed_track",
        "parameters": vars(args),
        "converged": bool(converged),
        "iterations": int(it),
        "metrics": {
            "max_abs_kappa": float(np.max(np.abs(kappa))),
            "max_abs_dkappa_ds": float(np.max(np.abs(dkappa))),
            "min_turn_radius_m": float(1.0 / max(np.max(np.abs(kappa)), 1e-9)),
            "path_length_m": path_length(centerline),
            "track_width_m": float(args.width),
        },
        "files": {
            "centerline_csv": center_csv,
            "left_boundary_csv": left_csv,
            "right_boundary_csv": right_csv,
            "curvature_profile_csv": curvature_csv,
            "preview_png": preview_png,
            "curvature_png": curvature_png,
        },
        "track": {
            "centerline": [
                {"id": int(i), "x": float(p[0]), "y": float(p[1])}
                for i, p in enumerate(centerline)
            ],
            "left_boundary": [
                {"id": int(i), "x": float(p[0]), "y": float(p[1])}
                for i, p in enumerate(left_boundary)
            ],
            "right_boundary": [
                {"id": int(i), "x": float(p[0]), "y": float(p[1])}
                for i, p in enumerate(right_boundary)
            ],
        },
    }

    with open(latest_json, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    # 4. preview plot
    fig, ax = plt.subplots(figsize=(10, 10))

    if map_boundary is not None:
        mb = close_ring(map_boundary)
        ax.plot(mb[:, 0], mb[:, 1], linewidth=2.8, label="map boundary")

    rp = close_ring(raw_points)
    cc = close_ring(centerline)
    lb = close_ring(left_boundary)
    rb = close_ring(right_boundary)

    ax.plot(rp[:, 0], rp[:, 1], "--o", alpha=0.35, linewidth=1.2, markersize=3, label="clicked/reclicked control")
    ax.plot(cc[:, 0], cc[:, 1], linewidth=3.0, label="curvature-limited centerline")
    ax.plot(lb[:, 0], lb[:, 1], linewidth=2.3, label="left boundary")
    ax.plot(rb[:, 0], rb[:, 1], linewidth=2.3, label="right boundary")

    info = (
        f"width = {args.width:.2f} m\n"
        f"kappa max target = {args.kappa_max:.3f} 1/m\n"
        f"kappa max actual = {np.max(np.abs(kappa)):.3f} 1/m\n"
        f"min radius = {1.0 / max(np.max(np.abs(kappa)), 1e-9):.3f} m\n"
        f"dkappa max actual = {np.max(np.abs(dkappa)):.3f}\n"
        f"converged = {converged}, iter = {it}"
    )

    ax.text(
        0.02, 0.02, info,
        transform=ax.transAxes,
        fontsize=10,
        bbox=dict(facecolor="white", alpha=0.75)
    )

    ax.set_title("Curvature-limited track")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="best")

    fig.savefig(preview_png, dpi=200, bbox_inches="tight")
    plt.close(fig)

    # curvature plot
    fig, ax = plt.subplots(figsize=(10, 4))

    ax.plot(s, kappa, linewidth=1.8, label="kappa")
    ax.axhline(args.kappa_max, linestyle="--", linewidth=1.2, label="+kappa limit")
    ax.axhline(-args.kappa_max, linestyle="--", linewidth=1.2, label="-kappa limit")
    ax.set_xlabel("s [m]")
    ax.set_ylabel("curvature kappa [1/m]")
    ax.set_title("Curvature profile")
    ax.grid(True)
    ax.legend(loc="best")

    fig.savefig(curvature_png, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("Curvature-limited track generated.")
    print("")
    print("source:", source_json)
    print("source kind:", source_kind)
    print("converged:", converged)
    print("iterations:", it)
    print("")
    print("target kappa max:", args.kappa_max)
    print("actual kappa max:", np.max(np.abs(kappa)))
    print("min turn radius :", 1.0 / max(np.max(np.abs(kappa)), 1e-9))
    print("actual dkappa max:", np.max(np.abs(dkappa)))
    print("")
    print("json:", latest_json)
    print("preview:", preview_png)
    print("curvature plot:", curvature_png)
    print("centerline:", center_csv)
    print("left boundary:", left_csv)
    print("right boundary:", right_csv)
    print("curvature csv:", curvature_csv)


if __name__ == "__main__":
    main()
