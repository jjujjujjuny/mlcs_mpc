#!/usr/bin/env python3

import os
import csv
import json
import glob
import time
import math
import argparse
import numpy as np
import pandas as pd


MANUAL_ROOT = "/home/user/Desktop/turtlebot_data/experiments/lab_boundary_manual"
MAP_SAVE_ROOT = "/home/user/Desktop/map_data"


def latest_manual_csv():
    files = glob.glob(os.path.join(MANUAL_ROOT, "session_*", "manual_lab_boundary_motion.csv"))
    if not files:
        raise FileNotFoundError("No manual lab boundary CSV found.")
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def polygon_signed_area(points):
    area = 0.0
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        area += points[i]["x"] * points[j]["y"]
        area -= points[j]["x"] * points[i]["y"]
    return area * 0.5


def polygon_area(points):
    return abs(polygon_signed_area(points))


def polygon_perimeter(points):
    p = 0.0
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        dx = points[j]["x"] - points[i]["x"]
        dy = points[j]["y"] - points[i]["y"]
        p += math.sqrt(dx * dx + dy * dy)
    return p


def centroid(points):
    signed_area = polygon_signed_area(points)
    if abs(signed_area) < 1e-12:
        return (
            sum(p["x"] for p in points) / len(points),
            sum(p["y"] for p in points) / len(points),
        )

    cx = 0.0
    cy = 0.0
    n = len(points)

    for i in range(n):
        j = (i + 1) % n
        cross = points[i]["x"] * points[j]["y"] - points[j]["x"] * points[i]["y"]
        cx += (points[i]["x"] + points[j]["x"]) * cross
        cy += (points[i]["y"] + points[j]["y"]) * cross

    cx /= 6.0 * signed_area
    cy /= 6.0 * signed_area
    return cx, cy


def edge_lengths(points):
    edges = []
    n = len(points)

    for i in range(n):
        j = (i + 1) % n
        dx = points[j]["x"] - points[i]["x"]
        dy = points[j]["y"] - points[i]["y"]
        length = math.sqrt(dx * dx + dy * dy)
        heading = math.atan2(dy, dx)

        edges.append({
            "edge_id": i,
            "from": points[i]["label"],
            "to": points[j]["label"],
            "from_point_id": points[i]["point_id"],
            "to_point_id": points[j]["point_id"],
            "dx": dx,
            "dy": dy,
            "length": length,
            "heading": heading,
        })

    return edges


def find_stationary_segments(df, speed_threshold, min_duration, min_gap):
    """
    Find segments where model_speed is below threshold for at least min_duration.

    Consecutive stationary parts separated by less than min_gap are merged.
    """

    d = df.copy()
    d["is_stationary"] = d["model_speed"].abs() < speed_threshold

    segments = []
    in_seg = False
    start_idx = None

    for idx, row in d.iterrows():
        if row["is_stationary"] and not in_seg:
            in_seg = True
            start_idx = idx

        if in_seg and (not row["is_stationary"]):
            end_idx = idx - 1
            segments.append((start_idx, end_idx))
            in_seg = False
            start_idx = None

    if in_seg:
        segments.append((start_idx, d.index[-1]))

    # duration filter
    filtered = []
    for s, e in segments:
        t0 = d.loc[s, "time"]
        t1 = d.loc[e, "time"]
        if t1 - t0 >= min_duration:
            filtered.append((s, e))

    # merge close stationary segments
    merged = []
    for seg in filtered:
        if not merged:
            merged.append(seg)
            continue

        prev_s, prev_e = merged[-1]
        cur_s, cur_e = seg

        gap = d.loc[cur_s, "time"] - d.loc[prev_e, "time"]

        if gap < min_gap:
            merged[-1] = (prev_s, cur_e)
        else:
            merged.append(seg)

    return merged


def summarize_segment(df, point_id, s, e, trim_edge_sec=0.2):
    seg = df.loc[s:e].copy()

    t0 = seg["time"].iloc[0]
    t1 = seg["time"].iloc[-1]

    # trim beginning/end to avoid hand-placement transient
    seg2 = seg[(seg["time"] >= t0 + trim_edge_sec) & (seg["time"] <= t1 - trim_edge_sec)].copy()
    if len(seg2) < 10:
        seg2 = seg

    x = seg2["model_x"].to_numpy()
    y = seg2["model_y"].to_numpy()
    z = seg2["model_z"].to_numpy()

    return {
        "point_id": point_id,
        "label": f"corner_{point_id}",
        "start_time": float(t0),
        "end_time": float(t1),
        "duration": float(t1 - t0),
        "n_samples": int(len(seg2)),

        "x": float(np.mean(x)),
        "y": float(np.mean(y)),
        "z": float(np.mean(z)),

        "x_std": float(np.std(x)),
        "y_std": float(np.std(y)),
        "z_std": float(np.std(z)),

        "speed_mean": float(seg2["model_speed"].mean()),
        "speed_max": float(seg2["model_speed"].max()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default=None)
    parser.add_argument("--speed_threshold", type=float, default=0.015)
    parser.add_argument("--min_duration", type=float, default=1.0)
    parser.add_argument("--min_gap", type=float, default=0.5)
    args = parser.parse_args()

    csv_path = args.csv if args.csv is not None else latest_manual_csv()
    df = pd.read_csv(csv_path)

    segments = find_stationary_segments(
        df,
        speed_threshold=args.speed_threshold,
        min_duration=args.min_duration,
        min_gap=args.min_gap,
    )

    if len(segments) < 3:
        raise RuntimeError(
            f"Only {len(segments)} stationary segments found. "
            "Need at least 3. Try increasing speed_threshold or stopping longer at each corner."
        )

    points = []
    for i, (s, e) in enumerate(segments):
        points.append(summarize_segment(df, i, s, e))

    stamp = int(time.time())
    save_dir = os.path.join(MAP_SAVE_ROOT, f"map_{stamp}")
    os.makedirs(save_dir, exist_ok=True)

    xs = [p["x"] for p in points]
    ys = [p["y"] for p in points]

    x_min = min(xs)
    x_max = max(xs)
    y_min = min(ys)
    y_max = max(ys)

    width_x = x_max - x_min
    height_y = y_max - y_min

    area = polygon_area(points)
    signed_area = polygon_signed_area(points)
    perimeter = polygon_perimeter(points)
    cx, cy = centroid(points)
    edges = edge_lengths(points)

    orientation = "counter_clockwise" if signed_area > 0 else "clockwise"

    map_data = {
        "created_unix_time": stamp,
        "source_manual_csv": csv_path,
        "stationary_detection": {
            "speed_threshold": args.speed_threshold,
            "min_duration": args.min_duration,
            "min_gap": args.min_gap,
            "n_segments": len(segments),
        },
        "coordinate_frame": {
            "name": "mocap_world",
            "point_reference": "model_center",
            "description": "All coordinates are corrected model-center positions in mocap world frame."
        },
        "boundary_polygon": points,
        "bounds": {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "width_x": width_x,
            "height_y": height_y,
            "center_x": 0.5 * (x_min + x_max),
            "center_y": 0.5 * (y_min + y_max),
        },
        "polygon_geometry": {
            "area": area,
            "perimeter": perimeter,
            "centroid_x": cx,
            "centroid_y": cy,
            "orientation": orientation,
            "n_vertices": len(points),
        },
        "edges": edges,
        "notes": [
            "Corners were extracted from manual stop segments.",
            "Boundary order follows the order in which the user stopped at corners.",
            "If a non-corner stop was included, remove it or repeat data collection."
        ]
    }

    json_path = os.path.join(save_dir, "lab_map.json")
    vertices_csv = os.path.join(save_dir, "lab_map_vertices.csv")
    edges_csv = os.path.join(save_dir, "lab_map_edges.csv")
    summary_md = os.path.join(save_dir, "lab_map_summary.md")
    stops_csv = os.path.join(save_dir, "detected_stationary_segments.csv")

    with open(json_path, "w") as f:
        json.dump(map_data, f, indent=2, ensure_ascii=False)

    pd.DataFrame(points).to_csv(vertices_csv, index=False)
    pd.DataFrame(edges).to_csv(edges_csv, index=False)

    stop_rows = []
    for i, (s, e) in enumerate(segments):
        stop_rows.append({
            "point_id": i,
            "start_index": int(s),
            "end_index": int(e),
            "start_time": float(df.loc[s, "time"]),
            "end_time": float(df.loc[e, "time"]),
            "duration": float(df.loc[e, "time"] - df.loc[s, "time"]),
        })
    pd.DataFrame(stop_rows).to_csv(stops_csv, index=False)

    with open(summary_md, "w") as f:
        f.write("# Manual Lab Map Summary\n\n")

        f.write("## Source\n\n")
        f.write(f"- source csv: `{csv_path}`\n\n")

        f.write("## Stationary detection\n\n")
        f.write(f"- speed_threshold: {args.speed_threshold}\n")
        f.write(f"- min_duration: {args.min_duration}\n")
        f.write(f"- detected corners: {len(points)}\n\n")

        f.write("## Bounds\n\n")
        f.write(f"- x_min: {x_min:.6f} m\n")
        f.write(f"- x_max: {x_max:.6f} m\n")
        f.write(f"- y_min: {y_min:.6f} m\n")
        f.write(f"- y_max: {y_max:.6f} m\n")
        f.write(f"- width_x: {width_x:.6f} m\n")
        f.write(f"- height_y: {height_y:.6f} m\n\n")

        f.write("## Polygon geometry\n\n")
        f.write(f"- area: {area:.6f} m^2\n")
        f.write(f"- perimeter: {perimeter:.6f} m\n")
        f.write(f"- centroid_x: {cx:.6f} m\n")
        f.write(f"- centroid_y: {cy:.6f} m\n")
        f.write(f"- orientation: {orientation}\n\n")

        f.write("## Vertices\n\n")
        f.write("| point_id | label | x [m] | y [m] | duration [s] | std_x [m] | std_y [m] |\n")
        f.write("|---:|---|---:|---:|---:|---:|---:|\n")
        for p in points:
            f.write(
                f"| {p['point_id']} | {p['label']} | "
                f"{p['x']:.6f} | {p['y']:.6f} | "
                f"{p['duration']:.3f} | "
                f"{p['x_std']:.6f} | {p['y_std']:.6f} |\n"
            )

        f.write("\n## Edges\n\n")
        f.write("| edge_id | from | to | length [m] | heading [rad] |\n")
        f.write("|---:|---|---|---:|---:|\n")
        for e in edges:
            f.write(
                f"| {e['edge_id']} | {e['from']} | {e['to']} | "
                f"{e['length']:.6f} | {e['heading']:.6f} |\n"
            )

    print("Manual lab map created.")
    print("source:", csv_path)
    print("save dir:", save_dir)
    print("json:", json_path)
    print("vertices:", vertices_csv)
    print("edges:", edges_csv)
    print("stops:", stops_csv)
    print("summary:", summary_md)
    print("")
    print("detected corners:", len(points))
    print("width_x:", width_x)
    print("height_y:", height_y)
    print("area:", area)
    print("perimeter:", perimeter)


if __name__ == "__main__":
    main()
