#!/usr/bin/env python3

import os
import csv
import json
import math
import glob
import time
from pathlib import Path


LAB_BOUNDARY_ROOT = "/home/user/Desktop/turtlebot_data/experiments/lab_boundary"
MAP_SAVE_ROOT = "/home/user/Desktop/map_data"


def latest_corners_csv():
    files = glob.glob(os.path.join(LAB_BOUNDARY_ROOT, "session_*", "lab_corners.csv"))

    if not files:
        raise FileNotFoundError("No lab_corners.csv found. Run collect_lab_corners.py first.")

    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def read_corners(path):
    corners = []

    with open(path, "r") as f:
        reader = csv.DictReader(f)

        for row in reader:
            corners.append({
                "point_id": int(row["point_id"]),
                "label": row["label"],
                "x": float(row["model_x"]),
                "y": float(row["model_y"]),
                "z": float(row["model_z"]),
                "yaw": float(row["model_yaw"]),
                "x_std": float(row["model_x_std"]),
                "y_std": float(row["model_y_std"]),
                "z_std": float(row["model_z_std"]),
                "n_samples": int(row["n_samples"]),
            })

    corners.sort(key=lambda p: p["point_id"])
    return corners


def polygon_area(points):
    area = 0.0
    n = len(points)

    for i in range(n):
        j = (i + 1) % n
        area += points[i]["x"] * points[j]["y"]
        area -= points[j]["x"] * points[i]["y"]

    return abs(area) * 0.5


def polygon_signed_area(points):
    area = 0.0
    n = len(points)

    for i in range(n):
        j = (i + 1) % n
        area += points[i]["x"] * points[j]["y"]
        area -= points[j]["x"] * points[i]["y"]

    return area * 0.5


def polygon_perimeter(points):
    p = 0.0
    n = len(points)

    for i in range(n):
        j = (i + 1) % n
        dx = points[j]["x"] - points[i]["x"]
        dy = points[j]["y"] - points[i]["y"]
        p += math.sqrt(dx * dx + dy * dy)

    return p


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


def centroid(points):
    signed_area = polygon_signed_area(points)

    if abs(signed_area) < 1e-12:
        x = sum(p["x"] for p in points) / len(points)
        y = sum(p["y"] for p in points) / len(points)
        return x, y

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


def main():
    source_csv = latest_corners_csv()
    source_session = Path(source_csv).parent.name

    points = read_corners(source_csv)

    if len(points) < 3:
        raise RuntimeError("At least 3 corner points are required to create a map polygon.")

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

    cx, cy = centroid(points)

    area = polygon_area(points)
    signed_area = polygon_signed_area(points)
    perimeter = polygon_perimeter(points)
    edges = edge_lengths(points)

    orientation = "counter_clockwise" if signed_area > 0 else "clockwise"

    map_data = {
        "created_unix_time": stamp,
        "source_lab_corners_csv": source_csv,
        "source_session": source_session,

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
            "center_x": (x_min + x_max) * 0.5,
            "center_y": (y_min + y_max) * 0.5,
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
            "Boundary vertices are stored in the order they were recorded.",
            "Use boundary_polygon for map constraints.",
            "Use bounds for simple rectangular limit checks.",
            "Coordinates are already corrected to the TurtleBot model center using mocap calibration."
        ]
    }

    json_path = os.path.join(save_dir, "lab_map.json")
    vertex_csv_path = os.path.join(save_dir, "lab_map_vertices.csv")
    edge_csv_path = os.path.join(save_dir, "lab_map_edges.csv")
    summary_path = os.path.join(save_dir, "lab_map_summary.md")

    with open(json_path, "w") as f:
        json.dump(map_data, f, indent=2, ensure_ascii=False)

    with open(vertex_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "point_id",
                "label",
                "x",
                "y",
                "z",
                "yaw",
                "x_std",
                "y_std",
                "z_std",
                "n_samples",
            ]
        )
        writer.writeheader()
        writer.writerows(points)

    with open(edge_csv_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "edge_id",
                "from",
                "to",
                "from_point_id",
                "to_point_id",
                "dx",
                "dy",
                "length",
                "heading",
            ]
        )
        writer.writeheader()
        writer.writerows(edges)

    with open(summary_path, "w") as f:
        f.write("# A105 실험실 맵 데이터\n\n")

        f.write("## Source\n\n")
        f.write(f"- source csv: `{source_csv}`\n")
        f.write(f"- source session: `{source_session}`\n\n")

        f.write("## Coordinate convention\n\n")
        f.write("- 좌표계: mocap world frame\n")
        f.write("- 기준점: TurtleBot model center\n")
        f.write("- 각 코너 좌표는 보정된 모델 중심 위치 기준이다.\n\n")

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
        f.write("| point_id | label | x [m] | y [m] | std_x [m] | std_y [m] |\n")
        f.write("|---:|---|---:|---:|---:|---:|\n")

        for p in points:
            f.write(
                f"| {p['point_id']} | {p['label']} | "
                f"{p['x']:.6f} | {p['y']:.6f} | "
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

    print("Lab map created.")
    print("save dir:", save_dir)
    print("json:", json_path)
    print("vertices csv:", vertex_csv_path)
    print("edges csv:", edge_csv_path)
    print("summary:", summary_path)
    print("")
    print("Bounds:")
    print("  x_min:", x_min)
    print("  x_max:", x_max)
    print("  y_min:", y_min)
    print("  y_max:", y_max)
    print("  width_x:", width_x)
    print("  height_y:", height_y)
    print("")
    print("Area:", area)
    print("Perimeter:", perimeter)


if __name__ == "__main__":
    main()
