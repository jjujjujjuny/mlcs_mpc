#!/usr/bin/env python3

import os
import json
import time
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
OLD_TRACK_JSON = "/home/user/Desktop/map_data/track_data/latest_track_waypoints.json"
LINEAR_TRACK_JSON = "/home/user/Desktop/map_data/track_linear_data/latest_track_linear_data.json"

SAVE_ROOT = "/home/user/Desktop/map_data/track_data"


def load_json(path):
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def try_extract_xy_list(obj, candidate_keys_list):
    """
    candidate_keys_list:
      [
        ["track", "centerline"],
        ["centerline"],
        ...
      ]
    """
    for keys in candidate_keys_list:
        cur = obj
        ok = True
        for k in keys:
            if isinstance(cur, dict) and k in cur:
                cur = cur[k]
            else:
                ok = False
                break
        if ok:
            pts = normalize_points(cur)
            if pts is not None and len(pts) > 0:
                return pts
    return None


def normalize_points(data):
    """
    data could be:
      - [{"x":..., "y":...}, ...]
      - [[x,y], [x,y], ...]
      - {"points":[...]}
    """
    if data is None:
        return None

    if isinstance(data, dict):
        if "points" in data:
            return normalize_points(data["points"])
        return None

    if isinstance(data, list):
        pts = []
        for p in data:
            if isinstance(p, dict) and "x" in p and "y" in p:
                pts.append([float(p["x"]), float(p["y"])])
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                pts.append([float(p[0]), float(p[1])])
            else:
                return None
        if len(pts) == 0:
            return None
        return np.array(pts, dtype=float)

    return None


class RedrawTrack:
    def __init__(self):
        self.points = []

        boundary_data = load_json(BOUNDARY_JSON)
        if boundary_data is None:
            raise FileNotFoundError(f"boundary json not found: {BOUNDARY_JSON}")

        self.boundary_data = boundary_data
        vertices = self.boundary_data["boundary"]["vertices"]
        self.boundary = np.array([[v["x"], v["y"]] for v in vertices], dtype=float)
        self.boundary_closed = np.vstack([self.boundary, self.boundary[0]])

        self.old_waypoints = None
        old_track = load_json(OLD_TRACK_JSON)
        if old_track is not None:
            self.old_waypoints = try_extract_xy_list(
                old_track,
                [
                    ["track", "waypoints"],
                    ["waypoints"],
                ]
            )

        self.centerline = None
        self.left_boundary = None
        self.right_boundary = None

        linear_track = load_json(LINEAR_TRACK_JSON)
        if linear_track is not None:
            self.centerline = try_extract_xy_list(
                linear_track,
                [
                    ["track", "centerline"],
                    ["centerline"],
                    ["center_line"],
                ]
            )
            self.left_boundary = try_extract_xy_list(
                linear_track,
                [
                    ["track", "left_boundary"],
                    ["left_boundary"],
                    ["left_bound"],
                ]
            )
            self.right_boundary = try_extract_xy_list(
                linear_track,
                [
                    ["track", "right_boundary"],
                    ["right_boundary"],
                    ["right_bound"],
                ]
            )

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.cid_click = self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.cid_key = self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        self.draw()

    def draw_polyline(self, pts, label, style="-", close_loop=False, alpha=1.0, linewidth=2.0):
        if pts is None or len(pts) == 0:
            return
        arr = pts
        if close_loop:
            arr = np.vstack([pts, pts[0]])
        self.ax.plot(
            arr[:, 0], arr[:, 1],
            style,
            linewidth=linewidth,
            alpha=alpha,
            label=label
        )

    def draw(self):
        self.ax.clear()

        # map boundary
        self.ax.plot(
            self.boundary_closed[:, 0],
            self.boundary_closed[:, 1],
            "-",
            linewidth=2.5,
            label="map boundary"
        )

        # existing linear track references
        self.draw_polyline(self.left_boundary, "current left boundary", style="-", alpha=0.55, linewidth=2.0)
        self.draw_polyline(self.right_boundary, "current right boundary", style="-", alpha=0.55, linewidth=2.0)
        self.draw_polyline(self.centerline, "current centerline", style="--", alpha=0.9, linewidth=2.2)

        # old clicked waypoints
        if self.old_waypoints is not None and len(self.old_waypoints) > 0:
            old_closed = np.vstack([self.old_waypoints, self.old_waypoints[0]])
            self.ax.plot(
                old_closed[:, 0],
                old_closed[:, 1],
                "o--",
                linewidth=1.3,
                markersize=4,
                alpha=0.45,
                label="old clicked waypoints"
            )

        # new points being clicked now
        if len(self.points) > 0:
            pts = np.array(self.points, dtype=float)
            self.ax.plot(
                pts[:, 0],
                pts[:, 1],
                "o-",
                linewidth=2.8,
                markersize=5,
                label="new clicked waypoints"
            )
            for i, p in enumerate(pts):
                self.ax.text(p[0], p[1], f"W{i}", fontsize=9)

        self.ax.set_title(
            "Redraw track with references\n"
            "left click:add | u:undo | c:clear | s:save | q:quit"
        )
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.axis("equal")
        self.ax.grid(True)
        self.ax.legend(loc="best")
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        if event.button == 1:
            x = float(event.xdata)
            y = float(event.ydata)
            self.points.append([x, y])
            print(f"added W{len(self.points)-1}: x={x:.4f}, y={y:.4f}")
            self.draw()

    def on_key(self, event):
        if event.key == "u":
            if self.points:
                p = self.points.pop()
                print(f"undo: x={p[0]:.4f}, y={p[1]:.4f}")
                self.draw()

        elif event.key == "c":
            self.points = []
            print("cleared all new points")
            self.draw()

        elif event.key == "s":
            self.save()

        elif event.key == "q":
            plt.close(self.fig)

    def save(self):
        if len(self.points) < 3:
            print("waypoint가 3개 미만이라 저장 안 함")
            return

        stamp = int(time.time())
        save_dir = Path(SAVE_ROOT) / f"track_{stamp}"
        save_dir.mkdir(parents=True, exist_ok=True)

        pts = np.array(self.points, dtype=float)

        csv_path = save_dir / "track_waypoints.csv"
        json_path = save_dir / "track_waypoints.json"
        png_path = save_dir / "track_preview.png"

        latest_csv = Path(SAVE_ROOT) / "latest_track_waypoints.csv"
        latest_json = Path(SAVE_ROOT) / "latest_track_waypoints.json"
        latest_png = Path(SAVE_ROOT) / "latest_track_preview.png"

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["waypoint_id", "x", "y"])
            for i, p in enumerate(self.points):
                writer.writerow([i, float(p[0]), float(p[1])])

        with open(latest_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["waypoint_id", "x", "y"])
            for i, p in enumerate(self.points):
                writer.writerow([i, float(p[0]), float(p[1])])

        out = {
            "created_unix_time": stamp,
            "source_boundary_json": BOUNDARY_JSON,
            "source_linear_track_json": LINEAR_TRACK_JSON if os.path.exists(LINEAR_TRACK_JSON) else None,
            "coordinate_frame": self.boundary_data.get("coordinate_frame", {}),
            "track": {
                "closed": True,
                "num_waypoints": len(self.points),
                "waypoints": [
                    {"id": i, "x": float(p[0]), "y": float(p[1])}
                    for i, p in enumerate(self.points)
                ]
            },
            "notes": [
                "Redrawn manually while referencing boundary and current centerline.",
                "These waypoints are centerline control points."
            ]
        }

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)

        with open(latest_json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(
            self.boundary_closed[:, 0],
            self.boundary_closed[:, 1],
            "-",
            linewidth=2.5,
            label="map boundary"
        )

        if self.left_boundary is not None:
            ax.plot(self.left_boundary[:, 0], self.left_boundary[:, 1], "-", linewidth=2.0, alpha=0.55, label="current left boundary")
        if self.right_boundary is not None:
            ax.plot(self.right_boundary[:, 0], self.right_boundary[:, 1], "-", linewidth=2.0, alpha=0.55, label="current right boundary")
        if self.centerline is not None:
            ax.plot(self.centerline[:, 0], self.centerline[:, 1], "--", linewidth=2.2, alpha=0.9, label="current centerline")

        pts_closed = np.vstack([pts, pts[0]])
        ax.plot(
            pts_closed[:, 0],
            pts_closed[:, 1],
            "o-",
            linewidth=2.8,
            markersize=5,
            label="new clicked waypoints"
        )

        for i, p in enumerate(pts):
            ax.text(p[0], p[1], f"W{i}", fontsize=9)

        ax.set_title("Redrawn track waypoints with references")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True)
        ax.legend(loc="best")

        fig.savefig(png_path, dpi=200, bbox_inches="tight")
        fig.savefig(latest_png, dpi=200, bbox_inches="tight")
        plt.close(fig)

        print("")
        print("saved:")
        print("  dir :", save_dir)
        print("  csv :", csv_path)
        print("  json:", json_path)
        print("  png :", png_path)
        print("")
        print("updated latest:")
        print("  ", latest_csv)
        print("  ", latest_json)
        print("  ", latest_png)


def main():
    drawer = RedrawTrack()
    plt.show()


if __name__ == "__main__":
    main()
