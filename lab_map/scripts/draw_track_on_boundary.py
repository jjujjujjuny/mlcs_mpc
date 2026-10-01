#!/usr/bin/env python3

import os
import json
import time
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
SAVE_ROOT = "/home/user/Desktop/map_data/track_data"


class TrackDrawer:
    def __init__(self, boundary_json):
        self.boundary_json = boundary_json
        self.points = []

        with open(boundary_json, "r", encoding="utf-8") as f:
            self.boundary_data = json.load(f)

        vertices = self.boundary_data["boundary"]["vertices"]
        self.boundary = np.array([[v["x"], v["y"]] for v in vertices], dtype=float)
        self.boundary_closed = np.vstack([self.boundary, self.boundary[0]])

        self.fig, self.ax = plt.subplots(figsize=(9, 9))

        self.boundary_line = None
        self.track_line = None
        self.point_scatter = None

        self.draw_base()

        self.cid_click = self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.cid_key = self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def draw_base(self):
        self.ax.clear()

        self.ax.plot(
            self.boundary_closed[:, 0],
            self.boundary_closed[:, 1],
            "-o",
            linewidth=2.5,
            markersize=4,
            label="map boundary"
        )

        for i, p in enumerate(self.boundary):
            self.ax.text(p[0], p[1], str(i), fontsize=8)

        self.ax.set_title(
            "Draw track on map boundary\n"
            "left click: add point | u: undo | c: clear | s: save | q: quit"
        )
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.axis("equal")
        self.ax.grid(True)

        self.update_track_plot()

    def update_track_plot(self):
        # 기존 track artist 제거
        if self.track_line is not None:
            self.track_line.remove()
            self.track_line = None

        if self.point_scatter is not None:
            self.point_scatter.remove()
            self.point_scatter = None

        if len(self.points) > 0:
            pts = np.array(self.points)

            self.track_line, = self.ax.plot(
                pts[:, 0],
                pts[:, 1],
                "-",
                linewidth=2.5,
                label="drawn track"
            )

            self.point_scatter = self.ax.scatter(
                pts[:, 0],
                pts[:, 1],
                s=45,
                zorder=5,
                label="track waypoints"
            )

            for i, p in enumerate(pts):
                self.ax.text(p[0], p[1], f"T{i}", fontsize=8)

        self.ax.legend(loc="best")
        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax:
            return

        if event.button == 1:
            x = float(event.xdata)
            y = float(event.ydata)

            self.points.append([x, y])
            print(f"added point {len(self.points)-1}: x={x:.4f}, y={y:.4f}")

            self.draw_base()

    def on_key(self, event):
        if event.key == "u":
            if self.points:
                removed = self.points.pop()
                print(f"undo: x={removed[0]:.4f}, y={removed[1]:.4f}")
                self.draw_base()

        elif event.key == "c":
            self.points = []
            print("cleared all track points")
            self.draw_base()

        elif event.key == "s":
            self.save()

        elif event.key == "q":
            print("quit")
            plt.close(self.fig)

    def save(self):
        if len(self.points) < 2:
            print("track point가 2개 미만이라 저장하지 않았어.")
            return

        stamp = int(time.time())
        save_dir = Path(SAVE_ROOT) / f"track_{stamp}"
        save_dir.mkdir(parents=True, exist_ok=True)

        pts = np.array(self.points)

        csv_path = save_dir / "track_waypoints.csv"
        json_path = save_dir / "track_waypoints.json"
        png_path = save_dir / "track_preview.png"

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["waypoint_id", "x", "y"])
            for i, p in enumerate(self.points):
                writer.writerow([i, p[0], p[1]])

        track_data = {
            "created_unix_time": stamp,
            "source_boundary_json": self.boundary_json,
            "coordinate_frame": self.boundary_data.get("coordinate_frame", {}),
            "track": {
                "closed": False,
                "num_waypoints": len(self.points),
                "waypoints": [
                    {
                        "id": i,
                        "x": float(p[0]),
                        "y": float(p[1])
                    }
                    for i, p in enumerate(self.points)
                ]
            },
            "notes": [
                "Track waypoints were drawn manually on top of the map boundary.",
                "Coordinates are in the same frame as map_boundary_data.json.",
                "Use these waypoints as centerline/reference path candidates."
            ]
        }

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(track_data, f, ensure_ascii=False, indent=2)

        # 저장용 preview
        fig, ax = plt.subplots(figsize=(9, 9))

        ax.plot(
            self.boundary_closed[:, 0],
            self.boundary_closed[:, 1],
            "-o",
            linewidth=2.0,
            markersize=4,
            label="map boundary"
        )

        ax.plot(
            pts[:, 0],
            pts[:, 1],
            "-o",
            linewidth=2.8,
            markersize=5,
            label="manual track"
        )

        for i, p in enumerate(pts):
            ax.text(p[0], p[1], f"T{i}", fontsize=8)

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("Manual track on map boundary")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
        fig.savefig(png_path, dpi=200, bbox_inches="tight")
        plt.close(fig)

        latest_json = Path(SAVE_ROOT) / "latest_track_waypoints.json"
        latest_csv = Path(SAVE_ROOT) / "latest_track_waypoints.csv"
        latest_png = Path(SAVE_ROOT) / "latest_track_preview.png"

        with open(latest_json, "w", encoding="utf-8") as f:
            json.dump(track_data, f, ensure_ascii=False, indent=2)

        with open(latest_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["waypoint_id", "x", "y"])
            for i, p in enumerate(self.points):
                writer.writerow([i, p[0], p[1]])

        # latest png도 복사 대신 다시 저장
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.plot(self.boundary_closed[:, 0], self.boundary_closed[:, 1], "-o", linewidth=2.0, markersize=4, label="map boundary")
        ax.plot(pts[:, 0], pts[:, 1], "-o", linewidth=2.8, markersize=5, label="manual track")
        for i, p in enumerate(pts):
            ax.text(p[0], p[1], f"T{i}", fontsize=8)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("Manual track on map boundary")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
        fig.savefig(latest_png, dpi=200, bbox_inches="tight")
        plt.close(fig)

        print("")
        print("saved track:")
        print("  dir :", save_dir)
        print("  csv :", csv_path)
        print("  json:", json_path)
        print("  png :", png_path)
        print("")
        print("latest:")
        print("  ", latest_json)
        print("  ", latest_csv)
        print("  ", latest_png)


def main():
    if not os.path.exists(BOUNDARY_JSON):
        raise FileNotFoundError(
            f"boundary json not found: {BOUNDARY_JSON}\n"
            "먼저 map_boundary_data.json을 만들어야 해."
        )

    drawer = TrackDrawer(BOUNDARY_JSON)
    plt.show()


if __name__ == "__main__":
    main()
