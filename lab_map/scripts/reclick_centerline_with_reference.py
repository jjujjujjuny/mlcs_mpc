#!/usr/bin/env python3
import os
import json
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

MAP_BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
REFERENCE_JSON_CANDIDATES = [
    "/home/user/Desktop/map_data/track_adaptive_data/latest_adaptive_track.json",
    "/home/user/Desktop/map_data/track_data/latest_track_data.json",
    "/home/user/Desktop/map_data/track_data/latest_track_waypoints.json",
]
SAVE_DIR = "/home/user/Desktop/map_data/track_reclick_data"


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


def load_boundary():
    data = load_json(MAP_BOUNDARY_JSON)
    verts = data["boundary"]["vertices"]
    pts = np.array([[float(v["x"]), float(v["y"])] for v in verts], dtype=float)
    return pts


def load_reference_centerline():
    for path in REFERENCE_JSON_CANDIDATES:
        if not os.path.exists(path):
            continue

        data = load_json(path)

        # adaptive / track json 안에 centerline이 있는 경우
        if "track" in data and "centerline" in data["track"]:
            arr = np.array(
                [[float(p["x"]), float(p["y"])] for p in data["track"]["centerline"]],
                dtype=float
            )
            return arr, path

        # waypoint만 저장된 경우
        if "track" in data and "waypoints" in data["track"]:
            arr = np.array(
                [[float(p["x"]), float(p["y"])] for p in data["track"]["waypoints"]],
                dtype=float
            )
            return arr, path

    return None, None


def catmull_rom_closed(points, samples_per_seg=30):
    pts = np.asarray(points, dtype=float)
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
                (2.0 * p1)
                + (-p0 + p2) * t
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
            )
            curve.append(p)

    return np.array(curve, dtype=float)


def save_outputs(clicked_points, smooth_centerline):
    os.makedirs(SAVE_DIR, exist_ok=True)
    stamp = int(time.time())
    session_dir = os.path.join(SAVE_DIR, f"session_{stamp}")
    os.makedirs(session_dir, exist_ok=True)

    clicked_points = np.asarray(clicked_points, dtype=float)
    smooth_centerline = np.asarray(smooth_centerline, dtype=float)

    clicked_csv = os.path.join(session_dir, "clicked_centerline_waypoints.csv")
    smooth_csv = os.path.join(session_dir, "smooth_centerline.csv")
    preview_png = os.path.join(session_dir, "preview.png")
    json_path = os.path.join(session_dir, "reclicked_centerline.json")

    pd.DataFrame({
        "idx": np.arange(len(clicked_points)),
        "x": clicked_points[:, 0],
        "y": clicked_points[:, 1],
    }).to_csv(clicked_csv, index=False)

    pd.DataFrame({
        "idx": np.arange(len(smooth_centerline)),
        "x": smooth_centerline[:, 0],
        "y": smooth_centerline[:, 1],
    }).to_csv(smooth_csv, index=False)

    data = {
        "created_unix_time": stamp,
        "source": "manual_reclick_centerline",
        "track": {
            "waypoints": [
                {"id": int(i), "x": float(p[0]), "y": float(p[1])}
                for i, p in enumerate(clicked_points)
            ],
            "centerline": [
                {"id": int(i), "x": float(p[0]), "y": float(p[1])}
                for i, p in enumerate(smooth_centerline)
            ],
        }
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    latest_json = os.path.join(SAVE_DIR, "latest_reclicked_centerline.json")
    latest_clicked_csv = os.path.join(SAVE_DIR, "latest_clicked_centerline_waypoints.csv")
    latest_smooth_csv = os.path.join(SAVE_DIR, "latest_smooth_centerline.csv")
    latest_preview = os.path.join(SAVE_DIR, "latest_reclicked_centerline_preview.png")

    with open(latest_json, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    pd.DataFrame({
        "idx": np.arange(len(clicked_points)),
        "x": clicked_points[:, 0],
        "y": clicked_points[:, 1],
    }).to_csv(latest_clicked_csv, index=False)

    pd.DataFrame({
        "idx": np.arange(len(smooth_centerline)),
        "x": smooth_centerline[:, 0],
        "y": smooth_centerline[:, 1],
    }).to_csv(latest_smooth_csv, index=False)

    return {
        "session_dir": session_dir,
        "clicked_csv": clicked_csv,
        "smooth_csv": smooth_csv,
        "json_path": json_path,
        "latest_json": latest_json,
        "latest_clicked_csv": latest_clicked_csv,
        "latest_smooth_csv": latest_smooth_csv,
        "latest_preview": latest_preview,
        "preview_png": preview_png,
    }


def main():
    boundary = load_boundary()
    ref_centerline, ref_path = load_reference_centerline()

    clicked = []
    current_smooth = None
    save_info = {}

    fig, ax = plt.subplots(figsize=(10, 10))

    boundary_closed = close_ring(boundary)

    def redraw():
        ax.clear()

        # map boundary
        ax.plot(
            boundary_closed[:, 0], boundary_closed[:, 1],
            linewidth=2.5, label="map boundary"
        )

        # reference centerline
        if ref_centerline is not None and len(ref_centerline) > 1:
            rc = close_ring(ref_centerline)
            ax.plot(
                rc[:, 0], rc[:, 1],
                "--", linewidth=1.5, alpha=0.7, label="reference centerline"
            )

        # clicked points
        if len(clicked) > 0:
            cp = np.array(clicked, dtype=float)
            cpc = close_ring(cp)
            ax.plot(
                cpc[:, 0], cpc[:, 1],
                "o-", linewidth=1.5, markersize=5, label="clicked sparse waypoints"
            )

            for i, p in enumerate(cp):
                ax.text(p[0], p[1], f"W{i}", fontsize=10)

        # smoothed centerline preview
        if len(clicked) >= 3:
            cp = np.array(clicked, dtype=float)
            smooth = catmull_rom_closed(cp, samples_per_seg=30)
            if len(smooth) > 0:
                sc = close_ring(smooth)
                ax.plot(
                    sc[:, 0], sc[:, 1],
                    linewidth=3.0, label="new smooth centerline"
                )

        title = "Re-click centerline with reference"
        if ref_path is not None:
            title += f"\nreference: {os.path.basename(ref_path)}"
        ax.set_title(title)

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True)
        ax.legend(loc="best")

        help_text = (
            "Left click: add point | u: undo | c: clear | s or enter: save | q: quit\n"
            "가능하면 성기게 찍어라. 너무 촘촘하면 다시 곡률이 커진다."
        )
        ax.text(
            0.01, 0.01, help_text,
            transform=ax.transAxes, fontsize=10,
            bbox=dict(facecolor="white", alpha=0.7)
        )

        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes != ax:
            return
        if event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return

        clicked.append([event.xdata, event.ydata])
        print(f"added point {len(clicked)-1}: x={event.xdata:.3f}, y={event.ydata:.3f}")
        redraw()

    def on_key(event):
        nonlocal current_smooth, save_info

        if event.key == "u":
            if len(clicked) > 0:
                removed = clicked.pop()
                print(f"undo: removed x={removed[0]:.3f}, y={removed[1]:.3f}")
                redraw()

        elif event.key == "c":
            clicked.clear()
            print("cleared all clicked points")
            redraw()

        elif event.key in ["s", "enter"]:
            if len(clicked) < 3:
                print("at least 3 points are needed")
                return

            cp = np.array(clicked, dtype=float)
            current_smooth = catmull_rom_closed(cp, samples_per_seg=30)

            save_info = save_outputs(cp, current_smooth)

            # current figure also save
            plt.savefig(save_info["preview_png"], dpi=200, bbox_inches="tight")
            plt.savefig(save_info["latest_preview"], dpi=200, bbox_inches="tight")

            print("\nSaved.")
            for k, v in save_info.items():
                print(f"{k}: {v}")
            plt.close(fig)

        elif event.key == "q":
            print("quit without saving")
            plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)

    redraw()
    plt.show()


if __name__ == "__main__":
    main()
