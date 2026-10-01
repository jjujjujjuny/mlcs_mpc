#!/usr/bin/env python3

import sys
from pathlib import Path

CODE_DIR = Path("/home/user/Desktop/turtlebot_data/code")
MAIN_PC_DIR = CODE_DIR / "main_pc"

for p in [CODE_DIR, MAIN_PC_DIR]:
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import os
import csv
import json
import time
import math
import rospy
import numpy as np

from mocap_api import MocapAPI


SAVE_ROOT = "/home/user/Desktop/turtlebot_data/experiments/lab_boundary"
MOCAP_TOPIC = "/natnet_ros/RigidBody/pose"

SAMPLE_SEC = 1.0
RATE_HZ = 100


def circular_mean_angle(angles):
    angles = np.asarray(angles)
    s = np.mean(np.sin(angles))
    c = np.mean(np.cos(angles))
    return math.atan2(s, c)


def collect_one_corner(mocap, sample_sec):
    samples = []
    start_wall = rospy.Time.now().to_sec()
    last_stamp = None

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()

        if now - start_wall >= sample_sec:
            break

        data = mocap.latest_data

        if data is None:
            rospy.sleep(0.001)
            continue

        stamp = data["mocap_stamp"]

        if last_stamp is not None and stamp == last_stamp:
            rospy.sleep(0.001)
            continue

        last_stamp = stamp

        model = data["model"]
        raw = data["raw"]

        samples.append({
            "wall_time": now,
            "mocap_stamp": stamp,

            "model_x": model["position"]["x"],
            "model_y": model["position"]["y"],
            "model_z": model["position"]["z"],
            "model_yaw": model["yaw"],

            "raw_x": raw["position"]["x"],
            "raw_y": raw["position"]["y"],
            "raw_z": raw["position"]["z"],
            "raw_yaw": raw["yaw"],
        })

    return samples


def summarize_samples(point_id, label, samples):
    xs = np.array([s["model_x"] for s in samples])
    ys = np.array([s["model_y"] for s in samples])
    zs = np.array([s["model_z"] for s in samples])
    yaws = np.array([s["model_yaw"] for s in samples])

    raw_xs = np.array([s["raw_x"] for s in samples])
    raw_ys = np.array([s["raw_y"] for s in samples])
    raw_zs = np.array([s["raw_z"] for s in samples])

    return {
        "point_id": point_id,
        "label": label,
        "n_samples": int(len(samples)),

        "model_x": float(np.mean(xs)),
        "model_y": float(np.mean(ys)),
        "model_z": float(np.mean(zs)),
        "model_yaw": float(circular_mean_angle(yaws)),

        "model_x_std": float(np.std(xs)),
        "model_y_std": float(np.std(ys)),
        "model_z_std": float(np.std(zs)),

        "raw_x": float(np.mean(raw_xs)),
        "raw_y": float(np.mean(raw_ys)),
        "raw_z": float(np.mean(raw_zs)),

        "raw_x_std": float(np.std(raw_xs)),
        "raw_y_std": float(np.std(raw_ys)),
        "raw_z_std": float(np.std(raw_zs)),
    }


def main():
    rospy.init_node("a105_collect_lab_corners", anonymous=True)

    stamp = int(time.time())
    session_dir = os.path.join(SAVE_ROOT, f"session_{stamp}")
    os.makedirs(session_dir, exist_ok=True)

    corners_csv = os.path.join(session_dir, "lab_corners.csv")
    corners_json = os.path.join(session_dir, "lab_corners.json")
    raw_csv = os.path.join(session_dir, "raw_corner_samples.csv")

    mocap = MocapAPI(
        topic=MOCAP_TOPIC,
        rate_hz=RATE_HZ
    )

    print("Waiting for mocap data...")
    while not rospy.is_shutdown() and mocap.latest_data is None:
        rospy.sleep(0.01)

    print("Mocap data received.")
    print("Session dir:", session_dir)
    print("")
    print("사용 방법:")
    print("  1. 터틀봇을 실험실 경계 코너 위치에 놓기")
    print("  2. 1초 이상 가만히 두기")
    print("  3. Enter 누르면 1초 동안 평균 위치 저장")
    print("  4. 모든 코너를 시계방향 또는 반시계방향으로 찍기")
    print("  5. q 입력 후 Enter로 종료")
    print("")

    corner_rows = []
    raw_rows = []
    point_id = 0

    while not rospy.is_shutdown():
        cmd = input(f"[point {point_id}] 코너에 둔 뒤 Enter 저장, q 종료: ").strip()

        if cmd.lower() in ["q", "quit", "exit"]:
            break

        label = cmd
        if label == "":
            label = f"corner_{point_id}"

        print(f"  collecting {SAMPLE_SEC:.1f} sec at {label} ...")

        samples = collect_one_corner(mocap, SAMPLE_SEC)

        if len(samples) == 0:
            print("  no samples collected")
            continue

        summary = summarize_samples(point_id, label, samples)
        corner_rows.append(summary)

        for k, s in enumerate(samples):
            row = dict(s)
            row["point_id"] = point_id
            row["label"] = label
            row["sample_index"] = k
            raw_rows.append(row)

        print(
            f"  saved {label}: "
            f"x={summary['model_x']:+.4f}, "
            f"y={summary['model_y']:+.4f}, "
            f"std=({summary['model_x_std']:.5f}, {summary['model_y_std']:.5f}), "
            f"n={summary['n_samples']}"
        )

        point_id += 1

    with open(corners_csv, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "point_id",
                "label",
                "n_samples",

                "model_x",
                "model_y",
                "model_z",
                "model_yaw",

                "model_x_std",
                "model_y_std",
                "model_z_std",

                "raw_x",
                "raw_y",
                "raw_z",

                "raw_x_std",
                "raw_y_std",
                "raw_z_std",
            ]
        )
        writer.writeheader()
        writer.writerows(corner_rows)

    with open(raw_csv, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "point_id",
                "label",
                "sample_index",

                "wall_time",
                "mocap_stamp",

                "model_x",
                "model_y",
                "model_z",
                "model_yaw",

                "raw_x",
                "raw_y",
                "raw_z",
                "raw_yaw",
            ]
        )
        writer.writeheader()
        writer.writerows(raw_rows)

    polygon = [
        {
            "point_id": r["point_id"],
            "label": r["label"],
            "x": r["model_x"],
            "y": r["model_y"],
            "z": r["model_z"],
        }
        for r in corner_rows
    ]

    result = {
        "created_unix_time": stamp,
        "session_dir": session_dir,
        "mocap_topic": MOCAP_TOPIC,
        "sample_sec_per_corner": SAMPLE_SEC,
        "coordinate": "model center in mocap world frame",
        "calibration_used": mocap.calib.as_dict(),
        "note": "Corners should be interpreted in recorded order. Use clockwise or counter-clockwise order.",
        "polygon": polygon,
        "files": {
            "corners_csv": corners_csv,
            "raw_samples_csv": raw_csv,
            "corners_json": corners_json,
        },
    }

    with open(corners_json, "w") as f:
        json.dump(result, f, indent=2)

    print("")
    print("Finished.")
    print("corners csv:", corners_csv)
    print("raw samples:", raw_csv)
    print("json:", corners_json)


if __name__ == "__main__":
    main()
