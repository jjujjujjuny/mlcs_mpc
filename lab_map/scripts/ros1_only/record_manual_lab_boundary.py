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
import rospy

from mocap_api import MocapAPI


SAVE_ROOT = "/home/user/Desktop/turtlebot_data/experiments/lab_boundary_manual"
MOCAP_TOPIC = "/natnet_ros/RigidBody/pose"
RATE_HZ = 100


def main():
    rospy.init_node("a105_record_manual_lab_boundary", anonymous=True)

    stamp = int(time.time())
    session_dir = os.path.join(SAVE_ROOT, f"session_{stamp}")
    os.makedirs(session_dir, exist_ok=True)

    csv_path = os.path.join(session_dir, "manual_lab_boundary_motion.csv")
    info_path = os.path.join(session_dir, "session_info.json")

    mocap = MocapAPI(
        topic=MOCAP_TOPIC,
        rate_hz=RATE_HZ
    )

    print("")
    print("Mocap calibration info:")
    print(mocap.calib.as_dict())

    if not mocap.calib.loaded:
        raise RuntimeError(
            "Mocap calibration JSON was not loaded. "
            "Run calibration first."
        )

    print("")
    print("Waiting for mocap data...")
    while not rospy.is_shutdown() and mocap.latest_data is None:
        rospy.sleep(0.01)

    print("Mocap data received.")
    print("Recording manual lab boundary motion.")
    print("Session dir:", session_dir)
    print("")
    print("사용 방법:")
    print("  1. 다른 터미널에서 키보드 조종 실행")
    print("  2. 터틀봇을 경계 코너로 직접 이동")
    print("  3. 코너에서 1초 이상 정지")
    print("  4. 다음 코너로 이동 후 다시 정지")
    print("  5. 모든 코너를 찍었으면 이 터미널에서 Ctrl+C")
    print("")
    print("기록 시작.")

    time_zero_abs = mocap.latest_data["mocap_stamp"]
    last_logged_stamp = None

    rows = 0

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "time",
            "dt",

            "raw_mocap_x",
            "raw_mocap_y",
            "raw_mocap_z",
            "raw_mocap_yaw",

            "model_x",
            "model_y",
            "model_z",
            "model_yaw",

            "model_yaw_rate",
            "model_vx",
            "model_vy",
            "model_vz",
            "model_speed",
            "model_v_forward",
            "model_v_lateral",
            "model_ax",
            "model_ay",
            "model_az",
        ])

        try:
            while not rospy.is_shutdown():
                data = mocap.latest_data

                if data is None:
                    rospy.sleep(0.001)
                    continue

                stamp_abs = data["mocap_stamp"]

                if last_logged_stamp is not None and stamp_abs == last_logged_stamp:
                    rospy.sleep(0.001)
                    continue

                if last_logged_stamp is None:
                    dt = 0.0
                else:
                    dt = stamp_abs - last_logged_stamp

                last_logged_stamp = stamp_abs
                t = stamp_abs - time_zero_abs

                raw = data["raw"]
                model = data["model"]

                writer.writerow([
                    t,
                    dt,

                    raw["position"]["x"],
                    raw["position"]["y"],
                    raw["position"]["z"],
                    raw["yaw"],

                    model["position"]["x"],
                    model["position"]["y"],
                    model["position"]["z"],
                    model["yaw"],

                    model["yaw_rate"],
                    model["velocity_world"]["x"],
                    model["velocity_world"]["y"],
                    model["velocity_world"]["z"],
                    model["speed"],
                    model["velocity_body"]["forward"],
                    model["velocity_body"]["lateral"],
                    model["acceleration_world"]["x"],
                    model["acceleration_world"]["y"],
                    model["acceleration_world"]["z"],
                ])

                rows += 1

                if rows % 100 == 0:
                    print(
                        f"t={t:7.2f}s, "
                        f"x={model['position']['x']:+.3f}, "
                        f"y={model['position']['y']:+.3f}, "
                        f"speed={model['speed']:.4f}"
                    )

        except KeyboardInterrupt:
            print("")
            print("Recording stopped by Ctrl+C.")

    info = {
        "created_unix_time": stamp,
        "csv_path": csv_path,
        "session_dir": session_dir,
        "mocap_topic": MOCAP_TOPIC,
        "coordinate": "model center in mocap world frame",
        "calibration_used": mocap.calib.as_dict(),
        "note": "User manually drove TurtleBot and stopped at boundary/corner points.",
        "rows": rows,
    }

    with open(info_path, "w") as f:
        json.dump(info, f, indent=2)

    print("")
    print("Finished.")
    print("CSV:", csv_path)
    print("Info:", info_path)
    print("Rows:", rows)


if __name__ == "__main__":
    main()
