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
import math
import argparse
import time

import rospy
import numpy as np
import pandas as pd

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Imu

from mocap_api import MocapAPI
from turtlebot_api import TurtleBotAPI
from track_path_api import TrackPath, wrap_angle


ROBOT_NAME = "turtlebot8"
MOCAP_TOPIC = "/natnet_ros/RigidBody/pose"

SAVE_ROOT = "/home/user/Desktop/turtlebot_data/experiments/pure_pursuit"

DEFAULT_CENTERLINE_CSV = "/home/user/Desktop/map_data/track_curvature_data/latest_centerline.csv"
DEFAULT_LEFT_CSV = "/home/user/Desktop/map_data/track_curvature_data/latest_left_boundary.csv"
DEFAULT_RIGHT_CSV = "/home/user/Desktop/map_data/track_curvature_data/latest_right_boundary.csv"
DEFAULT_MAP_BOUNDARY_CSV = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_vertices.csv"

MARKER_TOPIC = "/a105/pure_pursuit_markers"

CONTROL_DT = 0.05


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def make_bot(rate_hz):
    try:
        return TurtleBotAPI(
            robot_name=ROBOT_NAME,
            rate_hz=rate_hz,
            experiment_name="pure_pursuit_track",
            auto_log=False,
        )
    except TypeError:
        return TurtleBotAPI(
            robot_name=ROBOT_NAME,
            rate_hz=rate_hz,
        )


def color(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def point_xy(x, y, z=0.0):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def load_csv_xy(path):
    if not os.path.exists(path):
        return None
    df = pd.read_csv(path)

    if "x" not in df.columns or "y" not in df.columns:
        return None

    return df[["x", "y"]].to_numpy(dtype=float)


class PurePursuitVisualizer:
    def __init__(self, frame_id="map"):
        self.frame_id = frame_id
        self.pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1, latch=True)

        self.centerline = load_csv_xy(DEFAULT_CENTERLINE_CSV)
        self.left_boundary = load_csv_xy(DEFAULT_LEFT_CSV)
        self.right_boundary = load_csv_xy(DEFAULT_RIGHT_CSV)
        self.map_boundary = load_csv_xy(DEFAULT_MAP_BOUNDARY_CSV)

        self.robot_radius = 0.18
        self.robot_height = 0.08

    def _base_marker(self, marker_id, ns, marker_type):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = int(marker_id)
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _line_strip(self, marker_id, ns, pts, rgba, width=0.03, z=0.01, closed=True):
        if pts is None or len(pts) < 2:
            return None

        arr = np.asarray(pts, dtype=float)
        if closed:
            arr = np.vstack([arr, arr[0]])

        m = self._base_marker(marker_id, ns, Marker.LINE_STRIP)
        m.scale.x = float(width)
        m.color = rgba
        m.points = [point_xy(p[0], p[1], z) for p in arr]
        return m

    def _sphere(self, marker_id, ns, x, y, rgba, scale=0.12, z=0.08):
        m = self._base_marker(marker_id, ns, Marker.SPHERE)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color = rgba
        return m

    def _cylinder(self, marker_id, ns, x, y, rgba, radius=0.18, height=0.08, z=0.04):
        m = self._base_marker(marker_id, ns, Marker.CYLINDER)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.scale.x = 2.0 * radius
        m.scale.y = 2.0 * radius
        m.scale.z = height
        m.color = rgba
        return m

    def _arrow(self, marker_id, ns, x, y, yaw, rgba, length=0.45, z=0.13):
        m = self._base_marker(marker_id, ns, Marker.ARROW)

        x0 = float(x)
        y0 = float(y)
        x1 = x0 + length * math.cos(float(yaw))
        y1 = y0 + length * math.sin(float(yaw))

        m.points = [
            point_xy(x0, y0, z),
            point_xy(x1, y1, z),
        ]

        m.scale.x = 0.035
        m.scale.y = 0.08
        m.scale.z = 0.12
        m.color = rgba
        return m

    def _text(self, marker_id, ns, x, y, text, rgba, z=0.35, size=0.16):
        m = self._base_marker(marker_id, ns, Marker.TEXT_VIEW_FACING)
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.scale.z = float(size)
        m.color = rgba
        m.text = text
        return m

    def publish_static_track(self):
        markers = MarkerArray()

        lines = [
            self._line_strip(1, "map_boundary", self.map_boundary, color(0.6, 0.6, 0.6, 0.75), width=0.04, z=0.0, closed=True),
            self._line_strip(2, "track_centerline", self.centerline, color(0.0, 0.6, 1.0, 1.0), width=0.045, z=0.02, closed=True),
            self._line_strip(3, "track_left", self.left_boundary, color(0.1, 1.0, 0.1, 0.9), width=0.03, z=0.02, closed=True),
            self._line_strip(4, "track_right", self.right_boundary, color(1.0, 0.4, 0.1, 0.9), width=0.03, z=0.02, closed=True),
        ]

        for m in lines:
            if m is not None:
                markers.markers.append(m)

        if self.centerline is not None and len(self.centerline) > 0:
            x0, y0 = self.centerline[0]
            markers.markers.append(
                self._sphere(
                    40,
                    "track_start",
                    x0,
                    y0,
                    color(0.0, 1.0, 0.0, 1.0),
                    scale=0.16,
                    z=0.10,
                )
            )

        self.pub.publish(markers)


    def publish(self, x, y, yaw, near, pp, v_cmd, w_cmd):
        markers = MarkerArray()

        # static map / track
        lines = [
            self._line_strip(1, "map_boundary", self.map_boundary, color(0.6, 0.6, 0.6, 0.7), width=0.035, z=0.0, closed=True),
            self._line_strip(2, "track_centerline", self.centerline, color(0.0, 0.6, 1.0, 1.0), width=0.035, z=0.02, closed=True),
            self._line_strip(3, "track_left", self.left_boundary, color(0.1, 1.0, 0.1, 0.8), width=0.025, z=0.02, closed=True),
            self._line_strip(4, "track_right", self.right_boundary, color(1.0, 0.4, 0.1, 0.8), width=0.025, z=0.02, closed=True),
        ]

        for m in lines:
            if m is not None:
                markers.markers.append(m)

        # robot footprint and heading
        markers.markers.append(
            self._cylinder(
                10,
                "robot_footprint",
                x,
                y,
                color(1.0, 1.0, 0.0, 0.45),
                radius=self.robot_radius,
                height=self.robot_height,
            )
        )

        markers.markers.append(
            self._arrow(
                11,
                "robot_heading",
                x,
                y,
                yaw,
                color(1.0, 1.0, 0.0, 1.0),
                length=0.45,
            )
        )

        # nearest point and lookahead target
        markers.markers.append(
            self._sphere(
                20,
                "nearest_centerline",
                near["x_ref"],
                near["y_ref"],
                color(1.0, 0.0, 1.0, 1.0),
                scale=0.10,
            )
        )

        markers.markers.append(
            self._sphere(
                21,
                "lookahead_target",
                pp["target_x"],
                pp["target_y"],
                color(0.0, 1.0, 1.0, 1.0),
                scale=0.13,
            )
        )

        # line from robot to lookahead target
        target_line = np.array([
            [x, y],
            [pp["target_x"], pp["target_y"]],
        ])

        m_target_line = self._line_strip(
            22,
            "robot_to_target",
            target_line,
            color(0.0, 1.0, 1.0, 0.8),
            width=0.02,
            z=0.08,
            closed=False,
        )
        markers.markers.append(m_target_line)


        self.pub.publish(markers)


def get_model_pose(mocap):
    data = mocap.latest_data

    if data is None:
        return None

    model = data["model"]

    x = model["position"]["x"]
    y = model["position"]["y"]
    yaw = model["yaw"]

    return {
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
        "stamp": float(data.get("mocap_stamp", rospy.Time.now().to_sec())),
    }


def get_nested(data, keys, default=""):
    cur = data
    try:
        for k in keys:
            cur = cur[k]
        if cur is None:
            return default
        return cur
    except Exception:
        return default


def get_latest_robot_obs(bot):
    """
    TurtleBotAPI 내부 obs 이름이 버전마다 다를 수 있어서
    가능한 attribute를 순서대로 확인한다.
    없으면 None을 반환한다.
    """
    for name in [
        "latest_obs",
        "obs",
        "latest_data",
        "latest_robot_data",
        "data",
    ]:
        if hasattr(bot, name):
            val = getattr(bot, name)
            if val is not None:
                return val
    return None


def parse_robot_obs(obs):
    """
    /turtlebot8/obs Float64MultiArray 기준:
      0 time
      1 imu_stamp
      2 acc_x
      3 acc_y
      4 acc_z
      5 gyro_x
      6 gyro_y
      7 gyro_z
      8 quat_x
      9 quat_y
      10 quat_z
      11 quat_w
      12 latest_cmd_linear_x
      13 latest_cmd_angular_z
    """
    out = {
        "imu_stamp": "",
        "imu_acc_x": "",
        "imu_acc_y": "",
        "imu_acc_z": "",
        "imu_gyro_x": "",
        "imu_gyro_y": "",
        "imu_gyro_z": "",
        "imu_quat_x": "",
        "imu_quat_y": "",
        "imu_quat_z": "",
        "imu_quat_w": "",
        "lowlevel_cmd_linear_x": "",
        "lowlevel_cmd_angular_z": "",
    }

    if obs is None:
        return out

    try:
        data = obs.data if hasattr(obs, "data") else obs
        if len(data) >= 14:
            out["imu_stamp"] = data[1]
            out["imu_acc_x"] = data[2]
            out["imu_acc_y"] = data[3]
            out["imu_acc_z"] = data[4]
            out["imu_gyro_x"] = data[5]
            out["imu_gyro_y"] = data[6]
            out["imu_gyro_z"] = data[7]
            out["imu_quat_x"] = data[8]
            out["imu_quat_y"] = data[9]
            out["imu_quat_z"] = data[10]
            out["imu_quat_w"] = data[11]
            out["lowlevel_cmd_linear_x"] = data[12]
            out["lowlevel_cmd_angular_z"] = data[13]
    except Exception:
        pass

    return out



class LatestImuBuffer:
    """
    IMU 토픽을 직접 subscribe해서 CSV에 기록할 최신 IMU 값을 보관한다.
    bot 내부 obs가 비어 있어도 이 fallback으로 IMU 값이 저장된다.
    """
    def __init__(self, topic_names):
        if isinstance(topic_names, str):
            topic_names = [t.strip() for t in topic_names.split(",") if t.strip()]

        self.latest = None
        self.latest_topic = ""
        self.subs = []

        for topic in topic_names:
            self.subs.append(
                rospy.Subscriber(
                    topic,
                    Imu,
                    self._callback,
                    callback_args=topic,
                    queue_size=100,
                )
            )

    def _callback(self, msg, topic):
        stamp = msg.header.stamp.to_sec()
        if stamp == 0.0:
            stamp = rospy.Time.now().to_sec()

        self.latest_topic = topic
        self.latest = {
            "imu_stamp": stamp,
            "imu_acc_x": msg.linear_acceleration.x,
            "imu_acc_y": msg.linear_acceleration.y,
            "imu_acc_z": msg.linear_acceleration.z,
            "imu_gyro_x": msg.angular_velocity.x,
            "imu_gyro_y": msg.angular_velocity.y,
            "imu_gyro_z": msg.angular_velocity.z,
            "imu_quat_x": msg.orientation.x,
            "imu_quat_y": msg.orientation.y,
            "imu_quat_z": msg.orientation.z,
            "imu_quat_w": msg.orientation.w,
        }

    def get(self):
        if self.latest is None:
            return {
                "imu_stamp": "",
                "imu_acc_x": "",
                "imu_acc_y": "",
                "imu_acc_z": "",
                "imu_gyro_x": "",
                "imu_gyro_y": "",
                "imu_gyro_z": "",
                "imu_quat_x": "",
                "imu_quat_y": "",
                "imu_quat_z": "",
                "imu_quat_w": "",
            }
        return dict(self.latest)


def merge_robot_obs_with_direct_imu(robot_obs, direct_imu):
    """
    우선순위:
      1) TurtleBotAPI obs에 IMU 값이 있으면 그대로 사용
      2) obs IMU 값이 비어 있으면 직접 subscribe한 IMU 값으로 채움
      3) lowlevel cmd는 기존 obs 값을 유지
    """
    out = dict(robot_obs)
    for key, value in direct_imu.items():
        if out.get(key, "") == "" and value != "":
            out[key] = value
    return out


def reverse_track_if_needed(track):
    track.points = track.points[::-1].copy()
    track.kappa = -track.kappa[::-1].copy()
    track.dkappa_ds = track.dkappa_ds[::-1].copy()
    track._build_arclength()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--speed", type=float, default=0.40)
    parser.add_argument("--lookahead", type=float, default=0.50)
    parser.add_argument("--rate_hz", type=float, default=100.0)
    parser.add_argument("--duration", type=float, default=0.0)

    # 1이면 시작 progress에서 한 바퀴 돌아왔을 때 자동 정지
    # 0이면 계속 주행
    parser.add_argument("--stop_after_lap", type=int, default=1)
    parser.add_argument("--lap_stop_margin", type=float, default=0.20)
    parser.add_argument("--min_lap_time", type=float, default=5.0)

    parser.add_argument("--max_w", type=float, default=2.84)
    parser.add_argument("--max_v", type=float, default=0.40)
    parser.add_argument("--max_track_error", type=float, default=0.80)

    parser.add_argument("--frame_id", type=str, default="map")
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--dry_run", action="store_true")
    parser.add_argument(
        "--imu_topic",
        type=str,
        default=f"/{ROBOT_NAME}/imu,/imu,/imu/data",
        help="Subscribe할 IMU 토픽. 여러 개는 comma로 구분. 예: /turtlebot8/imu,/imu",
    )

    args = parser.parse_args()

    rospy.init_node("a105_pure_pursuit_track_follow", anonymous=True)

    os.makedirs(SAVE_ROOT, exist_ok=True)
    stamp = int(time.time())
    session_dir = os.path.join(SAVE_ROOT, f"session_{stamp}")
    os.makedirs(session_dir, exist_ok=True)
    csv_path = os.path.join(session_dir, "pure_pursuit_log.csv")

    track = TrackPath()

    if args.reverse:
        reverse_track_if_needed(track)

    print("")
    print("===== Track summary =====")
    for k, v in track.summary().items():
        print(f"{k}: {v}")

    mocap = MocapAPI(
        topic=MOCAP_TOPIC,
        rate_hz=args.rate_hz,
    )

    print("")
    print("Mocap calibration info:")
    print(mocap.calib.as_dict())

    if not mocap.calib.loaded:
        raise RuntimeError("Mocap calibration이 loaded=False야. 보정 JSON을 먼저 확인해야 해.")

    bot = make_bot(rate_hz=args.rate_hz)
    imu_buffer = LatestImuBuffer(args.imu_topic)
    viz = PurePursuitVisualizer(frame_id=args.frame_id)

    # 시작하자마자 RViz에 전체 트랙을 먼저 표시한다.
    viz.publish_static_track()
    rospy.sleep(0.2)
    viz.publish_static_track()

    print("")
    print("Track markers published.")
    print("RViz marker topic:", MARKER_TOPIC)
    print("Waiting for mocap data...")
    while not rospy.is_shutdown() and mocap.latest_data is None:
        rospy.sleep(0.01)

    print("Mocap data received.")
    print("")
    print("Pure Pursuit ready.")
    print(f"speed       : {args.speed}")
    print(f"lookahead   : {args.lookahead}")
    print(f"max_w       : {args.max_w}")
    print(f"control dt  : {CONTROL_DT}")
    print(f"log rate hz : {args.rate_hz}")
    print(f"duration    : {args.duration}  (0 means infinite)")
    print(f"stop lap     : {bool(args.stop_after_lap)}")
    print(f"lap margin   : {args.lap_stop_margin}")
    print(f"min lap time : {args.min_lap_time}")
    print(f"dry_run     : {args.dry_run}")
    print(f"marker topic: {MARKER_TOPIC}")
    print(f"imu topic candidates: {args.imu_topic}")
    print("")

    input("Enter를 누르면 시작. 중지는 Ctrl+C: ")

    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(args.rate_hz)

    last_log_time = None
    next_control_time = 0.0
    last_control_time = 0.0
    latest_v_cmd = 0.0
    latest_w_cmd = 0.0
    latest_pp = None
    latest_near = None

    lap_start_s = None
    lap_prev_s = None
    lap_progress = 0.0

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "time",
            "dt",
            "control_time",
            "control_update",

            "raw_mocap_x",
            "raw_mocap_y",
            "raw_mocap_z",
            "raw_mocap_yaw",

            "model_x",
            "model_y",
            "model_z",
            "model_yaw",
            "model_yaw_rate",
            "model_yaw_accel",
            "model_vx",
            "model_vy",
            "model_vz",
            "model_speed",
            "model_v_forward",
            "model_v_lateral",
            "model_ax",
            "model_ay",
            "model_az",

            "imu_stamp",
            "imu_acc_x",
            "imu_acc_y",
            "imu_acc_z",
            "imu_gyro_x",
            "imu_gyro_y",
            "imu_gyro_z",
            "imu_quat_x",
            "imu_quat_y",
            "imu_quat_z",
            "imu_quat_w",
            "lowlevel_cmd_linear_x",
            "lowlevel_cmd_angular_z",

            "s",
            "theta",
            "x_ref",
            "y_ref",
            "yaw_ref",
            "lateral_error",
            "heading_error",
            "kappa",
            "dkappa_ds",

            "target_x",
            "target_y",
            "target_s",
            "target_theta",
            "target_kappa",

            "cmd_linear_x",
            "cmd_angular_z",
            "alpha",
            "kappa_cmd",
            "distance_to_track",
        ])

        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now().to_sec()
                elapsed = now - start_time

                if args.duration > 0.0 and elapsed >= args.duration:
                    break

                pose = get_model_pose(mocap)

                if pose is None:
                    bot.put_control(0.0, 0.0)
                    rate.sleep()
                    continue

                x = pose["x"]
                y = pose["y"]
                yaw = pose["yaw"]

                control_update = False

                if elapsed + 1e-9 >= next_control_time:
                    control_update = True
                    last_control_time = elapsed

                    near = track.nearest(x, y, yaw)

                    # 안전장치: 트랙에서 너무 멀면 정지
                    if abs(near["lateral_error"]) > args.max_track_error:
                        latest_v_cmd = 0.0
                        latest_w_cmd = 0.0

                        print(
                            f"[{elapsed:7.2f}s] STOP: track error too large "
                            f"e_y={near['lateral_error']:.3f}"
                        )

                        pp = track.pure_pursuit(
                            x=x,
                            y=y,
                            yaw=yaw,
                            v=0.0,
                            lookahead=args.lookahead,
                        )

                    else:
                        latest_v_cmd = clamp(args.speed, -args.max_v, args.max_v)

                        pp = track.pure_pursuit(
                            x=x,
                            y=y,
                            yaw=yaw,
                            v=latest_v_cmd,
                            lookahead=args.lookahead,
                        )

                        latest_w_cmd = clamp(pp["omega"], -args.max_w, args.max_w)

                    latest_near = near
                    latest_pp = pp

                    # 시작 위치의 progress를 기준으로 한 바퀴 완료 여부 계산
                    if lap_start_s is None:
                        lap_start_s = near["s"]
                        lap_prev_s = near["s"]
                        lap_progress = 0.0
                        print(
                            f"Lap start set: s0={lap_start_s:.3f}, "
                            f"theta0={near['theta']:.3f}, track_length={track.length:.3f}"
                        )
                    else:
                        ds_lap = near["s"] - lap_prev_s

                        # closed track wrap-around 보정
                        if ds_lap < -0.5 * track.length:
                            ds_lap += track.length
                        elif ds_lap > 0.5 * track.length:
                            ds_lap -= track.length

                        # 후진/노이즈로 progress가 줄어드는 것은 lap 진행도로 보지 않음
                        lap_progress += max(0.0, ds_lap)
                        lap_prev_s = near["s"]

                    if (
                        int(args.stop_after_lap) == 1
                        and elapsed >= args.min_lap_time
                        and lap_progress >= max(0.0, track.length - args.lap_stop_margin)
                    ):
                        latest_v_cmd = 0.0
                        latest_w_cmd = 0.0
                        bot.put_control(0.0, 0.0)
                        print(
                            f"[{elapsed:7.2f}s] LAP FINISHED: "
                            f"progress={lap_progress:.3f}/{track.length:.3f}, "
                            f"s={near['s']:.3f}, theta={near['theta']:.3f}"
                        )
                        break

                    if args.dry_run:
                        bot.put_control(0.0, 0.0)
                    else:
                        bot.put_control(latest_v_cmd, latest_w_cmd)

                    while next_control_time <= elapsed + 1e-9:
                        next_control_time += CONTROL_DT

                # 아직 첫 control update 전이면 한 번 계산해둔다.
                if latest_near is None or latest_pp is None:
                    latest_near = track.nearest(x, y, yaw)
                    latest_pp = track.pure_pursuit(
                        x=x,
                        y=y,
                        yaw=yaw,
                        v=0.0,
                        lookahead=args.lookahead,
                    )

                near = latest_near
                pp = latest_pp

                viz.publish(
                    x=x,
                    y=y,
                    yaw=yaw,
                    near=near,
                    pp=pp,
                    v_cmd=latest_v_cmd,
                    w_cmd=latest_w_cmd,
                )

                if last_log_time is None:
                    dt = 0.0
                else:
                    dt = elapsed - last_log_time
                last_log_time = elapsed

                mocap_data = mocap.latest_data

                # Brownian logger와 동일하게 매 로그 루프에서 TurtleBotAPI 내부 obs를 갱신한다.
                # 이 호출이 없으면 latest_obs / obs / latest_data가 오래된 값이거나 None으로 남아서
                # CSV의 imu_* 컬럼이 빈 칸으로 기록될 수 있다.
                try:
                    bot.get_data()
                except AttributeError:
                    # TurtleBotAPI 버전에 따라 get_data가 없을 수도 있으므로,
                    # 그런 경우에는 subscriber/direct IMU fallback만 사용한다.
                    pass

                robot_obs_from_bot = parse_robot_obs(get_latest_robot_obs(bot))
                robot_obs = merge_robot_obs_with_direct_imu(
                    robot_obs_from_bot,
                    imu_buffer.get(),
                )

                raw = get_nested(mocap_data, ["raw"], {})
                model = get_nested(mocap_data, ["model"], {})

                writer.writerow([
                    elapsed,
                    dt,
                    last_control_time,
                    int(control_update),

                    get_nested(raw, ["position", "x"]),
                    get_nested(raw, ["position", "y"]),
                    get_nested(raw, ["position", "z"]),
                    get_nested(raw, ["yaw"]),

                    get_nested(model, ["position", "x"]),
                    get_nested(model, ["position", "y"]),
                    get_nested(model, ["position", "z"]),
                    get_nested(model, ["yaw"]),
                    get_nested(model, ["yaw_rate"]),
                    get_nested(model, ["yaw_accel"]),
                    get_nested(model, ["velocity_world", "x"]),
                    get_nested(model, ["velocity_world", "y"]),
                    get_nested(model, ["velocity_world", "z"]),
                    get_nested(model, ["speed"]),
                    get_nested(model, ["velocity_body", "forward"]),
                    get_nested(model, ["velocity_body", "lateral"]),
                    get_nested(model, ["acceleration_world", "x"]),
                    get_nested(model, ["acceleration_world", "y"]),
                    get_nested(model, ["acceleration_world", "z"]),

                    robot_obs["imu_stamp"],
                    robot_obs["imu_acc_x"],
                    robot_obs["imu_acc_y"],
                    robot_obs["imu_acc_z"],
                    robot_obs["imu_gyro_x"],
                    robot_obs["imu_gyro_y"],
                    robot_obs["imu_gyro_z"],
                    robot_obs["imu_quat_x"],
                    robot_obs["imu_quat_y"],
                    robot_obs["imu_quat_z"],
                    robot_obs["imu_quat_w"],
                    robot_obs["lowlevel_cmd_linear_x"],
                    robot_obs["lowlevel_cmd_angular_z"],

                    near["s"],
                    near["theta"],
                    near["x_ref"],
                    near["y_ref"],
                    near["yaw_ref"],
                    near["lateral_error"],
                    near.get("heading_error", 0.0),
                    near["kappa"],
                    near["dkappa_ds"],

                    pp["target_x"],
                    pp["target_y"],
                    pp["target_s"],
                    pp["target_theta"],
                    pp["target_kappa"],

                    latest_v_cmd,
                    latest_w_cmd,
                    pp["alpha"],
                    pp["kappa_cmd"],
                    near["distance"],
                ])

                if control_update:
                    print(
                        f"[{elapsed:7.2f}s] "
                        f"s={near['s']:.2f}/{track.length:.2f}, "
                        f"lap={lap_progress:.2f}/{track.length:.2f}, "
                        f"e_y={near['lateral_error']:+.3f}, "
                        f"e_psi={near.get('heading_error', 0.0):+.3f}, "
                        f"kappa={near['kappa']:+.3f}, "
                        f"v={latest_v_cmd:+.2f}, w={latest_w_cmd:+.2f}"
                    )

                rate.sleep()

        except KeyboardInterrupt:
            print("")
            print("Stopped by Ctrl+C.")

    print("Stopping robot...")
    for _ in range(20):
        bot.put_control(0.0, 0.0)
        rospy.sleep(0.02)

    bot.close()

    print("")
    print("Finished.")
    print("log csv:", csv_path)
    print("RViz marker topic:", MARKER_TOPIC)


if __name__ == "__main__":
    main()