#!/usr/bin/env python3

import sys
from pathlib import Path

CODE_DIR = Path("/home/user/Desktop/turtlebot_data/code")
MAIN_PC_DIR = CODE_DIR / "main_pc"

for p in [CODE_DIR, MAIN_PC_DIR]:
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import select
import termios
import tty
import rospy

from turtlebot_api import TurtleBotAPI


ROBOT_NAME = "turtlebot8"
PUBLISH_HZ = 50

LINEAR_SPEED = 0.22
ANGULAR_SPEED = 1.0

MAX_LINEAR = 0.4
MAX_ANGULAR = 2.84


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def get_key(timeout=0.001):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)

    if not rlist:
        return None

    return sys.stdin.read(1)


def print_help():
    print("")
    print("========== A105 TurtleBotAPI Number Teleop ==========")
    print("8     : forward 계속 이동")
    print("2     : backward 계속 이동")
    print("4     : rotate left 계속 회전")
    print("6     : rotate right 계속 회전")
    print("5     : stop")
    print("+/=   : increase speed")
    print("-     : decrease speed")
    print("q     : stop and quit")
    print("")
    print("control path:")
    print("  keyboard")
    print("    -> TurtleBotAPI.put_control()")
    print("    -> /turtlebot8/high_cmd_vel")
    print("    -> a105_lowlevel_bridge")
    print("    -> /turtlebot8/cmd_vel")
    print("=====================================================")
    print("")


def make_bot():
    try:
        return TurtleBotAPI(
            robot_name=ROBOT_NAME,
            rate_hz=PUBLISH_HZ,
            experiment_name="keyboard_teleop",
            auto_log=False
        )
    except TypeError:
        return TurtleBotAPI(
            robot_name=ROBOT_NAME,
            rate_hz=PUBLISH_HZ
        )


def main():
    global LINEAR_SPEED, ANGULAR_SPEED

    bot = make_bot()
    rate = rospy.Rate(PUBLISH_HZ)

    old_settings = termios.tcgetattr(sys.stdin)

    current_v = 0.0
    current_w = 0.0

    print_help()

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            key = get_key(0.001)

            if key is not None:
                if key == "8":
                    current_v = LINEAR_SPEED
                    current_w = 0.0
                    print(f"forward: v={current_v:.2f}, w={current_w:.2f}")

                elif key == "2":
                    current_v = -LINEAR_SPEED
                    current_w = 0.0
                    print(f"backward: v={current_v:.2f}, w={current_w:.2f}")

                elif key == "4":
                    current_v = 0.0
                    current_w = ANGULAR_SPEED
                    print(f"left: v={current_v:.2f}, w={current_w:.2f}")

                elif key == "6":
                    current_v = 0.0
                    current_w = -ANGULAR_SPEED
                    print(f"right: v={current_v:.2f}, w={current_w:.2f}")

                elif key == "5" or key == " ":
                    current_v = 0.0
                    current_w = 0.0
                    print("stop")

                elif key in ["+", "="]:
                    LINEAR_SPEED = clamp(LINEAR_SPEED + 0.02, 0.02, MAX_LINEAR)
                    ANGULAR_SPEED = clamp(ANGULAR_SPEED + 0.1, 0.1, MAX_ANGULAR)
                    print(f"speed up: linear={LINEAR_SPEED:.2f}, angular={ANGULAR_SPEED:.2f}")

                elif key == "-":
                    LINEAR_SPEED = clamp(LINEAR_SPEED - 0.02, 0.02, MAX_LINEAR)
                    ANGULAR_SPEED = clamp(ANGULAR_SPEED - 0.1, 0.1, MAX_ANGULAR)
                    print(f"speed down: linear={LINEAR_SPEED:.2f}, angular={ANGULAR_SPEED:.2f}")

                elif key.lower() == "q":
                    current_v = 0.0
                    current_w = 0.0
                    break

            bot.put_control(current_v, current_w)
            rate.sleep()

    finally:
        for _ in range(20):
            bot.put_control(0.0, 0.0)
            rospy.sleep(0.02)

        bot.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("")
        print("TurtleBotAPI number teleop stopped.")


if __name__ == "__main__":
    main()
