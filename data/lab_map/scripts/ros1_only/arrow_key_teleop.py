#!/usr/bin/env python3

import sys
import select
import termios
import tty
import rospy

from geometry_msgs.msg import Twist


TOPIC = "/turtlebot8/high_cmd_vel"

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
    print("========== A105 Number Key Teleop ==========")
    print("8     : forward 계속 이동")
    print("2     : backward 계속 이동")
    print("4     : rotate left 계속 회전")
    print("6     : rotate right 계속 회전")
    print("5     : stop")
    print("+/=   : increase speed")
    print("-     : decrease speed")
    print("q     : stop and quit")
    print("")
    print(f"topic       : {TOPIC}")
    print(f"publish_hz  : {PUBLISH_HZ}")
    print("============================================")
    print("")


def main():
    global LINEAR_SPEED, ANGULAR_SPEED

    rospy.init_node("a105_number_key_teleop", anonymous=True)
    pub = rospy.Publisher(TOPIC, Twist, queue_size=10)

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

            cmd = Twist()
            cmd.linear.x = current_v
            cmd.angular.z = current_w
            pub.publish(cmd)

            rate.sleep()

    finally:
        stop = Twist()
        for _ in range(20):
            pub.publish(stop)
            rospy.sleep(0.02)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("")
        print("Number key teleop stopped.")


if __name__ == "__main__":
    main()
