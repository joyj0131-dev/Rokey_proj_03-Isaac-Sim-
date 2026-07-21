#!/usr/bin/env python3
"""Keyboard teleop for the mecanum parking robot — external side (ROS 2 Humble, Python 3.10).

Arrow keys drive the robot via /cmd_vel. 'o' / 'c' open / fold the arms via the
/arm_control service (std_srvs/SetBool) and print the completion result the
Isaac driver sends back. Hold an arrow to keep moving (terminal key-repeat);
release and the robot stops after a short idle.

Run on any machine with ROS 2 Humble, on the SAME domain + whitelist as Isaac:
  export ROS_DOMAIN_ID=122
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
  source /opt/ros/humble/setup.bash
  python3 /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_teleop_key.py
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

LIN = 0.4          # m/s for drive / strafe
YAW = 0.6          # rad/s for rotate
IDLE_STOP = 0.35   # s without a key -> stop
CMD_VEL_TOPIC = "cmd_vel"
ARM_SERVICE = "arm_control"

HELP = (
    "\r\n=== mecanum teleop ===\r\n"
    "  ↑ / ↓   forward / back\r\n"
    "  ← / →   strafe left / right\r\n"
    "  a / d   rotate left / right\r\n"
    "  o / c   arm OPEN / CLOSE (service, waits for done)\r\n"
    "  space   stop\r\n"
    "  x       quit\r\n"
    "(hold an arrow to keep moving; release to stop)\r\n"
)


def get_key(timeout):
    fd = sys.stdin.fileno()
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return None
    # os.read (unbuffered) so escape sequences aren't swallowed by Python's
    # stdin buffer — that is what makes arrow keys work, not just single chars.
    ch = os.read(fd, 1)
    if ch == b"\x1b":  # escape sequence (arrow keys send \x1b [ A/B/C/D)
        r2, _, _ = select.select([fd], [], [], 0.01)
        if r2:
            ch += os.read(fd, 2)
    return ch.decode("latin-1", "ignore")


def say(msg):
    sys.stdout.write(msg + "\r\n")
    sys.stdout.flush()


def main():
    rclpy.init()
    node = rclpy.create_node("mecanum_teleop_key")
    pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
    arm_cli = node.create_client(SetBool, ARM_SERVICE)

    def publish(vx, vy, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        pub.publish(t)

    def call_arm(open_):
        publish(0.0, 0.0, 0.0)  # stop before moving arms
        if not arm_cli.wait_for_service(timeout_sec=1.5):
            say("[arm] service /%s not available (driver up? domain match?)" % ARM_SERVICE)
            return
        say("[arm] %s ..." % ("OPEN" if open_ else "CLOSE"))
        req = SetBool.Request()
        req.data = bool(open_)
        fut = arm_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=25.0)
        res = fut.result()
        if res is None:
            say("[arm] no response (timeout)")
        else:
            say("[arm] success=%s  \"%s\"" % (res.success, res.message))

    settings = termios.tcgetattr(sys.stdin)
    say(HELP)
    vx = vy = wz = 0.0
    last_key = 0.0
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            key = get_key(0.05)
            now = time.monotonic()
            if key is not None:
                if key in ("x", "\x03"):        # x or Ctrl-C
                    break
                elif key == "\x1b[A":            vx, vy, wz = +LIN, 0.0, 0.0
                elif key == "\x1b[B":            vx, vy, wz = -LIN, 0.0, 0.0
                elif key == "\x1b[D":            vx, vy, wz = 0.0, +LIN, 0.0   # strafe left
                elif key == "\x1b[C":            vx, vy, wz = 0.0, -LIN, 0.0   # strafe right
                elif key == "a":                 vx, vy, wz = 0.0, 0.0, +YAW
                elif key == "d":                 vx, vy, wz = 0.0, 0.0, -YAW
                elif key == " ":                 vx, vy, wz = 0.0, 0.0, 0.0
                elif key == "o":                 call_arm(True);  last_key = 0.0; continue
                elif key == "c":                 call_arm(False); last_key = 0.0; continue
                else:                            key = None
                if key is not None:
                    last_key = now

            if now - last_key > IDLE_STOP:       # released / idle -> stop
                vx = vy = wz = 0.0
            publish(vx, vy, wz)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        publish(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        say("teleop stopped.")


if __name__ == "__main__":
    main()
