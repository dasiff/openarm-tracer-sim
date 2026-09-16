#!/usr/bin/env python3
"""
WASD keyboard teleop -> /cmd_vel (geometry_msgs/msg/Twist)

Drop-in replacement for teleop_twist_keyboard. Publishes the same message
on the same topic, so tracer_sim.py needs no changes - and so would the
real Tracer base.

    W / S   both wheels faster / slower   (linear.x)
    A / D   left-right wheel imbalance    (angular.z)
    SPACE   all stop
    X       zero the turn, keep driving
    Q       quit

Speeds are held, not pulsed: each press nudges the target and it stays
there until you change it. Same as a throttle, not a dead-man switch.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

LIN_STEP = 0.1        # m/s added per W / S press
ANG_STEP = 0.2        # rad/s added per A / D press
LIN_MAX = 1.6         # m/s   - Tracer 1.0 top speed, check against the manual
ANG_MAX = 2.0         # rad/s - arbitrary, tune to taste
PUBLISH_HZ = 20.0

BANNER = """
WASD teleop - publishing on /cmd_vel

  W / S   forward / back     (both wheels together)
  A / D   left / right       (wheel imbalance)
  X       straighten out
  SPACE   all stop
  Q       quit
"""


def clamp(value, limit):
    return max(-limit, min(limit, value))


class WasdTeleop(Node):
    def __init__(self):
        super().__init__("wasd_teleop")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.lin = 0.0
        self.ang = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)


def read_key(old_settings, timeout):
    """Non-blocking single-character read with the terminal in raw mode."""
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if ready else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


def main():
    old_settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = WasdTeleop()
    print(BANNER)

    period = 1.0 / PUBLISH_HZ
    try:
        while rclpy.ok():
            # The timeout doubles as the loop rate: we block here for one
            # publish period, then send whatever the current target is.
            # Publishing continuously (rather than only on keypress) is what
            # a real base expects - it lets a driver-side watchdog tell
            # "still commanded" apart from "operator went away".
            key = read_key(old_settings, period).lower()

            if key == "w":
                node.lin = clamp(node.lin + LIN_STEP, LIN_MAX)
            elif key == "s":
                node.lin = clamp(node.lin - LIN_STEP, LIN_MAX)
            elif key == "a":
                node.ang = clamp(node.ang + ANG_STEP, ANG_MAX)
            elif key == "d":
                node.ang = clamp(node.ang - ANG_STEP, ANG_MAX)
            elif key == "x":
                node.ang = 0.0
            elif key == " ":
                node.lin = 0.0
                node.ang = 0.0
            elif key == "q" or key == "\x03":      # q or Ctrl-C
                break

            node.publish()
            print(
                f"\r  linear.x {node.lin:+5.2f} m/s    "
                f"angular.z {node.ang:+5.2f} rad/s    ",
                end="",
                flush=True,
            )
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.lin = 0.0
        node.ang = 0.0
        node.publish()                              # leave the robot stopped
        node.destroy_node()
        rclpy.shutdown()
        print()


if __name__ == "__main__":
    main()