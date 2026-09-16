#!/usr/bin/env python3
"""
Minimal ROS 2 -> MuJoCo bridge for the AgileX Tracer 1.0 base.

Subscribes: /cmd_vel  (geometry_msgs/msg/Twist)
Publishes:  nothing. Scope here is "keyboard makes it drive in the viewer".

Run:
    python3 tracer_sim.py                       # this file
    ros2 run teleop_twist_keyboard teleop_twist_keyboard   # another terminal
"""

import os
import time

import mujoco
import mujoco.viewer
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# --- geometry: must match tracer.xml, and both should match the real base ---
WHEEL_RADIUS = 0.08   # m
TRACK_WIDTH = 0.40    # m, distance between the two driven wheel contact points

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tracer.xml")

# how many physics steps between viewer redraws
# timestep 0.002 s -> 8 steps is ~62 Hz of rendering, which is plenty
STEPS_PER_FRAME = 8


class TracerSim(Node):
    def __init__(self, model, data):
        super().__init__("tracer_mujoco_sim")
        self.model = model
        self.data = data

        # Look actuators up by name rather than hardcoding index 0 and 1, so
        # reordering the <actuator> block in the XML cannot silently swap
        # left and right on you.
        self.left_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wheel_vel"
        )
        self.right_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wheel_vel"
        )
        if self.left_id < 0 or self.right_id < 0:
            raise RuntimeError("actuator names not found in tracer.xml")

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.get_logger().info("listening on /cmd_vel")

    def on_cmd_vel(self, msg: Twist):
        # Tracer 1.0 is 2WD differential: it cannot strafe, so linear.y is
        # ignored. Only forward speed and yaw rate mean anything.
        v = msg.linear.x       # m/s
        w = msg.angular.z      # rad/s

        v_left = v - w * TRACK_WIDTH / 2.0
        v_right = v + w * TRACK_WIDTH / 2.0

        self.data.ctrl[self.left_id] = v_left / WHEEL_RADIUS    # rad/s
        self.data.ctrl[self.right_id] = v_right / WHEEL_RADIUS


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    rclpy.init()
    node = TracerSim(model, data)

    # launch_passive gives us the viewer window but leaves the stepping to us,
    # which is what we want: one loop owns both the physics clock and the ROS
    # callbacks, so there is no shared state across threads to get wrong.
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and rclpy.ok():
                frame_start = time.perf_counter()

                # Drain any pending /cmd_vel callbacks. timeout_sec=0 means
                # "do not block" - the physics clock stays in charge.
                rclpy.spin_once(node, timeout_sec=0.0)

                for _ in range(STEPS_PER_FRAME):
                    mujoco.mj_step(model, data)

                viewer.sync()

                # Sleep off whatever is left so sim time tracks wall time.
                # Without this the sim runs as fast as the CPU allows and the
                # robot teleports across the screen.
                lag = (STEPS_PER_FRAME * model.opt.timestep) - (
                    time.perf_counter() - frame_start
                )
                if lag > 0:
                    time.sleep(lag)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()