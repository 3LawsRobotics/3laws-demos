# Copyright 2024 3Laws Robotics Inc.
#
# Licensed under the MIT License

import sys
import termios
import threading
import tty

import geometry_msgs.msg
import rclpy
from pynput.keyboard import Listener
from rclpy.node import Node

helpMsg = """
This node takes key presses from the keyboard and publishes them as a Twist messages.
Contrary to the default teleop_twist_keyboard package, the keys need to be held.

W key: positive linear velocity along x-axis
S key: negative linear velocity along x-axis
A key: positive linear velocity along y-axis
D key: negative linear velocity along y-axis
Q key: positive angular velocity along z-axis
E key: negative angular velocity along z-axis


CTRL-C to quit
"""

keyStatus = {
    "w": False,
    "s": False,
    "a": False,
    "d": False,
    "q": False,
    "e": False,
}


def press(key):
    try:
        if key.char in keyStatus.keys():
            keyStatus[key.char] = True
    except AttributeError:
        pass


def release(key):
    try:
        if key.char in keyStatus.keys():
            keyStatus[key.char] = False
    except AttributeError:
        pass


class KeyboardInterface(Node):
    def __init__(self):
        super().__init__("lll_teleop_twist_keyboard")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("stamped", False),
                ("frame_id", ""),
                ("keyboard_publish_rate_ms", 100),
                ("cmd_output_topic_name", "cmd_vel"),
                ("vx_min", -1.0),
                ("vx_max", 1.0),
                ("vy_min", -1.0),
                ("vy_max", 1.0),
                ("wz_min", -1.0),
                ("wz_max", 1.0),
            ],
        )

        self.stamped = self.get_parameter("stamped").get_parameter_value().bool_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.keyboard_publish_rate_ms = (
            self.get_parameter("keyboard_publish_rate_ms").get_parameter_value().integer_value
        )
        self.cmd_output_topic_name = (
            self.get_parameter("cmd_output_topic_name").get_parameter_value().string_value
        )
        self.vx_min = self.get_parameter("vx_min").get_parameter_value().double_value
        self.vx_max = self.get_parameter("vx_max").get_parameter_value().double_value
        self.vy_min = self.get_parameter("vy_min").get_parameter_value().double_value
        self.vy_max = self.get_parameter("vy_max").get_parameter_value().double_value
        self.wz_min = self.get_parameter("wz_min").get_parameter_value().double_value
        self.wz_max = self.get_parameter("wz_max").get_parameter_value().double_value

        if not self.stamped and self.frame_id:
            raise Exception("'frame_id' can only be set when 'stamped' is True")

        if self.stamped:
            self.TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            self.TwistMsg = geometry_msgs.msg.Twist

        self.pubCmd = self.create_publisher(
            self.TwistMsg, self.cmd_output_topic_name, rclpy.qos.qos_profile_system_default
        )

        self.timer = self.create_timer(
            float(self.keyboard_publish_rate_ms) / 1000.0, self.timer_callback
        )

        print(helpMsg)

    def timer_callback(self):
        twist_msg = self.TwistMsg()

        if self.stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_id
        else:
            twist = twist_msg

        twist.linear.x = keyStatus["w"] * self.vx_max + keyStatus["s"] * self.vx_min
        twist.linear.y = keyStatus["a"] * self.vy_max + keyStatus["d"] * self.vy_min
        twist.angular.z = keyStatus["q"] * self.wz_max + keyStatus["e"] * self.wz_min

        self.pubCmd.publish(twist_msg)


def listenToKeyboard():
    with Listener(on_press=press, on_release=release) as listener:
        try:
            listener.join()
        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()

    node = KeyboardInterface()

    listener = Listener(on_press=press, on_release=release)
    listener.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if listener.is_alive():
        try:
            raise KeyboardInterrupt
        except KeyboardInterrupt:
            pass

    node.destroy_node()


if __name__ == "__main__":
    main()
