#!/usr/bin/env python3
"""Converts /dvl/data (waterlinked_a50_ros_driver/msg/DVL) to /bluerov2/dvl (nav_msgs/Odometry)."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from waterlinked_a50_ros_driver.msg import DVL


class DvlConverter(Node):
    def __init__(self):
        super().__init__('dvl_converter')
        self.pub = self.create_publisher(Odometry, '/bluerov2/dvl', 10)
        self.sub = self.create_subscription(DVL, '/dvl/data', self.callback, 10)

    def callback(self, msg: DVL):
        if not msg.velocity_valid:
            return
        odom = Odometry()
        odom.header = msg.header
        odom.twist.twist.linear.x = msg.velocity.x
        odom.twist.twist.linear.y = msg.velocity.y
        odom.twist.twist.linear.z = msg.velocity.z
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = DvlConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
