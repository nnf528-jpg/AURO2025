#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from assessment_interfaces.msg import BarrelList
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        barrel_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
   
        self.create_subscription(
            BarrelList,
            'barrels',
            self.barrel_callback,
            barrel_qos
        )

        self.front = 10.0
        self.left = 10.0
        self.right = 10.0
        self.target_barrel = None

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'Robot controller started (LASER + BARREL FOLLOWING)'
        )

    # ---------- LASER ----------
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max

        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        def sector(center, width):
            mask = (angles > center - width) & (angles < center + width)
            return float(np.min(ranges[mask])) if np.any(mask) else msg.range_max

        self.front = sector(0.0, np.deg2rad(15))
        self.left = sector(math.pi / 2, np.deg2rad(20))
        self.right = sector(-math.pi / 2, np.deg2rad(20))

    # ---------- BARRELS ----------
    def barrel_callback(self, msg: BarrelList):
        if len(msg.data) == 0:
            self.target_barrel = None
            return

        # Pick the closest (largest blob)
        self.target_barrel = max(msg.data, key=lambda b: b.size)

    # ---------- CONTROL ----------
    def control_loop(self):
        cmd = Twist()

        # 1) Emergency obstacle avoidance
        if self.front < 0.6:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.6 if self.left > self.right else -0.6
            self.cmd_pub.publish(cmd)
            return

        # 2) Barrel following
        if self.target_barrel is not None:
            IMAGE_CENTER = 320.0
            K_TURN = 0.004
            CENTER_TOL = 15

            error = self.target_barrel.x - IMAGE_CENTER
            cmd.angular.z = max(min(-K_TURN * error, 0.6), -0.6)

            if abs(error) < CENTER_TOL:
                cmd.linear.x = 0.12
            else:
                cmd.linear.x = 0.0

            if self.target_barrel.size > 12000:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)
            return

        # 3) Search
        cmd.linear.x = 0.0
        cmd.angular.z = 0.6
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    