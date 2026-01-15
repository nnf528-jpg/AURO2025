#!/usr/bin/env python3
"""
Robot Controller for Hazardous Material Collection
- Collects RED (contaminated) barrels
- Deposits in GREEN collection zone  
- Decontaminates in BLUE zone when radiation >= 30
"""

import rclpy
from rclpy.node import Node
from geometry_msgs. msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from assessment_interfaces.msg import BarrelList, ZoneList, RadiationList
from auro_interfaces.srv import ItemRequest
import numpy as np
import math
from enum import Enum
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RobotState(Enum):
    SEARCHING = 1
    ALIGNING_TO_BARREL = 2
    DRIVING_TO_BARREL = 3
    PASSING_BARREL = 4
    COLLECTING = 5
    NAVIGATING_TO_ZONE = 6
    APPROACHING_ZONE = 7
    DEPOSITING = 8
    NAVIGATING_TO_DECONTAMINATION = 9
    APPROACHING_DECONTAMINATION = 10
    DECONTAMINATING = 11
    NAVIGATING_TO_BARRELS = 12


class RobotController(Node):
    RED = 0
    BLUE = 1
    ZONE_BLUE = 0    # Decontamination zone (BLUE color)
    ZONE_GREEN = 1   # Collection zone (GREEN color)
    IMAGE_CENTER_X = 320

    def __init__(self):
        super().__init__('robot_controller')
        
        self.robot_id = self.get_namespace().strip('/')
        if not self.robot_id:
            self.robot_id = 'robot1'

        self.state = RobotState.SEARCHING
        self.holding_barrel = False
        self.service_in_progress = False
        self. barrels_collected = 0
        self.radiation_level = 0
        self.state_timer = 0
        self.pickup_attempts = 0
        self.nav_log_counter = 0

        # Known locations
        self.storage_zone = {'x': 13.5, 'y': 9.4}       # GREEN collection zone
        self.decontamination_zone = {'x': 7.5, 'y': 9.4}  # BLUE decontamination zone
        self.barrel_areas = [
            {'x': -2.0, 'y': 4.0},
            {'x': -9.0, 'y': 5.0},
            {'x':  -12.0, 'y': 6.0},
            {'x': -11.0, 'y': 10.0},
        ]
        self.current_barrel_area = 0

        # Sensor data
        self.front_distance = 10.0
        self.left_distance = 10.0
        self.right_distance = 10.0
        self.target_barrel = None
        self.visible_zones = []
        self.robot_x = 0.0
        self.robot_y = -2.0
        self.robot_yaw = 0.0

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        barrel_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy. BEST_EFFORT)
        self.create_subscription(BarrelList, 'barrels', self.barrel_callback, barrel_qos)
        self.create_subscription(ZoneList, 'zones', self.zone_callback, barrel_qos)
        self.create_subscription(RadiationList, '/radiation_levels', self.radiation_callback, 10)

        # Service clients
        self.pickup_client = self.create_client(ItemRequest, '/pick_up_item')
        self.offload_client = self.create_client(ItemRequest, '/offload_item')
        self.decontaminate_client = self.create_client(ItemRequest, '/decontaminate')
        
        self.pickup_client.wait_for_service(timeout_sec=10.0)
        self.offload_client.wait_for_service(timeout_sec=10.0)
        self.decontaminate_client.wait_for_service(timeout_sec=10.0)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('=== HAZMAT ROBOT STARTED:  ' + self.robot_id + ' ===')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        
        n = len(ranges)
        front_start = n - n // 24
        front_end = n // 24
        front_ranges = list(ranges[front_start:]) + list(ranges[:front_end])
        self.front_distance = float(np.min(front_ranges)) if front_ranges else msg.range_max
        
        left_start = n // 4 - n // 12
        left_end = n // 4 + n // 12
        self.left_distance = float(np.min(ranges[left_start:left_end]))
        
        right_start = 3 * n // 4 - n // 12
        right_end = 3 * n // 4 + n // 12
        self.right_distance = float(np.min(ranges[right_start:right_end]))

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self. robot_y = msg.pose. pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def barrel_callback(self, msg):
        if self.holding_barrel or self.state in [RobotState.PASSING_BARREL, RobotState.COLLECTING]:
            self.target_barrel = None
            return
            
        red_barrels = [b for b in msg.data if b.colour == self.RED]
        if red_barrels:
            self.target_barrel = max(red_barrels, key=lambda b: b.size)
        else:
            self.target_barrel = None

    def zone_callback(self, msg):
        self.visible_zones = list(msg.data)

    def radiation_callback(self, msg):
        for entry in msg.data:
            if entry.robot_id == self.robot_id:
                old_level = self.radiation_level
                self.radiation_level = entry. level
                if abs(self.radiation_level - old_level) >= 5:
                    self.get_logger().info('Radiation level:  ' + str(self.radiation_level))
                break

    def get_distance_to(self, x, y):
        return math. sqrt((x - self.robot_x)**2 + (y - self. robot_y)**2)

    def get_angle_to(self, x, y):
        angle = math.atan2(y - self.robot_y, x - self.robot_x) - self.robot_yaw
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def navigate_to(self, x, y, cmd):
        dist = self.get_distance_to(x, y)
        angle = self.get_angle_to(x, y)
        
        if self.front_distance < 0.4:
            cmd.linear.x = 0.0
            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.5
            else:
                cmd. angular.z = -0.5
            return dist
        
        if abs(angle) > 0.2:
            cmd.angular.z = 0.5 if angle > 0 else -0.5
            cmd.linear.x = 0.05
        else:
            cmd.angular.z = angle * 1.0
            cmd.linear.x = min(0.22, dist * 0.3)
        return dist

    def call_service(self, client, callback):
        if self.service_in_progress:
            return
        self.service_in_progress = True
        request = ItemRequest. Request()
        request.robot_id = self.robot_id
        future = client.call_async(request)
        future.add_done_callback(callback)

    def pickup_done(self, future):
        self.service_in_progress = False
        try:
            if future.result().success:
                self.holding_barrel = True
                self. barrels_collected += 1
                self.pickup_attempts = 0
                self.get_logger().info('*** COLLECTED BARREL #' + str(self.barrels_collected) + ' ***')
                self.state = RobotState.NAVIGATING_TO_ZONE
                self.nav_log_counter = 0
                self.get_logger().info('Heading to GREEN collection zone...')
            else:
                self.pickup_attempts += 1
                self.get_logger().warn('Pickup failed, attempt ' + str(self.pickup_attempts))
                if self.pickup_attempts >= 3:
                    self. pickup_attempts = 0
                    self.state = RobotState.SEARCHING
                else:
                    self.state = RobotState.ALIGNING_TO_BARREL
        except Exception as e:
            self.get_logger().error('Pickup error: ' + str(e))
            self.state = RobotState.SEARCHING

    def offload_done(self, future):
        self.service_in_progress = False
        try:
            if future. result().success:
                self. holding_barrel = False
                self.get_logger().info('*** DEPOSITED BARREL IN GREEN ZONE, TOTAL:  ' + str(self.barrels_collected) + ' ***')
                
                if self.radiation_level >= 30:
                    self.get_logger().warn('Radiation high (' + str(self.radiation_level) + '), going to BLUE decontamination zone!')
                    self.state = RobotState.NAVIGATING_TO_DECONTAMINATION
                    self.nav_log_counter = 0
                else:
                    self.get_logger().info('Going to find more barrels...')
                    self.state = RobotState. NAVIGATING_TO_BARRELS
            else:
                self.get_logger().warn('Offload failed, retrying...')
                self.state = RobotState.APPROACHING_ZONE
        except Exception as e:
            self.get_logger().error('Offload error: ' + str(e))

    def decontaminate_done(self, future):
        self.service_in_progress = False
        try:
            if future.result().success:
                self.get_logger().info('*** DECONTAMINATION COMPLETE - Radiation cleared ***')
            else:
                self.get_logger().warn('Decontamination failed')
            self.state = RobotState.NAVIGATING_TO_BARRELS
        except Exception as e:
            self.get_logger().error('Decontaminate error: ' + str(e))
            self.state = RobotState.NAVIGATING_TO_BARRELS

    def control_loop(self):
        cmd = Twist()
        
        if self.service_in_progress:
            self.cmd_pub.publish(cmd)
            return

        if self.state == RobotState.SEARCHING:
            self. do_searching(cmd)
        elif self.state == RobotState.ALIGNING_TO_BARREL: 
            self.do_aligning(cmd)
        elif self.state == RobotState.DRIVING_TO_BARREL:
            self.do_driving(cmd)
        elif self.state == RobotState.PASSING_BARREL:
            self.do_passing(cmd)
        elif self.state == RobotState. COLLECTING:
            self. do_collecting(cmd)
        elif self.state == RobotState.NAVIGATING_TO_ZONE:
            self.do_nav_to_zone(cmd)
        elif self.state == RobotState.APPROACHING_ZONE:
            self.do_approach_zone(cmd)
        elif self.state == RobotState.DEPOSITING:
            self.do_depositing(cmd)
        elif self.state == RobotState.NAVIGATING_TO_DECONTAMINATION: 
            self.do_nav_to_decon(cmd)
        elif self.state == RobotState. APPROACHING_DECONTAMINATION: 
            self.do_approach_decon(cmd)
        elif self.state == RobotState.DECONTAMINATING: 
            self.do_decontaminating(cmd)
        elif self.state == RobotState. NAVIGATING_TO_BARRELS:
            self.do_nav_to_barrels(cmd)

        self.cmd_pub.publish(cmd)

    def do_searching(self, cmd):
        self.state_timer += 1
        
        if self.target_barrel:
            self.state = RobotState.ALIGNING_TO_BARREL
            self.state_timer = 0
            self.get_logger().info('Barrel found, aligning.. .')
            return
        
        if self.state_timer > 80:
            self.state = RobotState.NAVIGATING_TO_BARRELS
            self.state_timer = 0
            return
        
        cmd.angular.z = 0.4

    def do_aligning(self, cmd):
        self.state_timer += 1
        
        if not self.target_barrel:
            if self.state_timer > 30:
                self.state = RobotState.SEARCHING
                self.state_timer = 0
            return
        
        error = self.target_barrel.x - self.IMAGE_CENTER_X
        
        if abs(error) < 40:
            self.state = RobotState.DRIVING_TO_BARREL
            self.state_timer = 0
            self.get_logger().info('Aligned!  Driving to barrel...')
            return
        
        cmd.linear.x = 0.0
        cmd.angular.z = 0.25 if error < 0 else -0.25

    def do_driving(self, cmd):
        self.state_timer += 1
        
        if self.front_distance < 0.30:
            self.state = RobotState. PASSING_BARREL
            self.state_timer = 0
            self.get_logger().info('Close!  Passing barrel...')
            return
        
        if not self.target_barrel:
            if self.state_timer > 20:
                self.state = RobotState.SEARCHING
                self.state_timer = 0
            else:
                cmd.linear.x = 0.15
            return
        
        error = self.target_barrel.x - self.IMAGE_CENTER_X
        
        if abs(error) > 120:
            self.state = RobotState.ALIGNING_TO_BARREL
            self.state_timer = 0
            return
        
        cmd.linear. x = 0.18
        cmd.angular.z = -0.002 * error
        cmd.angular.z = max(-0.15, min(0.15, cmd.angular.z))
        
        if self.state_timer % 10 == 0:
            self.get_logger().info('Driving:  err=' + str(int(error)) + ' front=' + str(round(self.front_distance, 2)))

    def do_passing(self, cmd):
        self.state_timer += 1
        
        if self.state_timer < 35:
            cmd.linear.x = 0.20
            cmd.angular.z = 0.0
        else:
            self.state = RobotState. COLLECTING
            self.state_timer = 0
            self. get_logger().info('Passed barrel, collecting...')

    def do_collecting(self, cmd):
        self.get_logger().info('Attempting pickup...')
        self.call_service(self.pickup_client, self.pickup_done)

    def do_nav_to_zone(self, cmd):
        self.nav_log_counter += 1
        
        green_zones = [z for z in self.visible_zones if z.zone == self.ZONE_GREEN]
        if green_zones:
            self.state = RobotState.APPROACHING_ZONE
            self.state_timer = 0
            self.get_logger().info('GREEN zone visible!  Approaching...')
            return
        
        dist = self.navigate_to(self.storage_zone['x'], self.storage_zone['y'], cmd)
        
        if self.nav_log_counter % 50 == 0:
            self.get_logger().info('Nav to GREEN:  dist=' + str(round(dist, 1)) + 
                                   ' pos=(' + str(round(self. robot_x, 1)) + ',' + str(round(self.robot_y, 1)) + ')')
        
        if dist < 3.0:
            self.state = RobotState.APPROACHING_ZONE
            self.state_timer = 0

    def do_approach_zone(self, cmd):
        self.state_timer += 1
        
        green_zones = [z for z in self.visible_zones if z. zone == self.ZONE_GREEN]
        
        if not green_zones: 
            cmd.angular.z = 0.3
            cmd.linear.x = 0.05
            if self.state_timer > 50:
                self.state = RobotState.NAVIGATING_TO_ZONE
                self.state_timer = 0
            return
        
        zone = max(green_zones, key=lambda z: z.size)
        error = zone.x - self.IMAGE_CENTER_X
        
        if zone.size > 20000:
            self.state = RobotState.DEPOSITING
            self.state_timer = 0
            self.get_logger().info('Inside GREEN zone! Depositing...')
            return
        
        cmd.linear.x = 0.15
        if abs(error) < 40:
            cmd.angular.z = 0.0
        else:
            cmd. angular.z = -0.003 * error
            cmd.angular.z = max(-0.3, min(0.3, cmd.angular.z))

    def do_depositing(self, cmd):
        self.get_logger().info('Attempting deposit in GREEN zone...')
        self.call_service(self.offload_client, self.offload_done)

    def do_nav_to_decon(self, cmd):
        """Navigate to BLUE decontamination zone"""
        self.nav_log_counter += 1
        
        blue_zones = [z for z in self.visible_zones if z. zone == self.ZONE_BLUE]
        if blue_zones:
            self.state = RobotState.APPROACHING_DECONTAMINATION
            self.state_timer = 0
            self.get_logger().info('BLUE zone visible!  Approaching...')
            return
        
        dist = self.navigate_to(self.decontamination_zone['x'], self.decontamination_zone['y'], cmd)
        
        if self.nav_log_counter % 50 == 0:
            self.get_logger().info('Nav to BLUE: dist=' + str(round(dist, 1)) + 
                                   ' pos=(' + str(round(self.robot_x, 1)) + ',' + str(round(self.robot_y, 1)) + ')')
        
        if dist < 3.0:
            self. state = RobotState. APPROACHING_DECONTAMINATION
            self.state_timer = 0

    def do_approach_decon(self, cmd):
        """Approach BLUE decontamination zone"""
        self. state_timer += 1
        
        blue_zones = [z for z in self.visible_zones if z.zone == self. ZONE_BLUE]
        
        if not blue_zones: 
            cmd.angular.z = 0.3
            cmd.linear.x = 0.05
            if self.state_timer > 50:
                self.state = RobotState.NAVIGATING_TO_DECONTAMINATION
                self.state_timer = 0
            return
        
        zone = max(blue_zones, key=lambda z: z.size)
        error = zone.x - self.IMAGE_CENTER_X
        
        if zone.size > 20000:
            self.state = RobotState.DECONTAMINATING
            self.state_timer = 0
            self.get_logger().info('Inside BLUE zone! Decontaminating...')
            return
        
        cmd.linear.x = 0.15
        if abs(error) < 40:
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = -0.003 * error
            cmd.angular.z = max(-0.3, min(0.3, cmd.angular.z))

    def do_decontaminating(self, cmd):
        """Decontaminate in BLUE zone"""
        self.get_logger().info('Attempting decontamination.. .')
        self.call_service(self.decontaminate_client, self.decontaminate_done)

    def do_nav_to_barrels(self, cmd):
        """Navigate to barrel areas to find more barrels"""
        if self.target_barrel:
            self.state = RobotState.ALIGNING_TO_BARREL
            self.state_timer = 0
            return
        
        target = self.barrel_areas[self.current_barrel_area]
        dist = self.navigate_to(target['x'], target['y'], cmd)
        
        if dist < 1.5:
            self.current_barrel_area = (self. current_barrel_area + 1) % len(self.barrel_areas)
            self.state = RobotState.SEARCHING
            self.state_timer = 0
            self.get_logger().info('Reached barrel area, searching...')


def main():
    rclpy.init()
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': 
    main()