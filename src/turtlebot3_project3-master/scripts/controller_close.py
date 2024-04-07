#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from transformations import quaternion_matrix, euler_from_matrix


class RPMControlNode(Node):
    def __init__(self, action_sets, time_step=1.0):
        super().__init__('rpm_control_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.action_sets = action_sets
        self.action_index = 0
        self.wheel_radius = 0.033
        self.robot_base = 0.288
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.expected_x = 0.0
        self.expected_y = 0.0
        self.expected_theta = 0.0
        self.time_step = time_step
        self.timer = self.create_timer(self.time_step, self.timer_callback)

        self.max_linear_speed = 1.0  
        self.max_angular_speed = 1.0  

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_x = position.x
        self.current_y = position.y
        self.current_theta = self.quaternion_to_euler(orientation) 
        self.get_logger().info(f'Current position: ({self.current_x}, {self.current_y}, {math.degrees(self.current_theta)})')
        self.get_logger().info(f'Expected position: ({self.expected_x}, {self.expected_y}, {math.degrees(self.expected_theta)})')



    def quaternion_to_euler(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


    def rpm_to_velocity(self, rpm_left, rpm_right):
        v_left = rpm_left * math.pi * self.wheel_radius / 30
        v_right = rpm_right * math.pi * self.wheel_radius / 30
        linear_vel = (v_left + v_right) / 2
        angular_vel = (v_right - v_left) / self.robot_base
        return linear_vel, angular_vel

    def calculate_expected_position(self, linear_vel, angular_vel):
        self.expected_x += linear_vel * math.cos(self.expected_theta) * self.time_step
        self.expected_y += linear_vel * math.sin(self.expected_theta) * self.time_step
        self.expected_theta += angular_vel * self.time_step


    def adjust_velocity_based_on_distance(self, linear_vel, angular_vel):
        distance = math.sqrt((self.expected_x - self.current_x) ** 2 + (self.expected_y - self.current_y) ** 2)
        angle_diff = abs(self.expected_theta - self.current_theta)

        linear_scale_factor = min(distance, 1)  
        adjusted_linear_vel = linear_scale_factor * self.max_linear_speed * linear_vel

        angular_scale_factor = min(angle_diff, 1) 
        adjusted_angular_vel = angular_scale_factor * self.max_angular_speed * angular_vel

        return adjusted_linear_vel, adjusted_angular_vel

    def timer_callback(self):
        if self.action_index < len(self.action_sets):
            action = self.action_sets[self.action_index]
            rpm_left, rpm_right = action
            linear_vel, angular_vel = self.rpm_to_velocity(rpm_left, rpm_right)
            self.calculate_expected_position(linear_vel, angular_vel)
            adjusted_linear_vel, adjusted_angular_vel = self.adjust_velocity_based_on_distance(linear_vel, angular_vel)
            
            velocity_message = Twist()
            # velocity_message.linear.x = linear_vel
            # velocity_message.angular.z = angular_vel
            # self.cmd_vel_pub.publish(velocity_message)
            velocity_message.linear.x = adjusted_linear_vel
            velocity_message.angular.z = adjusted_angular_vel
            self.cmd_vel_pub.publish(velocity_message)
            
            self.action_index += 1
        else:
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    action_sets = [(0,0),
                (50, 0),
                (50, 100),
                (50, 0),
                (50, 100),
                (50, 0),
                (50, 100),
                (100, 100),
                (0, 50),
                (50, 0),
                (0, 100),
                (100, 100),
                (100, 0),
                (50, 50),
                (100, 100),
                (50, 0),
                (50, 50),
                (50, 0),
                (0, 50),
                (50, 0),
                (0, 50),
                (50, 0),
                (0, 50),
                (50, 50),
                (50, 50),
                (50, 50),
                (50, 100),
                (50, 50),
                (50, 100),
                (50, 0),
                (0, 50),
                (50, 50),
                (100, 100),
                (50, 0),
                (50, 100),
                (50, 0),
                (50, 100),
                (0, 0)]  # actions from path geneator 
    node = RPMControlNode(action_sets)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()