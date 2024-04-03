#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RPMControlNode(Node):

    def _init_(self, action_sets):
        super()._init_('rpm_control_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_sets = action_sets
        self.wheel_radius = 0.033  # wheel radius TBD
        self.robot_base = 0.287  # dist of two wheel 

    def rpm_to_velocity(self, rpm_left, rpm_right):
        # RPM -> linear and angular speed
        linear_vel = (rpm_left + rpm_right) * math.pi * self.wheel_radius / 60
        angular_vel = (rpm_right - rpm_left) * math.pi * self.wheel_radius / (30 * self.robot_base)
        return linear_vel, angular_vel

    def follow_actions(self):
        for action in self.action_sets:
            rpm_left, rpm_right = action
            linear_vel, angular_vel = self.rpm_to_velocity(rpm_left, rpm_right)
            
            # Publish
            velocity_message = Twist()
            velocity_message.linear.x = linear_vel
            velocity_message.angular.z = angular_vel
            self.cmd_vel_pub.publish(velocity_message)
            
            # set of time for each action
            rclpy.spin_once(self, timeout_sec=1)

def main(args=None):
    rclpy.init(args=args)
    action_sets = [[50,0], [100,100], [0,50], [0,50], [100,50], [100,100], [0,100], [100,0]]  # actions drom path geneator 
    node = RPMControlNode(action_sets)
    node.follow_actions()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()