#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RPMControlNode(Node):
    def __init__(self, action_sets):
        super().__init__('rpm_control_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_sets = action_sets
        self.action_index = 0  # track actions
        self.wheel_radius = 0.033  # wheel radius
        self.robot_base = 0.287  # dist of two wheel 
        
        # Setup timer with 1 second duration in simulation time
        self.timer = self.create_timer(1.0, self.timer_callback)

    def rpm_to_velocity(self, rpm_left, rpm_right):
        # RPM -> linear and angular speed
        linear_vel = (rpm_left + rpm_right) * math.pi * self.wheel_radius / 60
        angular_vel = (rpm_right - rpm_left) * math.pi * self.wheel_radius / (60 * self.robot_base)
        return linear_vel, angular_vel

    def timer_callback(self):
        if self.action_index < len(self.action_sets):
            action = self.action_sets[self.action_index]
            rpm_left, rpm_right = action
            linear_vel, angular_vel = self.rpm_to_velocity( rpm_left, rpm_right)
            
            # Publish
            velocity_message = Twist()
            velocity_message.linear.x = linear_vel
            velocity_message.angular.z = angular_vel
            self.cmd_vel_pub.publish(velocity_message)
            
            self.action_index += 1
        else:
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    action_sets = [(0,0),
                (100, 50),
                (0, 50),
                (100, 50),
                (50, 100),
                (100, 100),
                (50, 0),
                (0, 50),
                (0, 50),
                (50, 50),
                (50, 100),
                (50, 50),
                (100, 0),
                (50, 50),
                (50, 50),
                (50, 50),
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
                (0, 50),
                (50, 0),
                (0, 50),
                (50, 50),
                (0, 50),
                (50, 0),
                (50, 100),
                (50, 50),
                (50, 0),
                (50, 100),
                (100, 100),
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