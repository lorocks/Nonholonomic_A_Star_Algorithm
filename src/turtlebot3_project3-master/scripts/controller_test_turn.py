#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class RPMControlNode(Node):
    def __init__(self, action_sets):
        super().__init__('rpm_control_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.action_sets = action_sets
        self.action_index = 0  # track actions
        self.wheel_radius = 0.033  # wheel radius
        self.robot_base = 0.288  # dist of two wheel 
        
        # Setup timer with 1 second duration in simulation time
        self.timer = self.create_timer(1.0, self.timer_callback)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        x_int = int(position.x*1000+500)
        y_int = int(position.y*1000+1000)
        self.get_logger().info(f'Position : (X: {x_int}, Y: {y_int})')

        
    def rpm_to_velocity(self, rpm_left, rpm_right):
        v_left = rpm_left * math.pi * self.wheel_radius / 30
        v_right = rpm_right * math.pi * self.wheel_radius / 30
        omega = (v_right - v_left) / self.robot_base
        angular_vel = (v_right - v_left) / self.robot_base

        if v_left != v_right:
            R = (self.robot_base / 2) * ((v_left + v_right) / (v_right - v_left))
            linear_vel = omega * R
        else:
            linear_vel = v_left
        # RPM -> linear and angular speed
        # linear_vel = (rpm_left + rpm_right) * math.pi * self.wheel_radius / (60*2)
        
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
                   (30*(1+0.288/2)/0.033, 30*(1-0.288/2)/0.033),
                # (50, 0),
                # (50, 100),
                # (50, 0),
                # (50, 100),
                # (50, 0),
                # (50, 100),
                # (100, 100),
                # (0, 50),
                # (50, 0),
                # (0, 100),
                # (100, 100),
                # (100, 0),
                # (50, 50),
                # (100, 100),
                # (50, 0),
                # (50, 50),
                # (50, 0),
                # (0, 50),
                # (50, 0),
                # (0, 50),
                # (50, 0),
                # (0, 50),
                # (50, 50),
                # (50, 50),
                # (50, 50),
                # (50, 100),
                # (50, 50),
                # (50, 100),
                # (50, 0),
                # (0, 50),
                # (50, 50),
                # (100, 100),
                # (50, 0),
                # (50, 100),
                # (50, 0),
                # (50, 100),
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