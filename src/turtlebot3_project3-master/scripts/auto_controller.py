#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import math

from nonholonomic_a_star_fn import runAStar, optimizePath

class RPMControlNode(Node):

    def __init__(self):
        super().__init__('RPM_control_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_x', 500.0),
                ('start_y', 1000.0),
                ('start_angle', 0.0),
                ('goal_x', 5750.0),
                ('goal_y', 1000.0),
                ('clearance', 28.0),
                ('rpm1', 10.0),
                ('rpm2', 20.0),
                ('scale', 1/5)
            ]
        )

        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.start_angle = self.get_parameter('start_angle').get_parameter_value().double_value
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.clearance = self.get_parameter('clearance').get_parameter_value().double_value
        self.rpm1 = self.get_parameter('rpm1').get_parameter_value().double_value
        self.rpm2 = self.get_parameter('rpm2').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_sets = []
        self.action_num = 0
        self.actual_start = False
        self.optimize = False
        self.wheel_radius = 0.033  # wheel radius TBD
        self.robot_base = 0.287  # dist of two wheel 
        self.robot_radius = 0.220

        qos_profile = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, history = HistoryPolicy.KEEP_LAST, depth = 10)

        self.current_time = 0
        self.time_diff = 1
        self.clock = self.create_subscription(Clock, 'clock', self.follow_actions, qos_profile)

    def rpm_to_velocity(self, rpm_left, rpm_right):
        # RPM -> linear and angular speed
        linear_vel = (rpm_left + rpm_right) / (1000)
        angular_vel = (rpm_right - rpm_left) * 2 / (self.robot_base * 1000)
        return linear_vel, angular_vel

    def follow_actions(self, msg):
        sim_time = msg.clock.sec + (msg.clock.nanosec * 0.000000001)
        time_diff = sim_time - self.current_time

        if time_diff >= self.time_diff and self.actual_start:
            try:
                # if self.action_num % 2 == 0:
                    # rpm_left = self.action_sets[int(self.action_num / 2)][0]
                    # rpm_right = self.action_sets[int(self.action_num / 2)][1]
                    # self.time_diff = self.action_sets[int(self.action_num / 2)][2]
                # else:
                #     rpm_left  = rpm_right = 0
                #     self.time_diff = 1

                rpm_left = self.action_sets[int(self.action_num)][0]
                rpm_right = self.action_sets[int(self.action_num)][1]
                self.time_diff = self.action_sets[int(self.action_num)][2]

                # if self.action_num == 20:
                #     self.time_diff = 1.0
            except Exception as error:
                raise SystemExit
            linear_vel, angular_vel = self.rpm_to_velocity(rpm_left, rpm_right)
            self.get_logger().info(f"Time travel: {time_diff}")
            # Publish
            velocity_message = Twist()
            velocity_message.linear.x = linear_vel
            velocity_message.angular.z = angular_vel
            self.cmd_vel_pub.publish(velocity_message)
            
            self.action_num += 1
            self.current_time = sim_time

def main(args=None):
    rclpy.init(args=args)

    height = 2000
    width = 6000
    print("Node created")
    node = RPMControlNode()
    print(node.scale, int(node.robot_radius * 1000), int(node.wheel_radius * 1000), int(node.clearance),int(node.robot_base * 1000), int(node.start_x), int(node.start_y), int(node.goal_x), int(node.goal_y), node.rpm1, node.rpm2)
    action_sets, useless = runAStar(height, width, int(node.robot_radius * 1000), int(node.clearance), int(node.robot_base * 1000), (node.wheel_radius * 1000), int(node.start_x), int(node.start_y), node.start_angle, int(node.goal_x), int(node.goal_y), node.rpm1, node.rpm2, node.scale)

    if action_sets == False:
        print("Invalid position entered!")
        exit()
        
    # action_sets = runAStar(2000, 6000, 220, 15, 287, 33, 500, 500, 0, 3000, 300, 50, 100, 1/5)
    action_sets.append((0, 0))

    optimized_actions = optimizePath(action_sets)

    node.action_sets = optimized_actions
    node.actual_start = True
    node.optimize = True

    print("Action sets found")

    try:
        rclpy.spin(node)
    except:
        rclpy.logging.get_logger("Exit Program").info("Execution Complete")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()