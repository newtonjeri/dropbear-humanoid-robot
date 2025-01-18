#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JTCPublisher(Node):

    def __init__(self):
        super().__init__('_jtc_publisher_node')
        # Get user input for  and joints
        self.leg:str
        self.joints:str = []

        # Publish topic name
        self.publish_topic:str = '/joint_controllers/joint_trajectory'
        self.joints = ["Joint1"]

        # Creating the trajectory publisher
        self.trajectory_publisher = self.create_publisher(JointTrajectory, self.publish_topic, 10)
        timer_period = 1.0
        # Creating a timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Prompting user for input for each joint
        self.goal_positions = []
        for joint in self.joints:
            try:
                position = float(input(f"Enter the target position for {joint}: "))
                self.goal_positions.append(position)
            except ValueError:
                self.get_logger().warn(f"Invalid input for {joint}. Defaulting to 0.0")
                self.goal_positions.append(0.0)
    
    # Timer callback function
    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info("Trajectory Sent!")

def main(args=None):
    rclpy.init(args=args)
    node = JTCPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
