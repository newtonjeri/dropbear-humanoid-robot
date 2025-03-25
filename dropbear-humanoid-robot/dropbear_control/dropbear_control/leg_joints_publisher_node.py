#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import math

class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller')

        # Define joint names for each controller (right and left legs)
        self.hip_joints = {
            'right': ['RL_hip_joint'],
            'left': ['LL_hip_joint']
        }
        self.foot_joints = {
            'right': ['RL_Revolute87'],
            'left': ['LL_Revolute87']
        }
        self.knee_joints = {
            'right': ['RL_knee_actuator_joint'],
            'left': ['LL_knee_actuator_joint']
        }
        self.lower_leg_joints = {
            'right': ['RL_Revolute67'],
            'left': ['LL_Revolute67']
        }

        # Publishers
        self.hip_pubs = {
            'right': self.create_publisher(JointTrajectory, '/right_leg_hip_joint_controller/joint_trajectory', 10),
            'left': self.create_publisher(JointTrajectory, '/left_leg_hip_joint_controller/joint_trajectory', 10)
        }
        self.foot_pubs = {
            'right': self.create_publisher(JointTrajectory, '/right_leg_foot_controller/joint_trajectory', 10),
            'left': self.create_publisher(JointTrajectory, '/left_leg_foot_controller/joint_trajectory', 10)
        }
        self.knee_pubs = {
            'right': self.create_publisher(Float64MultiArray, '/right_leg_knee_controller/commands', 10),
            'left': self.create_publisher(Float64MultiArray, '/left_leg_knee_controller/commands', 10)
        }
        self.lower_leg_pubs = {
            'right': self.create_publisher(Float64MultiArray, '/lower_right_leg_controller/commands', 10),
            'left': self.create_publisher(Float64MultiArray, '/lower_left_leg_controller/commands', 10)
        }

        # Single timer that runs at the higher frequency (60Hz)
        self.timer = self.create_timer(1.0 / 60.0, self.combined_callback)

        # Joint angle arrays for walking motion
        self.hip_joint_angles = {
            'right': [0.5, -0.5, 0.5, -0.5, 0.5,],
            'left': [-0.5, 0.5, -0.5, 0.5, -0.0]
        }
        self.foot_joint_angles = {
            'right': [0.0, -0.3, 0.0, 0.3, 0.0],
            'left': [0.0, 0.3, 0.0, -0.3, 0.0]
        }
        self.knee_joint_efforts = {
            'right': [7.0, 5.0, 2.0, 5.0, 7.0],
            'left': [7.0, 5.0, 2.0, 5.0, 7.0]
        }
        self.lower_leg_joint_efforts = {
            'right': [0.0, 0.2, 0.0, -0.2, 0.0],
            'left': [0.0, -0.2, 0.0, 0.2, 0.0]
        }

        # Counters to manage different rates
        self.step_index = 0
        self.high_freq_counter = 0
        self.step_duration = 0.5  # 1 second per step

    def combined_callback(self):
        """Combined callback that handles both controllers at different rates."""
        # Always run the high frequency (60Hz) effort controllers
        for side in ['right', 'left']:
            knee_effort = self.knee_joint_efforts[side][self.step_index]
            lower_leg_effort = self.lower_leg_joint_efforts[side][self.step_index]
            
            knee_effort_msg = self.create_effort(self.knee_joints[side], [knee_effort])
            self.knee_pubs[side].publish(knee_effort_msg)
            
            lower_leg_effort_msg = self.create_effort(self.lower_leg_joints[side], [lower_leg_effort])
            self.lower_leg_pubs[side].publish(lower_leg_effort_msg)

        # Only run the trajectory controllers every 60th call (1Hz)
        self.high_freq_counter += 1
        if self.high_freq_counter >= 60:
            self.high_freq_counter = 0
            for side in ['right', 'left']:
                hip_angle = self.hip_joint_angles[side][self.step_index]
                foot_angle = self.foot_joint_angles[side][self.step_index]
                
                hip_trajectory = self.create_trajectory(self.hip_joints[side], [hip_angle])
                self.hip_pubs[side].publish(hip_trajectory)
                
                foot_trajectory = self.create_trajectory(self.foot_joints[side], [foot_angle])
                self.foot_pubs[side].publish(foot_trajectory)

            # Increment step index after completing a full cycle
            self.step_index = (self.step_index + 1) % len(self.hip_joint_angles['right'])

    def create_trajectory(self, joint_names, positions):
        """Create a JointTrajectory message."""
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.duration.Duration(seconds=self.step_duration).to_msg()
        trajectory.points.append(point)
        return trajectory

    def create_effort(self, joint_names, efforts):
        """Create a Float64MultiArray message for effort controllers."""
        effort_msg = Float64MultiArray()
        effort_msg.data = efforts
        return effort_msg

def main(args=None):
    rclpy.init(args=args)
    node = LegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()