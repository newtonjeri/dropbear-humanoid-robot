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

        # Publishers for Joint Trajectory Controllers (JTC)
        self.hip_pubs = {
            'right': self.create_publisher(JointTrajectory, '/right_leg_hip_joint_controller/joint_trajectory', 10),
            'left': self.create_publisher(JointTrajectory, '/left_leg_hip_joint_controller/joint_trajectory', 10)
        }
        self.foot_pubs = {
            'right': self.create_publisher(JointTrajectory, '/right_leg_foot_controller/joint_trajectory', 10),
            'left': self.create_publisher(JointTrajectory, '/left_leg_foot_controller/joint_trajectory', 10)
        }

        # Publishers for Effort Controllers
        self.knee_pubs = {
            'right': self.create_publisher(Float64MultiArray, '/right_leg_knee_controller/commands', 10),
            'left': self.create_publisher(Float64MultiArray, '/left_leg_knee_controller/commands', 10)
        }
        self.lower_leg_pubs = {
            'right': self.create_publisher(Float64MultiArray, '/lower_right_leg_controller/commands', 10),
            'left': self.create_publisher(Float64MultiArray, '/lower_left_leg_controller/commands', 10)
        }

        # Timers for publishing at different rates
        self.create_timer(1.0, self.hip_foot_callback)  # 1 Hz for JTC
        self.create_timer(1.0 / 60.0, self.knee_lower_leg_callback)  # 60 Hz for Effort Controllers

        # Joint angle arrays for walking motion
        self.hip_joint_angles = {
            'right': [0.0, 0.5, 0.0, -0.5, 0.0],  # angles for right hip
            'left': [0.0, -0.5, 0.0, 0.5, 0.0]   # angles for left hip
        }
        self.foot_joint_angles = {
            'right': [0.0, -0.3, 0.0, 0.3, 0.0],  # angles for right foot
            'left': [0.0, 0.3, 0.0, -0.3, 0.0]    # angles for left foot
        }
        self.knee_joint_efforts = {
            'right': [7.0, 0.0, 0.0, 0.0, 0.0],  # efforts for right knee
            'left': [7.0, 0.0, 0.0, 0.0, 0.0]   # efforts for left knee
        }
        self.lower_leg_joint_efforts = {
            'right': [0.0, 0.2, 0.0, -0.2, 0.0],  # efforts for right lower leg
            'left': [0.0, -0.2, 0.0, 0.2, 0.0]    # efforts for left lower leg
        }

        # Step index and duration
        self.step_index = 0
        self.step_duration = 1.0  # 1 second per step

    def hip_foot_callback(self):
        """Callback for Joint Trajectory Controllers (Hip and Foot)."""
        for side in ['right', 'left']:
            # Get current step angles
            hip_angle = self.hip_joint_angles[side][self.step_index]
            foot_angle = self.foot_joint_angles[side][self.step_index]

            # Publish hip trajectory
            hip_trajectory = self.create_trajectory(self.hip_joints[side], [hip_angle])
            self.hip_pubs[side].publish(hip_trajectory)

            # Publish foot trajectory
            foot_trajectory = self.create_trajectory(self.foot_joints[side], [foot_angle])
            self.foot_pubs[side].publish(foot_trajectory)

        # Increment step index
        self.step_index = (self.step_index + 1) % len(self.hip_joint_angles['right'])

    def knee_lower_leg_callback(self):
        """Callback for Effort Controllers (Knee and Lower Leg)."""
        for side in ['right', 'left']:
            # Get current step efforts
            knee_effort = self.knee_joint_efforts[side][self.step_index]
            lower_leg_effort = self.lower_leg_joint_efforts[side][self.step_index]

            # Publish knee effort
            knee_effort_msg = self.create_effort(self.knee_joints[side], [knee_effort])
            self.knee_pubs[side].publish(knee_effort_msg)

            # Publish lower leg effort
            lower_leg_effort_msg = self.create_effort(self.lower_leg_joints[side], [lower_leg_effort])
            self.lower_leg_pubs[side].publish(lower_leg_effort_msg)

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