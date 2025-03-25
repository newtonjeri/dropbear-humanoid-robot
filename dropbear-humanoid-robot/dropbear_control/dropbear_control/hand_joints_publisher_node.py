#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import math

class HandController(Node):
    def __init__(self):
        super().__init__('hand_controller')

        # Define joint names for each controller (right and left hands)
        self.hand_joints = {
            'right': ['RH_yaw', 'RH_pitch', 'RH_roll', 'RH_wrist_roll'],
            'left': ['LH_yaw', 'LH_pitch', 'LH_roll', 'LH_wrist_roll']
        }
        self.elbow_joints = {
            'right': ['RH_elbow_joint'],
            'left': ['LH_elbow_joint']
        }

        # Publishers for Joint Trajectory Controllers (JTC)
        self.hand_pubs = {
            'right': self.create_publisher(JointTrajectory, '/right_hand_controller/joint_trajectory', 10),
            'left': self.create_publisher(JointTrajectory, '/left_hand_controller/joint_trajectory', 10)
        }

        # Publishers for Effort Controllers
        self.elbow_pubs = {
            'right': self.create_publisher(Float64MultiArray, '/right_hand_elbow_controller/commands', 10),
            'left': self.create_publisher(Float64MultiArray, '/left_hand_elbow_controller/commands', 10)
        }

        # Timers for publishing at different rates (now in separate threads)
        self.hand_timer = self.create_timer(1.0, self.hand_callback)  # 1 Hz for JTC
        self.elbow_timer = self.create_timer(1.0 / 60.0, self.elbow_callback)  # 60 Hz for Effort Controllers

        # Joint angle arrays for hand motion
        self.hand_joint_angles = {
            'right': [0.5, 0.0, 0.0, 0.5],  # angles for right hand
            'left': [0.5, 0.0, 0.0, 0.5]   # angles for left hand
        }
        self.elbow_joint_efforts = {
            'right': [5.0, 0.0, 0.0, 0.0, 0.0],  # efforts for right elbow
            'left': [5.0, 0.0, 0.0, 0.0, 0.0]  # efforts for left elbow
        }

        # Step index and duration
        self.step_index = 0
        self.step_duration = 1.0  # 1 second per step

        # Add thread synchronization if needed
        self.lock = rclpy.Lock()

    def hand_callback(self):
        """Callback for Joint Trajectory Controllers (Hands)."""
        with self.lock:  # Protect shared resources
            for side in ['right', 'left']:
                # Get current step angles
                hand_angles = self.hand_joint_angles[side]

                # Publish hand trajectory
                hand_trajectory = self.create_trajectory(self.hand_joints[side], hand_angles)
                self.hand_pubs[side].publish(hand_trajectory)

            # Increment step index
            self.step_index = (self.step_index + 1) % len(self.hand_joint_angles['right'])

    def elbow_callback(self):
        """Callback for Effort Controllers (Elbows)."""
        with self.lock:  # Protect shared resources
            for side in ['right', 'left']:
                # Get current step efforts
                elbow_effort = self.elbow_joint_efforts[side][self.step_index]

                # Publish elbow effort
                elbow_effort_msg = self.create_effort(self.elbow_joints[side], [elbow_effort])
                self.elbow_pubs[side].publish(elbow_effort_msg)

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
    
    try:
        node = HandController()
        
        # Use MultiThreadedExecutor with 2 threads (one for each timer)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()