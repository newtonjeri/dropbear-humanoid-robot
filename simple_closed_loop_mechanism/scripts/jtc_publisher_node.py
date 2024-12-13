#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tkinter import Tk, Scale, HORIZONTAL, Button, Label
import time

class JointControllerGUI(Node):
    def __init__(self):
        super().__init__('joint_controller_gui')
        
        # Create a publisher to the joint_trajectory_controller's command topic
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_controllers/joint_trajectory_controller/commands', 10)
        
        # Initialize GUI window
        self.root = Tk()
        self.root.title("Joint Controller GUI")

        # Joint names
        self.joints = ['Joint1', 'Joint2', 'Joint3']

        # Create sliders and store their references
        self.sliders = {}
        self.labels = {}

        for joint in self.joints:
            label = Label(self.root, text=f"{joint} Position")
            label.pack()

            slider = Scale(self.root, from_=-3.14, to=3.14, resolution=0.01, orient=HORIZONTAL, length=300)
            slider.pack()

            self.labels[joint] = label
            self.sliders[joint] = slider

        # Add a Send button
        send_button = Button(self.root, text="Send Command", command=self.send_command)
        send_button.pack()

    def send_command(self):
        # Create a JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = self.joints

        point = JointTrajectoryPoint()
        positions = [self.sliders[joint].get() for joint in self.joints]
        
        point.positions = positions
        point.time_from_start.sec = 1  # Command to execute in 1 second

        msg.points.append(point)

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Joint Positions: {positions}")

    def run(self):
        # Run the Tkinter main loop
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    gui = JointControllerGUI()

    try:
        gui.run()
    except KeyboardInterrupt:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
