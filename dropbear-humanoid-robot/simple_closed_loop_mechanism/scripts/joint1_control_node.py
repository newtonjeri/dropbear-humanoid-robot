#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tkinter import Tk, Scale, HORIZONTAL, Button, Label


class Joint1ControllerGUI(Node):
    def __init__(self):
        super().__init__('joint1_controller_gui')

        # Set the topic for the joint trajectory controller
        self.publish_topic = '/joint_controllers/joint_trajectory'

        # Create a publisher to the joint trajectory controller's command topic
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            self.publish_topic,
            10
        )

        # Initialize GUI window
        self.root = Tk()
        self.root.title("Joint1 Controller GUI")

        # Create a label and a slider for Joint1
        self.label = Label(self.root, text="Joint1 Position (radians)")
        self.label.pack()

        self.slider = Scale(
            self.root,
            from_=-3.14,
            to=3.14,
            resolution=0.01,
            orient=HORIZONTAL,
            length=300
        )
        self.slider.pack()

        # Add a Send button
        self.send_button = Button(self.root, text="Send Command", command=self.send_command)
        self.send_button.pack()

    def send_command(self):
        # Create a JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['Joint1']

        point = JointTrajectoryPoint()
        position = self.slider.get()

        # Set the joint position
        point.positions = [position]

        # Set the time duration for the trajectory point
        point.time_from_start = Duration(sec=1)

        # Append the point to the trajectory message
        trajectory_msg.points.append(point)

        # Publish the message
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info(f"Published Joint1 Position: {position:.2f} radians")

    def run(self):
        # Run the Tkinter main loop
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    gui = Joint1ControllerGUI()

    try:
        gui.run()
    except KeyboardInterrupt:
        gui.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
