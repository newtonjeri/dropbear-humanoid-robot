
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Hand Controllers
    right_hand_cont = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_hand_controller",
                "--controller-manager", "/controller_manager",
            ]
    )

    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ]
        )


    return LaunchDescription([
        joint_state_broadcaster,
        # stewart_prismatic_cont,
        right_hand_cont,
        # left_hand_cont,
        # waist_cont,
        # right_leg_cont,
        # left_leg_cont,
    ])