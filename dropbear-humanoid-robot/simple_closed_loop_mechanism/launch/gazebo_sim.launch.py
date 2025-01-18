import os
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

description_pkg = "simple_closed_loop_mechanism"
xacro_filename = "simple_closed_loop_mechanism.urdf.xacro"
def generate_launch_description():
    # Path to xacro file
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', xacro_filename)
    # model_arg
    model_args = DeclareLaunchArgument(
        name = "model",
        default_value = xacro_file,
        description = "Absolute path to robot urdf"
    )

    # Environment Variable
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_prefix(description_pkg), "share")

    # robot_description
    robot_description = ParameterValue(Command(
            ['xacro ', LaunchConfiguration("model")]
        )
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        arguments=[xacro_file],
    )
    
    start_gazebo_server_cmd = ExecuteProcess(
       cmd=[
            'gzserver',
            '-s',
            '--verbose',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
            # "-u" # This ensures Gazebo starts paused
        ],        
        output='screen',
    )
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=[
            'gzclient',
            # '-s',
            # 'libgazebo_ros_init.so',
            # '-s',
            # 'libgazebo_ros_factory.so',
        ],
        output='screen',)
    # gazebo spawn entity
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "simple_closed_loop_mechanism", "-topic", "robot_description"],
        output="screen",
    )

    joint_controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
                "joint_controllers",
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
        model_args,
        robot_state_publisher,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
        joint_controllers,
        joint_state_broadcaster,  # This node is needed for joint state publishing in rviz2 and other ROS 2 tools.
    ])