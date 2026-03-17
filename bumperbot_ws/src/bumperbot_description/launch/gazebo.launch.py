#!/usr/bin/env python3
# =============================================================================
# gazebo.launch.py  —  bumperbot_description
# =============================================================================
# Simulation-only launch file. NOT used on the real robot.
# Starts a Gazebo (GZ/Ignition) world, spawns the bumperbot robot model into
# it, and bridges select Gazebo topics back into ROS 2:
#   - /clock  → keeps ROS nodes synchronised to simulation time
#   - /imu    → simulated IMU data (remapped to /imu/out to match real robot)
# Automatically selects between Ignition Gazebo (ROS Humble) and GZ Sim (newer)
# based on the ROS_DISTRO environment variable.
# =============================================================================

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Resolve the installed share directory for this package
    bumperbot_description = get_package_share_directory("bumperbot_description")

    # Allow the URDF path to be overridden at launch time (defaults to the
    # standard xacro file in the installed share directory).
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    # GZ_SIM_RESOURCE_PATH tells Gazebo where to find the package's meshes and
    # other resources. We point it at the parent of the share directory so
    # Gazebo can resolve package:// URIs in the URDF.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(bumperbot_description).parent.resolve())
            ]
        )
    
    # Detect ROS distro to pick the right Gazebo backend.
    # "humble" uses Ignition Gazebo; newer distros use GZ Sim.
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    # Process the Xacro into a URDF string, injecting is_ignition so
    # bumperbot_gazebo.xacro uses the matching sensor plugin.
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    # Publishes TF tree from the URDF joints. use_sim_time=True means it
    # stamps messages with Gazebo /clock rather than wall clock.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    # Start the Gazebo simulator with an empty world at verbosity level 4.
    # -r starts simulation running immediately; -v 4 shows info messages.
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4", " -r", " empty.sdf"]
                    )
                ]
             )

    # Spawn the bumperbot entity into the running Gazebo world by reading
    # the robot_description topic (published by robot_state_publisher above).
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot"],
    )

    # Bridge Gazebo topics into ROS 2 using ros_gz_bridge.
    # /clock      — simulation time needed by any node using use_sim_time=True
    # /imu        — simulated IMU accelerometer/gyroscope data
    # The /imu topic is remapped to /imu/out to match the real MPU6050 driver's
    # output topic, allowing the localization stack to work unchanged.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ('/imu', '/imu/out'),   # match real robot topic name
        ]
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])