#!/usr/bin/env python3
# =============================================================================
# controller.launch.py  —  bumperbot_controller
# =============================================================================
# Flexible launch file that starts the motion-control stack.
# Called by both real_robot.launch.py and simulated_robot.launch.py with
# different argument overrides. On the real robot:
#   use_simple_controller=False  → uses bumperbot_controller (ros2_control)
#   use_python=False             → uses the C++ noisy_controller executable
#
# Three mutually exclusive configurations are available:
#   A) simple_controller (use_simple_controller=True)
#      → simple_velocity_controller + simple_controller node
#      → publishes clean odometry on /bumperbot_controller/odom
#   B) noisy_controller via bumperbot_controller (use_simple_controller=False)
#      → bumperbot_controller (velocity PID) + noisy_controller node
#      → publishes noisy odometry on /bumperbot_controller/odom_noisy
#      → this is the configuration used on the real robot
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def noisy_controller(context, *args, **kwargs):
    """OpaqueFunction that creates the noisy_controller node at launch time.

    Using OpaqueFunction lets us read concrete float values for wheel geometry
    (needed to add the error offset) rather than lazy substitution objects.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")
    # Perform the substitution now so we can do arithmetic on the values
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    # Python variant — selected when use_python:=True
    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,      # slightly wrong radius
             "wheel_separation": wheel_separation + wheel_separation_error,  # slightly wrong separation
             "use_sim_time": use_sim_time}],
        condition=IfCondition(use_python),
    )

    # C++ variant — selected when use_python:=False (default on real robot)
    noisy_controller_cpp = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_python),
    )

    return [
        noisy_controller_py,
        noisy_controller_cpp,
    ]



def generate_launch_description():
    # ---- Launch Arguments ------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",           # True for sim, overridden to False for real robot
    )
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",           # False on real robot (use velocity controller)
    )
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",          # False on real robot (use C++ nodes)
    )
    # Nominal wheel geometry — changed slightly for the noisy controller to
    # simulate calibration error
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",          # metres
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",           # metres
    )
    # Offsets added to the nominal values to create intentional model error
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # ---- Always-on Controllers -------------------------------------------
    # Reads joint positions/velocities from hardware and publishes /joint_states.
    # Required by both simple and noisy controller paths.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # ---- Path B: bumperbot_controller velocity controller ----------------
    # Used on the real robot (use_simple_controller=False).
    # Applies PID-based velocity control per wheel.
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bumperbot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    # ---- Path A: simple_velocity_controller + simple_controller ----------
    # Tutorial/simulation path (use_simple_controller=True).
    # Uses a direct per-axis velocity interface and a clean odometry node.
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", 
                        "--controller-manager", 
                        "/controller_manager"
                ]
            ),
            # Python variant of simple_controller
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time}],
                condition=IfCondition(use_python),
            ),
            # C++ variant of simple_controller
            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time}],
                condition=UnlessCondition(use_python),
            ),
        ]
    )

    # Deferred function call — resolves wheel geometry values at runtime
    # then creates the noisy_controller node with intentional model error applied
    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_simple_controller_arg,
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
            noisy_controller_launch,
        ]
    )