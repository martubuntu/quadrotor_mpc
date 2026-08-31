from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("uav_mpc")
    real_config = PathJoinSubstitution([package_share, "config", "mpc_para.yaml"])
    sim_config = PathJoinSubstitution([package_share, "config", "mpc_simulation.yaml"])

    is_sim = LaunchConfiguration("is_sim")
    start_trajectory = LaunchConfiguration("start_trajectory")
    use_eso = LaunchConfiguration("use_eso")
    radius = LaunchConfiguration("radius")
    linear_speed = LaunchConfiguration("linear_speed")
    takeoff_height = LaunchConfiguration("takeoff_height")
    start_delay_sec = LaunchConfiguration("start_delay_sec")

    return LaunchDescription([
        # -------------------------------------------------------------
        # Unified launch arguments
        # -------------------------------------------------------------
        DeclareLaunchArgument(
            "is_sim", default_value="false",
            description="Toggle simulation mode: enables use_sim_time, auto-arm, auto-offboard and auto-takeoff."
        ),
        DeclareLaunchArgument(
            "start_trajectory", default_value="false",
            description="Start trajectory tracking (Phase 2 circle). False for hover only."
        ),
        DeclareLaunchArgument(
            "use_eso", default_value="false",
            description="Accept /eso/disturbance input."
        ),
        DeclareLaunchArgument(
            "radius", default_value="1.5",
            description="Radius of the circular trajectory in meters."
        ),
        DeclareLaunchArgument(
            "linear_speed", default_value="0.20",
            description="Linear speed of the circular trajectory in m/s."
        ),
        DeclareLaunchArgument(
            "takeoff_height", default_value="1.5",
            description="Target takeoff/hover altitude in meters (active when is_sim:=true)."
        ),
        DeclareLaunchArgument(
            "start_delay_sec", default_value="6.0",
            description="Hover duration before transitioning to Phase 2 trajectory in simulation."
        ),

        # -------------------------------------------------------------
        # Real Flight Group (is_sim == false, Manual Safe Controls)
        # -------------------------------------------------------------
        GroupAction(
            condition=UnlessCondition(is_sim),
            actions=[
                Node(
                    package="uav_mpc",
                    executable="mpc_node",
                    name="uav_mpc_node",
                    output="screen",
                    parameters=[
                        real_config,
                        {
                            "use_sim_time": False,
                            "auto_arm": False,
                            "auto_offboard": False,
                            "takeoff_height": 0.0,
                            "use_eso": ParameterValue(use_eso, value_type=bool),
                        },
                    ],
                ),
                Node(
                    condition=IfCondition(start_trajectory),
                    package="uav_mpc",
                    executable="circle_traj_node",
                    name="circle_traj_node",
                    output="screen",
                    parameters=[
                        real_config,
                        {
                            "use_sim_time": False,
                            "start_delay_sec": 0.0,
                            "radius": ParameterValue(radius, value_type=float),
                            "linear_speed": ParameterValue(linear_speed, value_type=float),
                        },
                    ],
                ),
            ]
        ),

        # -------------------------------------------------------------
        # Simulation Group (is_sim == true, Autonomous Phase 1 -> 2)
        # -------------------------------------------------------------
        GroupAction(
            condition=IfCondition(is_sim),
            actions=[
                Node(
                    package="uav_mpc",
                    executable="mpc_node",
                    name="uav_mpc_node",
                    output="screen",
                    parameters=[
                        sim_config,
                        {
                            "use_sim_time": True,
                            "auto_arm": True,
                            "auto_offboard": True,
                            "takeoff_height": ParameterValue(takeoff_height, value_type=float),
                            "use_eso": ParameterValue(use_eso, value_type=bool),
                        },
                    ],
                ),
                Node(
                    condition=IfCondition(start_trajectory),
                    package="uav_mpc",
                    executable="circle_traj_node",
                    name="circle_traj_node",
                    output="screen",
                    parameters=[
                        sim_config,
                        {
                            "use_sim_time": True,
                            "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                            "radius": ParameterValue(radius, value_type=float),
                            "linear_speed": ParameterValue(linear_speed, value_type=float),
                            "height": ParameterValue(takeoff_height, value_type=float),
                        },
                    ],
                ),
            ]
        ),
    ])
