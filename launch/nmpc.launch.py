from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("uav_mpc")
    config_file = PathJoinSubstitution([package_share, "config", "mpc_para.yaml"])

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
            description="Toggle simulation mode: false for real flight (manual), true for Gazebo SITL (autonomous)."
        ),
        DeclareLaunchArgument(
            "start_trajectory", default_value="false",
            description="Enable Phase 2 circle trajectory tracking. Default false for hover only."
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
            description="Target altitude for Phase 1 takeoff and hover in meters (simulation)."
        ),
        DeclareLaunchArgument(
            "start_delay_sec", default_value="10.0",
            description="Hover duration before transitioning from Phase 1 to Phase 2 in simulation."
        ),

        # -------------------------------------------------------------
        # NMPC Controller Node (Always active: manages hover & tracking)
        # -------------------------------------------------------------
        Node(
            package="uav_mpc",
            executable="mpc_node",
            name="uav_mpc_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "is_sim": ParameterValue(is_sim, value_type=bool),
                    "use_sim_time": ParameterValue(is_sim, value_type=bool),
                    "use_eso": ParameterValue(use_eso, value_type=bool),
                    "takeoff_height": ParameterValue(takeoff_height, value_type=float),
                },
            ],
        ),

        # -------------------------------------------------------------
        # Trajectory Generator Node (Optional in launch or run independently)
        # -------------------------------------------------------------
        Node(
            condition=IfCondition(start_trajectory),
            package="uav_mpc",
            executable="circle_traj_node",
            name="circle_traj_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "is_sim": ParameterValue(is_sim, value_type=bool),
                    "use_sim_time": ParameterValue(is_sim, value_type=bool),
                    "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                    "radius": ParameterValue(radius, value_type=float),
                    "linear_speed": ParameterValue(linear_speed, value_type=float),
                    "height": ParameterValue(takeoff_height, value_type=float),
                },
            ],
        ),
    ])
