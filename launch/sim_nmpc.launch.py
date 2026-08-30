from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("uav_mpc")
    config_file = PathJoinSubstitution(
        [package_share, "config", "mpc_simulation.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_trajectory = LaunchConfiguration("start_trajectory")
    use_eso = LaunchConfiguration("use_eso")
    auto_arm = LaunchConfiguration("auto_arm")
    auto_offboard = LaunchConfiguration("auto_offboard")
    takeoff_height = LaunchConfiguration("takeoff_height")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    radius = LaunchConfiguration("radius")
    linear_speed = LaunchConfiguration("linear_speed")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use Gazebo simulation clock."
        ),
        DeclareLaunchArgument(
            "start_trajectory", default_value="true",
            description="Enable Phase 2 circle trajectory tracking after Phase 1 takeoff."
        ),
        DeclareLaunchArgument(
            "use_eso", default_value="false",
            description="Accept /eso/disturbance input in simulation."
        ),
        DeclareLaunchArgument(
            "auto_arm", default_value="true",
            description="Automatically send arming command to MAVROS in simulation."
        ),
        DeclareLaunchArgument(
            "auto_offboard", default_value="true",
            description="Automatically switch to OFFBOARD mode in simulation."
        ),
        DeclareLaunchArgument(
            "takeoff_height", default_value="1.5",
            description="Target altitude for Phase 1 takeoff and hover in meters."
        ),
        DeclareLaunchArgument(
            "start_delay_sec", default_value="6.0",
            description="Hover duration before transitioning from Phase 1 (Takeoff) to Phase 2 (Trajectory)."
        ),
        DeclareLaunchArgument(
            "radius", default_value="1.5",
            description="Radius of the circular trajectory in meters."
        ),
        DeclareLaunchArgument(
            "linear_speed", default_value="0.30",
            description="Linear speed of the circular trajectory in m/s."
        ),

        Node(
            package="uav_mpc",
            executable="mpc_node",
            name="uav_mpc_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "use_eso": ParameterValue(use_eso, value_type=bool),
                    "auto_arm": ParameterValue(auto_arm, value_type=bool),
                    "auto_offboard": ParameterValue(auto_offboard, value_type=bool),
                    "takeoff_height": ParameterValue(takeoff_height, value_type=float),
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
                config_file,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                    "radius": ParameterValue(radius, value_type=float),
                    "linear_speed": ParameterValue(linear_speed, value_type=float),
                    "height": ParameterValue(takeoff_height, value_type=float),
                },
            ],
        ),
    ])
