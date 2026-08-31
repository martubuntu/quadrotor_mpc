from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("uav_mpc")
    main_launch = PathJoinSubstitution([package_share, "launch", "real_nmpc.launch.py"])

    start_trajectory = LaunchConfiguration("start_trajectory")
    use_eso = LaunchConfiguration("use_eso")
    takeoff_height = LaunchConfiguration("takeoff_height")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    radius = LaunchConfiguration("radius")
    linear_speed = LaunchConfiguration("linear_speed")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_trajectory", default_value="true",
            description="Enable Phase 2 circle trajectory tracking after Phase 1 takeoff in simulation."
        ),
        DeclareLaunchArgument(
            "use_eso", default_value="false",
            description="Accept /eso/disturbance input in simulation."
        ),
        DeclareLaunchArgument(
            "takeoff_height", default_value="1.5",
            description="Target altitude for Phase 1 takeoff and hover in meters."
        ),
        DeclareLaunchArgument(
            "start_delay_sec", default_value="6.0",
            description="Hover duration in seconds before transitioning from Phase 1 (Takeoff) to Phase 2 (Trajectory)."
        ),
        DeclareLaunchArgument(
            "radius", default_value="1.5",
            description="Radius of the circular trajectory in meters."
        ),
        DeclareLaunchArgument(
            "linear_speed", default_value="0.30",
            description="Linear speed of the circular trajectory in m/s."
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={
                "is_sim": "true",
                "start_trajectory": start_trajectory,
                "use_eso": use_eso,
                "takeoff_height": takeoff_height,
                "start_delay_sec": start_delay_sec,
                "radius": radius,
                "linear_speed": linear_speed,
            }.items(),
        ),
    ])
