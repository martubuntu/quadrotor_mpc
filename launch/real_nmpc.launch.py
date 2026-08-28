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
        [package_share, "config", "mpc_para.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_trajectory = LaunchConfiguration("start_trajectory")
    use_eso = LaunchConfiguration("use_eso")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock. Keep false on the real aircraft."
        ),
        DeclareLaunchArgument(
            "start_trajectory", default_value="false",
            description="Start the circle reference generator. Default false for safe hover testing."
        ),
        DeclareLaunchArgument(
            "use_eso", default_value="false",
            description="Accept /eso/disturbance input. This launch does not start an ESO node."
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
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
