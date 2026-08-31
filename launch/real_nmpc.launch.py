from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        DeclareLaunchArgument(
            "is_sim", default_value="false",
            description="Enable Gazebo SITL simulation mode (auto arm, auto offboard, sim clock and sim parameters)."
        ),
        DeclareLaunchArgument(
            "start_trajectory", default_value="false",
            description="Start the circle reference generator. In simulation mode, transitions automatically after takeoff and hover."
        ),
        DeclareLaunchArgument(
            "use_eso", default_value="false",
            description="Accept /eso/disturbance input. Default false."
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
            description="Target takeoff altitude in meters (active when is_sim:=true)."
        ),
        DeclareLaunchArgument(
            "start_delay_sec", default_value="6.0",
            description="Hover stabilization duration in seconds before starting trajectory in simulation mode."
        ),

        # 1. Real Flight Controller Node (is_sim == false)
        Node(
            condition=UnlessCondition(is_sim),
            package="uav_mpc",
            executable="mpc_node",
            name="uav_mpc_node",
            output="screen",
            parameters=[
                real_config,
                {
                    "use_sim_time": False,
                    "use_eso": ParameterValue(use_eso, value_type=bool),
                },
            ],
        ),

        # 2. Gazebo Simulation Controller Node (is_sim == true)
        Node(
            condition=IfCondition(is_sim),
            package="uav_mpc",
            executable="mpc_node",
            name="uav_mpc_node",
            output="screen",
            parameters=[
                sim_config,
                {
                    "use_sim_time": True,
                    "use_eso": ParameterValue(use_eso, value_type=bool),
                    "auto_arm": True,
                    "auto_offboard": True,
                    "takeoff_height": ParameterValue(takeoff_height, value_type=float),
                },
            ],
        ),

        # 3. Real Flight Trajectory Node (start_trajectory == true && is_sim == false)
        Node(
            condition=IfCondition(
                PythonExpression(["'", start_trajectory, "' == 'true' and '", is_sim, "' != 'true'"])
            ),
            package="uav_mpc",
            executable="circle_traj_node",
            name="circle_traj_node",
            output="screen",
            parameters=[
                real_config,
                {
                    "use_sim_time": False,
                    "radius": ParameterValue(radius, value_type=float),
                    "linear_speed": ParameterValue(linear_speed, value_type=float),
                },
            ],
        ),

        # 4. Gazebo Simulation Trajectory Node (start_trajectory == true && is_sim == true)
        Node(
            condition=IfCondition(
                PythonExpression(["'", start_trajectory, "' == 'true' and '", is_sim, "' == 'true'"])
            ),
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
    ])
