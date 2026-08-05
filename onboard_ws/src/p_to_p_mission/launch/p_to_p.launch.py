import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("p_to_p_mission")
    default_params = os.path.join(package_share, "config", "p_to_p.yaml")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    lidar_params_file = LaunchConfiguration("lidar_params_file")

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("my_carto_pkg"),
                "launch",
                "fly_carto.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "lidar_params_file": lidar_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Point-to-point configuration YAML",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with Cartographer",
            ),
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value=os.path.join(
                    get_package_share_directory("bluesea2"),
                    "params",
                    "uart_lidar.yaml",
                ),
                description="BlueSea UART lidar parameter YAML",
            ),
            mapping_launch,
            Node(
                package="pid_control_pkg",
                executable="position_pid_controller",
                name="position_pid_controller",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="uart_to_stm32",
                executable="uart_to_stm32_node",
                name="uart_to_stm32_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="p_to_p_mission",
                executable="p_to_p_mission_node",
                name="p_to_p_mission",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
