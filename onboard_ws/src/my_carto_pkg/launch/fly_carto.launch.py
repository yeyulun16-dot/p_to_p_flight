"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ## ***** Launch arguments *****
    default_lidar_params = os.path.join(
        FindPackageShare('bluesea2').find('bluesea2'), 'params', 'uart_lidar.yaml'
    )
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    lidar_node = Node(
        package='bluesea2',
        executable='bluesea2_node',
        name='bluesea_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_params_file],
    )

  ## ***** File paths ******
    pkg_share = FindPackageShare('my_carto_pkg').find('my_carto_pkg')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'fly.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'cartographer.rviz')
    use_rviz = LaunchConfiguration('use_rviz')

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False}],
        output = 'screen'
        )


    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', FindPackageShare('my_carto_pkg').find('my_carto_pkg') + '/configuration_files',
            '-configuration_basename', 'amphi.lua'],
        remappings = [
            ('scan', 'scan')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz for live mapping visualization.'
        ),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=default_lidar_params,
            description='BlueSea UART lidar parameter file.'
        ),
        # Step 1: Launch lidar node
        TimerAction(
            period=0.0,  # Immediately
            actions=[lidar_node]
        ),
        # Step 2: Launch robot_state_publisher_node after 2 seconds
        TimerAction(
            period=1.0,
            actions=[robot_state_publisher_node]
        ),
        # Step 3: Launch cartographer_node after 10 seconds
        TimerAction(
            period=10.0,
            actions=[cartographer_node, cartographer_occupancy_grid_node]
        ),
        # Step 4: Launch rviz2
        TimerAction(
            period=12.0,
            actions=[rviz_node]
        ),
    ])
