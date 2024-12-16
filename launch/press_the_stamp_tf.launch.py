# Copyright 2023 Keitaro Nakamura
# Copyright 2024 Akira Matsumoto
# SPDX-License-Identifier: Apache 2.0
# このファイルはKeitaro Nakamuraによって作成されたcamera_picking.launch.pyをコピーし, 一部を改変したものです.
# 変更内容
#   color_detection : fixed_position_publisher
#   crane_x7_simple_example : crane_x7_stamp
#   pick_and_move_tf : press_the_stamp_tf

import os

from ament_index_python.packages import get_package_share_directory
from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    robot_description_semantic_config = load_file(
        'crane_x7_moveit_config', 'config/crane_x7.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('crane_x7_moveit_config', 'config/kinematics.yaml')

    # Gazebo用
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )
    # アームの動きのコードを起動するのはこっち
    picking_node = Node(# name="pick_and_move_tf",
                        package='crane_x7_stamp',
                        executable='press_the_stamp_tf',
                        output='screen',
                        parameters=[{'robot_description': description_loader.load()},
                                    robot_description_semantic,
                                    kinematics_yaml])
    # 固定座標を送るコードを起動するのはこっち
    detection_node = Node(# name='fixed_position_publisher'
                          package='crane_x7_stamp',
                          executable='fixed_position_publisher',
                          output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        picking_node,
        detection_node,
    ])
