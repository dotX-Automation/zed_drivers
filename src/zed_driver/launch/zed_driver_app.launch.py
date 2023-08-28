"""
ZED Driver app launch file.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

Jume 20, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('zed_driver'),
        'config',
        'zed_driver.yaml')

    # Declare launch arguments
    nm = LaunchConfiguration('name')
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    nm_launch_arg = DeclareLaunchArgument(
        'name',
        default_value='zed_driver')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='zed')
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config)
    ld.add_action(nm_launch_arg)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Create node launch description
    node = Node(
        package='zed_driver',
        executable='zed_driver_app',
        name=nm,
        namespace=ns,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[cf])

    ld.add_action(node)

    return ld
