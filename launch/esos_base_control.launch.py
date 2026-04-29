#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""
Launch file for esos_base_control node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # General parameters
        DeclareLaunchArgument('send_hz', default_value='20.0'),
        DeclareLaunchArgument('odom_hz', default_value='50.0'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='0.4'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),

        # Robot parameters
        DeclareLaunchArgument('wheel_diameter', default_value='0.067'),
        DeclareLaunchArgument('wheel_base', default_value='0.28'),
        DeclareLaunchArgument('motor1_factor', default_value='1.0'),
        DeclareLaunchArgument('motor2_factor', default_value='1.0'),
        DeclareLaunchArgument('reduction_ratio', default_value='56.0'),
        DeclareLaunchArgument('ff_factor', default_value='0.3'),
        DeclareLaunchArgument('pid_kp', default_value='0.05'),
        DeclareLaunchArgument('pid_ki', default_value='0.2'),
        DeclareLaunchArgument('pid_kd', default_value='0.01'),
        DeclareLaunchArgument('cfg_send_on_startup', default_value='true'),
        DeclareLaunchArgument('feedback_enable', default_value='false'),

        # RPMSG parameters
        DeclareLaunchArgument('rpmsg_ctrl_dev', default_value='/dev/rpmsg_ctrl0'),
        DeclareLaunchArgument('rpmsg_data_dev', default_value='/dev/rpmsg0'),
        DeclareLaunchArgument('rpmsg_service_name', default_value='rpmsg:motor_ctrl'),
        DeclareLaunchArgument('rpmsg_local_addr', default_value='1003'),
        DeclareLaunchArgument('rpmsg_remote_addr', default_value='1002'),

        Node(
            package='base',
            executable='esos_base_control_node',
            name='esos_base_control_node',
            output='screen',
            parameters=[{
                'send_hz': LaunchConfiguration('send_hz'),
                'odom_hz': LaunchConfiguration('odom_hz'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'wheel_diameter': LaunchConfiguration('wheel_diameter'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'motor1_factor': LaunchConfiguration('motor1_factor'),
                'motor2_factor': LaunchConfiguration('motor2_factor'),
                'reduction_ratio': LaunchConfiguration('reduction_ratio'),
                'ff_factor': LaunchConfiguration('ff_factor'),
                'pid_kp': LaunchConfiguration('pid_kp'),
                'pid_ki': LaunchConfiguration('pid_ki'),
                'pid_kd': LaunchConfiguration('pid_kd'),
                'cfg_send_on_startup': LaunchConfiguration('cfg_send_on_startup'),
                'feedback_enable': LaunchConfiguration('feedback_enable'),
                'rpmsg_ctrl_dev': LaunchConfiguration('rpmsg_ctrl_dev'),
                'rpmsg_data_dev': LaunchConfiguration('rpmsg_data_dev'),
                'rpmsg_service_name': LaunchConfiguration('rpmsg_service_name'),
                'rpmsg_local_addr': LaunchConfiguration('rpmsg_local_addr'),
                'rpmsg_remote_addr': LaunchConfiguration('rpmsg_remote_addr'),
            }],
        ),
    ])
