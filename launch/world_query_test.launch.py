#!/usr/bin/env python3

"""Launch file for testing the WorldQuerySystem plugin."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for world query test."""
    
    # Declare arguments
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='world_query_test.sdf',
        description='Name of the world file to load'
    )
    
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='query_points_example.csv',
        description='Name of the CSV file with query points'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('ohms_sim')
    
    # Path to world file
    world_path = PathJoinSubstitution([
        pkg_share,
        'worlds',
        LaunchConfiguration('world')
    ])
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_path, ' -v 4 -r'],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    return LaunchDescription([
        world_file_arg,
        csv_file_arg,
        gazebo_launch,
    ])
