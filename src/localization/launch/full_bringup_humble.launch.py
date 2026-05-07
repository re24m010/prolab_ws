#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

 
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=TextSubstitution(text='true'),
        description='Use simulation time (Gazebo clock)'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=TextSubstitution(text=os.path.expanduser('~/map.yaml')),
        description='Absolute path to map YAML file'
    )


    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    nav2_pkg = get_package_share_directory('turtlebot3_navigation2')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation2.launch.py')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        gazebo,
        nav2
    ])
