import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Gazebo World Launch
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    # 2. RViz2 mit deiner Konfiguration
    rviz_config_file = '/home/sameh/prolab_ws/src/localization/probabilistic_localization/rviz/probLab_config.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # RViz erst nach 4 Sekunden öffnen, damit Gazebo Zeit zum Initialisieren hat
    delayed_rviz = TimerAction(
        period=4.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        gazebo,
        delayed_rviz
    ])