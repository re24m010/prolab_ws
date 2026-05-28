from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Kalman Filter Node
        Node(
            package='probabilistic_localization',
            executable='kalmanFilter_node',
            name='kalmanFilter_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 2. Extended Kalman Filter Node
        Node(
            package='probabilistic_localization',
            executable='extendedKalmanFilter_node',
            name='extendedKalmanFilter_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 3. Partikelfilter Node
        Node(
            package='probabilistic_localization',
            executable='particleFilter_node',
            name='particleFilter_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 4. Automatischer Trajektorien-Generator (Steuerung)
        Node(
            package='probabilistic_localization',
            executable='trajectory_node',
            name='trajectory_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
