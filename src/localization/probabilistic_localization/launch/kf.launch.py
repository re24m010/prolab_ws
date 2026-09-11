from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    kf_node = Node(
        package='probabilistic_localization',
        executable='kalmanFilter_node',
        name='kalmanFilter_node',
        output='screen'
    )

    ekf_node = Node(
        package='probabilistic_localization',
        executable='extendedKalmanFilter_node',
        name='extendedKalmanFilter_node',
        output='screen'
    )

    pf_node = Node(
        package='probabilistic_localization',
        executable='particleFilter_node',
        name='particleFilter_node',
        output='screen'
    )

    noise_node = Node(
        package='probabilistic_localization',
        executable='noise_simulator_node',
        name='noise_simulator_node',
        output='screen'
    )

    trajectory_node = Node(
        package='probabilistic_localization',
        executable='infinity_node',
        name='infinity_node',
        output='screen'
    )

    return LaunchDescription([
        noise_node,
        kf_node,
        ekf_node,
        pf_node,
        trajectory_node
    ])