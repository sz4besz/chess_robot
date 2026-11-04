from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chess_robot',
            executable='robot_controller',
            name='chess_robot',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
