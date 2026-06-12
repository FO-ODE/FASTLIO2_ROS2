from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrain_mapping',
            executable='tf_mapping_relay',
            name='tf_mapping_relay',
            output='screen',
        ),
    ])