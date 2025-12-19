from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch talker and listener nodes together."""
    return LaunchDescription([
        Node(
            package='sample_cpp_pkg',
            executable='talker',
            name='talker',
            output='screen',
        ),
        Node(
            package='sample_cpp_pkg',
            executable='listener',
            name='listener',
            output='screen',
        ),
    ])

