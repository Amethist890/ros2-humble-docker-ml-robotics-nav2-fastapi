from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch sensor fusion pipeline with simulated sensors."""

    # Declare launch arguments
    fusion_method_arg = DeclareLaunchArgument(
        "fusion_method",
        default_value="mean",
        description="Fusion method: mean, weighted, or max",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate", default_value="10.0", description="Sensor publish rate in Hz"
    )

    return LaunchDescription(
        [
            fusion_method_arg,
            publish_rate_arg,
            # Sensor publisher node
            Node(
                package="sample_py_pkg",
                executable="sensor_publisher",
                name="sensor_publisher",
                output="screen",
                parameters=[
                    {
                        "base_value": 25.0,
                        "noise_std_a": 0.5,
                        "noise_std_b": 1.0,
                        "publish_rate": LaunchConfiguration("publish_rate"),
                    }
                ],
            ),
            # Sensor fusion node
            Node(
                package="sample_py_pkg",
                executable="sensor_fusion",
                name="sensor_fusion",
                output="screen",
                parameters=[
                    {
                        "fusion_method": LaunchConfiguration("fusion_method"),
                    }
                ],
            ),
        ]
    )
