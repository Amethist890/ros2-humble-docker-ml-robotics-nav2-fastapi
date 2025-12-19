"""
Sensor Publisher Node

Publishes simulated sensor data for testing the sensor fusion pipeline.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SensorPublisher(Node):
    """
    Simulates two sensors publishing noisy readings.

    Publishes:
        - sensor_a (Float32): Simulated sensor A readings
        - sensor_b (Float32): Simulated sensor B readings
    """

    def __init__(self):
        super().__init__("sensor_publisher")

        # Declare parameters
        self.declare_parameter("base_value", 25.0)
        self.declare_parameter("noise_std_a", 0.5)
        self.declare_parameter("noise_std_b", 1.0)
        self.declare_parameter("publish_rate", 10.0)

        # Get parameters
        self.base_value = (
            self.get_parameter("base_value").get_parameter_value().double_value
        )
        self.noise_std_a = (
            self.get_parameter("noise_std_a").get_parameter_value().double_value
        )
        self.noise_std_b = (
            self.get_parameter("noise_std_b").get_parameter_value().double_value
        )
        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        # Publishers
        self.pub_a = self.create_publisher(Float32, "sensor_a", 10)
        self.pub_b = self.create_publisher(Float32, "sensor_b", 10)

        # Timer for periodic publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Sensor publisher started (rate: {publish_rate} Hz)")

    def timer_callback(self):
        """Publish simulated sensor readings."""
        # Generate noisy readings
        value_a = self.base_value + np.random.normal(0, self.noise_std_a)
        value_b = self.base_value + np.random.normal(0, self.noise_std_b)

        # Publish sensor A
        msg_a = Float32()
        msg_a.data = float(value_a)
        self.pub_a.publish(msg_a)

        # Publish sensor B
        msg_b = Float32()
        msg_b.data = float(value_b)
        self.pub_b.publish(msg_b)

        self.get_logger().debug(f"Published: A={value_a:.4f}, B={value_b:.4f}")


def main(args=None):
    """Entry point for sensor publisher node."""
    rclpy.init(args=args)
    node = SensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
