"""
Sensor Fusion Node

Fuses data from multiple sensors using a simple averaging algorithm.
Demonstrates ROS 2 multi-subscription pattern with synchronized processing.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SensorFusion(Node):
    """
    Sensor fusion node that combines data from multiple sensors.

    Subscribes to:
        - sensor_a (Float32): First sensor reading
        - sensor_b (Float32): Second sensor reading

    Publishes:
        - fused (Float32): Fused sensor data
    """

    def __init__(self):
        super().__init__("sensor_fusion")

        # Declare parameters
        self.declare_parameter("fusion_method", "mean")
        self.declare_parameter("publish_rate", 10.0)

        # Get parameters
        self.fusion_method = (
            self.get_parameter("fusion_method").get_parameter_value().string_value
        )

        # Publisher for fused data
        self.pub_ = self.create_publisher(Float32, "fused", 10)

        # Subscribers for sensor inputs
        self.sub_a = self.create_subscription(Float32, "sensor_a", self.cb_a, 10)
        self.sub_b = self.create_subscription(Float32, "sensor_b", self.cb_b, 10)

        # Storage for latest readings
        self.a = None
        self.b = None

        self.get_logger().info(
            f"Sensor fusion node started (method: {self.fusion_method})"
        )

    def cb_a(self, msg: Float32):
        """Callback for sensor A."""
        self.a = msg.data
        self.try_publish()

    def cb_b(self, msg: Float32):
        """Callback for sensor B."""
        self.b = msg.data
        self.try_publish()

    def try_publish(self):
        """Attempt to publish fused data if both sensors have readings."""
        if self.a is None or self.b is None:
            return

        # Compute fused value based on method
        if self.fusion_method == "mean":
            fused = float(np.mean([self.a, self.b]))
        elif self.fusion_method == "weighted":
            fused = float(0.7 * self.a + 0.3 * self.b)
        elif self.fusion_method == "max":
            fused = float(max(self.a, self.b))
        else:
            fused = float(np.mean([self.a, self.b]))

        # Publish fused data
        out = Float32()
        out.data = fused
        self.pub_.publish(out)

        self.get_logger().debug(f"Fused: {fused:.4f} (a={self.a:.4f}, b={self.b:.4f})")


def main(args=None):
    """Entry point for sensor fusion node."""
    rclpy.init(args=args)
    node = SensorFusion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
