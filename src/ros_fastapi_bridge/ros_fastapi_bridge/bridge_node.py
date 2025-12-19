"""
ROS 2 FastAPI Bridge Node

Provides HTTP REST API access to ROS 2 topics.

Endpoints:
    GET  /health              - Health check
    GET  /topics              - List available topics
    GET  /last/{topic}        - Get last message from topic
    POST /publish/{topic}     - Publish message to topic
    GET  /services            - List available services
    WS   /ws/{topic}          - WebSocket stream for topic

Usage:
    ros2 run ros_fastapi_bridge bridge

    Then access: http://localhost:8000/docs for API documentation
"""

import threading
from contextlib import asynccontextmanager
from typing import Any, Dict, Optional

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from geometry_msgs.msg import Pose, Twist
from pydantic import BaseModel
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String


# Message storage
class MessageStore:
    """Thread-safe storage for latest messages."""

    def __init__(self):
        self._data: Dict[str, Any] = {}
        self._lock = threading.Lock()

    def set(self, topic: str, data: Any):
        with self._lock:
            self._data[topic] = data

    def get(self, topic: str) -> Optional[Any]:
        with self._lock:
            return self._data.get(topic)

    def all(self) -> Dict[str, Any]:
        with self._lock:
            return self._data.copy()


# Pydantic models for API
class PublishRequest(BaseModel):
    """Request body for publish endpoint."""

    data: str


class TwistRequest(BaseModel):
    """Request body for Twist messages."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


# Global instances
store = MessageStore()
ros_node: Optional["BridgeNode"] = None


class BridgeNode(Node):
    """ROS 2 node that bridges to FastAPI."""

    def __init__(self):
        super().__init__("ros_fastapi_bridge")

        # Publishers
        self.pub_string = self.create_publisher(String, "bridge/string_out", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers
        self.sub_string = self.create_subscription(
            String, "bridge/string_in", self._cb_string, 10
        )
        self.sub_float = self.create_subscription(
            Float32, "bridge/float_in", self._cb_float, 10
        )

        self.get_logger().info("ROS FastAPI Bridge node started")

    def _cb_string(self, msg: String):
        store.set("bridge/string_in", {"data": msg.data})
        self.get_logger().debug(f"Received string: {msg.data}")

    def _cb_float(self, msg: Float32):
        store.set("bridge/float_in", {"data": msg.data})
        self.get_logger().debug(f"Received float: {msg.data}")

    def publish_string(self, data: str):
        msg = String()
        msg.data = data
        self.pub_string.publish(msg)
        self.get_logger().info(f"Published string: {data}")

    def publish_cmd_vel(self, twist: TwistRequest):
        msg = Twist()
        msg.linear.x = twist.linear_x
        msg.linear.y = twist.linear_y
        msg.linear.z = twist.linear_z
        msg.angular.x = twist.angular_x
        msg.angular.y = twist.angular_y
        msg.angular.z = twist.angular_z
        self.pub_cmd_vel.publish(msg)
        self.get_logger().info(
            f"Published cmd_vel: linear=({twist.linear_x}, {twist.linear_y}, {twist.linear_z})"
        )


# FastAPI app setup
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage ROS node lifecycle with FastAPI."""
    global ros_node

    # Startup
    rclpy.init()
    ros_node = BridgeNode()

    # Run ROS in background thread
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    yield

    # Shutdown
    ros_node.destroy_node()
    rclpy.shutdown()


app = FastAPI(
    title="ROS 2 FastAPI Bridge",
    description="HTTP REST API for ROS 2 topic access",
    version="0.1.0",
    lifespan=lifespan,
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# API Endpoints
@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "ros_node": ros_node is not None}


@app.get("/topics")
async def list_topics():
    """List available topics with cached data."""
    return {
        "subscribed": ["bridge/string_in", "bridge/float_in"],
        "publishers": ["bridge/string_out", "cmd_vel"],
        "cached": list(store.all().keys()),
    }


@app.get("/last/{topic:path}")
async def get_last_message(topic: str):
    """Get the last received message from a topic."""
    data = store.get(topic)
    if data is None:
        raise HTTPException(status_code=404, detail=f"No data for topic: {topic}")
    return {"topic": topic, "data": data}


@app.post("/publish/string")
async def publish_string(request: PublishRequest):
    """Publish a string message."""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not initialized")
    ros_node.publish_string(request.data)
    return {"published": True, "data": request.data}


@app.post("/publish/cmd_vel")
async def publish_cmd_vel(request: TwistRequest):
    """Publish a Twist message to cmd_vel."""
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS node not initialized")
    ros_node.publish_cmd_vel(request)
    return {"published": True, "twist": request.dict()}


@app.get("/")
async def root():
    """Root endpoint with API info."""
    return {
        "name": "ROS 2 FastAPI Bridge",
        "version": "0.1.0",
        "docs": "/docs",
        "health": "/health",
    }


def main(args=None):
    """Entry point for the bridge node."""
    uvicorn.run(
        "ros_fastapi_bridge.bridge_node:app",
        host="0.0.0.0",
        port=8000,
        log_level="info",
    )


if __name__ == "__main__":
    main()
