from setuptools import setup

package_name = "ros_fastapi_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="ROS 2 to FastAPI bridge for REST API access to topics",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = ros_fastapi_bridge.bridge_node:main",
        ],
    },
)
