import os
from glob import glob

from setuptools import setup

package_name = "sample_py_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Sample Python ROS 2 package with sensor fusion",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_fusion = sample_py_pkg.sensor_fusion_node:main",
            "gpu_inference = sample_py_pkg.gpu_inference_stub:main",
            "sensor_publisher = sample_py_pkg.sensor_publisher:main",
        ],
    },
)
