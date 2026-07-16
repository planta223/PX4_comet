from glob import glob
from setuptools import find_packages, setup


package_name = "comet_ekf_replay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kyubeom",
    maintainer_email="kyubeom@example.com",
    description="ROS 2 and RViz2 playback for ArduPilot DataFlash EKF3 state",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bin_ekf_replay_node = comet_ekf_replay.bin_ekf_replay_node:main",
        ],
    },
)
