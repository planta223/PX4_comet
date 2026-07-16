"""Launch the DataFlash EKF replay node and RViz2."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description() -> LaunchDescription:
    rviz_config = os.path.join(
        get_package_share_directory("comet_ekf_replay"),
        "config",
        "bin_ekf_replay.rviz",
    )
    arguments = [
        DeclareLaunchArgument("bin_path", description="Absolute path to the ArduPilot DataFlash BIN file"),
        DeclareLaunchArgument("core", default_value="0"),
        DeclareLaunchArgument("playback_speed", default_value="1.0"),
        DeclareLaunchArgument("publish_rate", default_value="30.0"),
        DeclareLaunchArgument("trail_seconds", default_value="30.0"),
        DeclareLaunchArgument("use_quaternion_conjugate", default_value="false"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("start_paused", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]
    replay_node = Node(
        package="comet_ekf_replay",
        executable="bin_ekf_replay_node",
        name="bin_ekf_replay",
        output="screen",
        parameters=[{
            "bin_path": LaunchConfiguration("bin_path"),
            "core": ParameterValue(LaunchConfiguration("core"), value_type=int),
            "playback_speed": ParameterValue(LaunchConfiguration("playback_speed"), value_type=float),
            "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
            "trail_seconds": ParameterValue(LaunchConfiguration("trail_seconds"), value_type=float),
            "use_quaternion_conjugate": ParameterValue(
                LaunchConfiguration("use_quaternion_conjugate"), value_type=bool
            ),
            "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
            "start_paused": ParameterValue(LaunchConfiguration("start_paused"), value_type=bool),
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return LaunchDescription(arguments + [replay_node, rviz])
