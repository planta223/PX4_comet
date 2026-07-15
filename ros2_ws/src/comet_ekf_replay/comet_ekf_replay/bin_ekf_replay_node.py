"""ROS 2 node publishing ArduPilot DataFlash EKF3 replay state."""

from __future__ import annotations

from pathlib import Path
import math
import time
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String, UInt32
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from bin_ekf_replay import PymavlinkUnavailable, build_states, read_dataflash_bin
except ImportError as exc:
    raise ImportError(
        "bin_ekf_replay core package is unavailable. Install it into the ROS 2 system "
        "Python with `python3 -m pip install --user -e ~/Workspace/PX4_comet/bin_ekf_replay`."
    ) from exc

from .marker_builder import MarkerBuilder
from .replay_controller import ReplayController
from .ros_state_converter import state_row_to_ros


FRAME_ID = "map"
CHILD_FRAME_ID = "rocket"


class BinEkfReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("bin_ekf_replay")
        self.declare_parameter("bin_path", "")
        self.declare_parameter("core", 0)
        self.declare_parameter("playback_speed", 1.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("trail_seconds", 30.0)
        self.declare_parameter("match_tolerance", 0.05)
        self.declare_parameter("use_quaternion_conjugate", False)
        self.declare_parameter("model_axis_offset_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("loop", False)
        self.declare_parameter("start_paused", False)

        self.bin_path = self._validated_bin_path(str(self.get_parameter("bin_path").value))
        self.core = int(self.get_parameter("core").value)
        self.playback_speed = self._positive_parameter("playback_speed")
        self.publish_rate = self._positive_parameter("publish_rate")
        self.trail_seconds = self._nonnegative_parameter("trail_seconds")
        self.match_tolerance = self._nonnegative_parameter("match_tolerance")
        self.use_quaternion_conjugate = bool(self.get_parameter("use_quaternion_conjugate").value)
        self.model_axis_offset_rpy_deg = list(self.get_parameter("model_axis_offset_rpy_deg").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.start_paused = bool(self.get_parameter("start_paused").value)

        try:
            parsed = read_dataflash_bin(self.bin_path, core=self.core)
        except PymavlinkUnavailable as exc:
            raise RuntimeError(str(exc)) from exc
        if parsed.xkf1.empty:
            available = ", ".join(parsed.available_types) or "none"
            raise RuntimeError(f"No XKF1 for core {self.core}. Available message types: {available}")
        built = build_states(parsed.xkf1, parsed.xkq, parsed.xkf4, self.match_tolerance)
        self.states = built.states
        self.times = self.states["time_s"].to_numpy(float)
        self.positions = self.states[["x_east_m", "y_north_m", "z_up_m"]].to_numpy(float)
        if not np.all(np.isfinite(self.positions)):
            raise RuntimeError("Aligned states contain non-finite ENU positions")
        self.controller = ReplayController(
            self.times,
            playback_speed=self.playback_speed,
            loop=self.loop,
            start_paused=self.start_paused,
        )

        axis_length = self._axis_display_length()
        speed_values = np.linalg.norm(
            self.states[["vx_east_mps", "vy_north_mps", "vz_up_mps"]].to_numpy(float), axis=1
        )
        finite_speeds = speed_values[np.isfinite(speed_values)]
        p90_speed = float(np.percentile(finite_speeds, 90)) if len(finite_speeds) else 1.0
        velocity_scale = max(0.25, axis_length / max(p90_speed, 0.1))
        self.marker_builder = MarkerBuilder(FRAME_ID, axis_length, velocity_scale)

        normal_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.odom_pub = self.create_publisher(Odometry, "/bin_ekf/odometry", normal_qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/bin_ekf/pose", normal_qos)
        self.path_pub = self.create_publisher(PathMessage, "/bin_ekf/path", normal_qos)
        self.full_path_pub = self.create_publisher(PathMessage, "/bin_ekf/full_path", path_qos)
        self.trail_path_pub = self.create_publisher(PathMessage, "/bin_ekf/trail_path", normal_qos)
        self.velocity_pub = self.create_publisher(Marker, "/bin_ekf/velocity_marker", normal_qos)
        self.axes_pub = self.create_publisher(MarkerArray, "/bin_ekf/body_axes", normal_qos)
        self.current_marker_pub = self.create_publisher(Marker, "/bin_ekf/current_marker", normal_qos)
        self.status_marker_pub = self.create_publisher(Marker, "/bin_ekf/status_marker", normal_qos)
        self.status_text_pub = self.create_publisher(String, "/bin_ekf/status_text", normal_qos)
        self.index_pub = self.create_publisher(UInt32, "/bin_ekf/current_index", normal_qos)
        self.replay_time_pub = self.create_publisher(Float64, "/bin_ekf/replay_time", normal_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pause_service = self.create_service(SetBool, "/bin_ekf/pause", self._on_pause)
        self.reset_service = self.create_service(Trigger, "/bin_ekf/reset", self._on_reset)
        self.add_on_set_parameters_callback(self._on_parameters)

        self._path_poses = self._build_path_poses()
        self._full_path = PathMessage()
        self._full_path.header.frame_id = FRAME_ID
        self._full_path.poses = self._path_poses
        self._last_index = -1
        self._last_full_path_wall = -math.inf
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)
        self._publish_full_path(force=True)

        self.get_logger().info(
            f"Loaded {len(self.states)} core-{self.core} states from {self.bin_path}; "
            f"time={self.times[0]:.3f}..{self.times[-1]:.3f}s, "
            f"XKQ valid={built.quaternion_valid_ratio:.2%}, rate={self.publish_rate:g}Hz"
        )
        for warning in built.warnings:
            self.get_logger().warning(warning)

    def _validated_bin_path(self, raw_path: str) -> Path:
        if not raw_path.strip():
            raise RuntimeError("ROS parameter 'bin_path' is required and must point to an ArduPilot .BIN file")
        path = Path(raw_path).expanduser()
        if not path.is_absolute():
            self.get_logger().warning(f"Relative bin_path resolved against current directory: {path}")
            path = path.resolve()
        if not path.is_file():
            raise RuntimeError(f"BIN file does not exist: {path}")
        return path

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"Parameter '{name}' must be finite and greater than zero")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(f"Parameter '{name}' must be finite and nonnegative")
        return value

    def _axis_display_length(self) -> float:
        span = np.ptp(self.positions, axis=0)
        return max(0.5, float(np.max(span)) * 0.06)

    def _build_path_poses(self) -> List[PoseStamped]:
        poses: List[PoseStamped] = []
        zero_stamp = self.get_clock().now().to_msg()
        for _, row in self.states.iterrows():
            converted = state_row_to_ros(
                row,
                self.use_quaternion_conjugate,
                self.model_axis_offset_rpy_deg,
            )
            pose = PoseStamped()
            pose.header.frame_id = FRAME_ID
            pose.header.stamp = zero_stamp
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, converted.position_enu)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(
                float, converted.quaternion_xyzw
            )
            poses.append(pose)
        return poses

    def _publish_full_path(self, force: bool = False) -> None:
        now_wall = time.monotonic()
        if force or now_wall - self._last_full_path_wall >= 5.0:
            self._full_path.header.stamp = self.get_clock().now().to_msg()
            self.full_path_pub.publish(self._full_path)
            self._last_full_path_wall = now_wall

    def _trail_message(self, index: int, stamp) -> PathMessage:
        start_time = self.times[index] - self.trail_seconds
        start_index = int(np.searchsorted(self.times, start_time, side="left"))
        message = PathMessage()
        message.header.frame_id = FRAME_ID
        message.header.stamp = stamp
        # Pose objects are prebuilt once; only the bounded reference list is sliced.
        message.poses = self._path_poses[start_index:index + 1]
        return message

    @staticmethod
    def _pose_message(stamp, converted) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = FRAME_ID
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, converted.position_enu)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(
            float, converted.quaternion_xyzw
        )
        return pose

    def _publish_tf(self, stamp, pose: PoseStamped) -> None:
        transform = TransformStamped()
        transform.header.frame_id = FRAME_ID
        transform.header.stamp = stamp
        transform.child_frame_id = CHILD_FRAME_ID
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _on_timer(self) -> None:
        snapshot = self.controller.snapshot()
        row = self.states.iloc[snapshot.index]
        converted = state_row_to_ros(
            row,
            self.use_quaternion_conjugate,
            self.model_axis_offset_rpy_deg,
        )
        stamp = self.get_clock().now().to_msg()
        pose = self._pose_message(stamp, converted)
        self.pose_pub.publish(pose)

        odometry = Odometry()
        odometry.header = pose.header
        odometry.child_frame_id = CHILD_FRAME_ID
        odometry.pose.pose = pose.pose
        odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z = map(
            float, converted.velocity_enu
        )
        self.odom_pub.publish(odometry)
        self._publish_tf(stamp, pose)

        if snapshot.index != self._last_index:
            trail = self._trail_message(snapshot.index, stamp)
            self.trail_path_pub.publish(trail)
            self.path_pub.publish(trail)
            self._last_index = snapshot.index

        status = self.marker_builder.status_string(row, snapshot.replay_time, self.controller.speed, snapshot.paused)
        self.velocity_pub.publish(self.marker_builder.velocity(stamp, converted))
        self.axes_pub.publish(self.marker_builder.body_axes(stamp, converted))
        self.current_marker_pub.publish(self.marker_builder.current_position(stamp, converted))
        self.status_marker_pub.publish(self.marker_builder.status_marker(stamp, converted, status))
        self.status_text_pub.publish(String(data=status))
        self.index_pub.publish(UInt32(data=snapshot.index))
        self.replay_time_pub.publish(Float64(data=snapshot.replay_time))
        self._publish_full_path()

    def _on_pause(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.controller.set_paused(request.data)
        response.success = True
        response.message = "paused" if request.data else "resumed"
        return response

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.controller.reset(resume=True)
        self._last_index = -1
        response.success = True
        response.message = "replay reset to the first sample and resumed"
        return response

    def _on_parameters(self, parameters) -> SetParametersResult:
        values = {parameter.name: parameter.value for parameter in parameters}
        try:
            for name in ("playback_speed", "publish_rate"):
                if name in values and (not math.isfinite(float(values[name])) or float(values[name]) <= 0.0):
                    raise ValueError(f"{name} must be finite and greater than zero")
            for name in ("trail_seconds", "match_tolerance"):
                if name in values and (not math.isfinite(float(values[name])) or float(values[name]) < 0.0):
                    raise ValueError(f"{name} must be finite and nonnegative")
            if "model_axis_offset_rpy_deg" in values:
                offset = np.asarray(values["model_axis_offset_rpy_deg"], dtype=float)
                if offset.shape != (3,) or not np.all(np.isfinite(offset)):
                    raise ValueError("model_axis_offset_rpy_deg must have three finite values")
            for restart_only in ("bin_path", "core", "match_tolerance", "start_paused"):
                if restart_only in values:
                    raise ValueError(f"{restart_only} can only be changed by restarting the node")
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if "playback_speed" in values:
            self.controller.set_speed(float(values["playback_speed"]))
            self.playback_speed = float(values["playback_speed"])
        if "loop" in values:
            self.controller.set_loop(bool(values["loop"]))
            self.loop = bool(values["loop"])
        if "trail_seconds" in values:
            self.trail_seconds = float(values["trail_seconds"])
        if "use_quaternion_conjugate" in values:
            self.use_quaternion_conjugate = bool(values["use_quaternion_conjugate"])
        if "model_axis_offset_rpy_deg" in values:
            self.model_axis_offset_rpy_deg = list(values["model_axis_offset_rpy_deg"])
        if "publish_rate" in values:
            self.publish_rate = float(values["publish_rate"])
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)
        return SetParametersResult(successful=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BinEkfReplayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(f"Replay node failed: {type(exc).__name__}: {exc}")
        else:
            print(f"Replay node failed: {type(exc).__name__}: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
