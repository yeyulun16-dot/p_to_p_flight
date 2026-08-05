import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int16, String, UInt8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import body_offset_to_map, normalize_angle_deg, target_error


@dataclass
class PoseCm:
    x: float
    y: float
    yaw_deg: float


@dataclass
class Target:
    x: float
    y: float
    z: float
    yaw_deg: float


class PointToPointMission(Node):
    """A -> takeoff -> B -> hold/descend mission coordinator.

    This node publishes map-frame targets.  Position PID converts them to a
    map-frame velocity, and uart_to_stm32 rotates that velocity into the body
    frame before sending protocol frame 0x31.
    """

    ACTIVE_STATES = {
        "TAKEOFF",
        "GOTO_B",
        "HOLD_B",
        "DESCEND_B",
        "LANDED_HOLD",
        "ABORT_HOLD",
        "FAULT_HOLD",
        "OBSTACLE_HOLD",
    }

    def __init__(self) -> None:
        super().__init__("p_to_p_mission")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.body_frame = self.declare_parameter("body_frame", "laser_link").value
        self.height_topic = self.declare_parameter(
            "height_topic", "/height"
        ).value
        self.target_mode = self.declare_parameter("target_mode", "relative_body").value
        self.b_offset_x_cm = float(self.declare_parameter("b_offset_x_cm", 250.0).value)
        self.b_offset_y_cm = float(self.declare_parameter("b_offset_y_cm", -250.0).value)
        self.b_x_cm = float(self.declare_parameter("b_x_cm", 250.0).value)
        self.b_y_cm = float(self.declare_parameter("b_y_cm", -250.0).value)
        self.cruise_height_cm = float(
            self.declare_parameter("cruise_height_cm", 150.0).value
        )
        self.landing_height_cm = float(
            self.declare_parameter("landing_height_cm", 13.0).value
        )
        self.target_yaw_deg = float(self.declare_parameter("target_yaw_deg", 0.0).value)
        self.keep_takeoff_yaw = bool(self.declare_parameter("keep_takeoff_yaw", True).value)
        self.terminal_behavior = self.declare_parameter("terminal_behavior", "hold").value
        self.pre_descent_hold_sec = float(
            self.declare_parameter("pre_descent_hold_sec", 3.0).value
        )
        self.position_tolerance_cm = float(
            self.declare_parameter("position_tolerance_cm", 12.0).value
        )
        self.height_tolerance_cm = float(
            self.declare_parameter("height_tolerance_cm", 12.0).value
        )
        self.yaw_tolerance_deg = float(
            self.declare_parameter("yaw_tolerance_deg", 8.0).value
        )
        self.reached_stable_sec = float(
            self.declare_parameter("reached_stable_sec", 1.0).value
        )
        self.sensor_timeout_sec = float(
            self.declare_parameter("sensor_timeout_sec", 0.5).value
        )
        self.stop_output_max_height_cm = float(
            self.declare_parameter("stop_output_max_height_cm", 25.0).value
        )
        self.abort_descent_on_obstacle = bool(
            self.declare_parameter("abort_descent_on_obstacle", True).value
        )
        self.route_choice = int(self.declare_parameter("route_choice", 3).value)
        self.update_rate_hz = float(self.declare_parameter("update_rate_hz", 10.0).value)

        self._validate_parameters()

        durable_qos = QoSProfile(depth=1)
        durable_qos.reliability = ReliabilityPolicy.RELIABLE
        durable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        self.target_pub = self.create_publisher(
            Float32MultiArray, "/target_position", durable_qos
        )
        self.state_pub = self.create_publisher(String, "/p_to_p/state", durable_qos)
        self.route_pub = self.create_publisher(UInt8, "/route_choice", route_qos)
        self.mission_complete_pub = self.create_publisher(Empty, "/mission_complete", 10)

        self.height_sub = self.create_subscription(
            Int16, self.height_topic, self._height_callback, 10
        )
        self.obstacle_sub = self.create_subscription(
            Bool, "/laser_array/obstacle_below", self._obstacle_callback, 10
        )
        self.start_srv = self.create_service(Trigger, "/p_to_p/start", self._start_callback)
        self.abort_srv = self.create_service(
            Trigger, "/p_to_p/abort_hold", self._abort_hold_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "/p_to_p/stop_output", self._stop_output_callback
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = "IDLE"
        self.current_pose: Optional[PoseCm] = None
        self.current_height_cm: Optional[float] = None
        self.last_pose_monotonic = 0.0
        self.last_height_monotonic = 0.0
        self.last_tf_warning_monotonic = 0.0
        self.obstacle_below = False
        self.a_pose: Optional[PoseCm] = None
        self.b_target: Optional[Target] = None
        self.active_target: Optional[Target] = None
        self.reached_since: Optional[float] = None
        self.state_entered_monotonic = time.monotonic()
        self.route_kick_remaining = 0

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self._timer_callback)
        self._publish_state("等待定位、测高与人工启动")
        self.get_logger().info(
            "Point-to-point mission ready. Call /p_to_p/start after preflight checks."
        )

    def _validate_parameters(self) -> None:
        if self.target_mode not in {"relative_body", "relative_map", "absolute"}:
            raise ValueError(
                "target_mode must be relative_body, relative_map, or absolute"
            )
        if self.terminal_behavior not in {"hold", "land"}:
            raise ValueError("terminal_behavior must be hold or land")
        if self.cruise_height_cm <= self.landing_height_cm:
            raise ValueError("cruise_height_cm must be greater than landing_height_cm")
        if self.update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        if self.sensor_timeout_sec <= 0.0:
            raise ValueError("sensor_timeout_sec must be positive")
        if self.stop_output_max_height_cm < 0.0:
            raise ValueError("stop_output_max_height_cm must not be negative")

    def _height_callback(self, msg: Int16) -> None:
        self.current_height_cm = float(msg.data)
        self.last_height_monotonic = time.monotonic()

    def _obstacle_callback(self, msg: Bool) -> None:
        self.obstacle_below = bool(msg.data)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _update_pose(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time()
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warning_monotonic >= 2.0:
                self.get_logger().warning(
                    f"TF {self.map_frame}->{self.body_frame} unavailable: {exc}"
                )
                self.last_tf_warning_monotonic = now
            return False

        transform_age_sec = (
            self.get_clock().now().nanoseconds
            - Time.from_msg(tf.header.stamp).nanoseconds
        ) / 1e9
        if transform_age_sec > self.sensor_timeout_sec or transform_age_sec < -0.1:
            return False

        q = tf.transform.rotation
        self.current_pose = PoseCm(
            x=tf.transform.translation.x * 100.0,
            y=tf.transform.translation.y * 100.0,
            yaw_deg=self._yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.last_pose_monotonic = time.monotonic()
        return True

    def _sensors_fresh(self) -> bool:
        now = time.monotonic()
        return (
            self.current_pose is not None
            and self.current_height_cm is not None
            and now - self.last_pose_monotonic <= self.sensor_timeout_sec
            and now - self.last_height_monotonic <= self.sensor_timeout_sec
        )

    def _start_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self._update_pose()
        if self.state in self.ACTIVE_STATES:
            response.success = False
            response.message = f"任务已在运行，当前状态={self.state}"
            return response
        if not self._sensors_fresh():
            response.success = False
            response.message = "定位或高度数据尚未就绪/已超时"
            return response

        assert self.current_pose is not None
        self.a_pose = PoseCm(
            self.current_pose.x, self.current_pose.y, self.current_pose.yaw_deg
        )
        yaw_target = (
            self.a_pose.yaw_deg if self.keep_takeoff_yaw else self.target_yaw_deg
        )

        if self.target_mode == "relative_body":
            b_x, b_y = body_offset_to_map(
                self.a_pose.x,
                self.a_pose.y,
                self.a_pose.yaw_deg,
                self.b_offset_x_cm,
                self.b_offset_y_cm,
            )
        elif self.target_mode == "relative_map":
            b_x = self.a_pose.x + self.b_offset_x_cm
            b_y = self.a_pose.y + self.b_offset_y_cm
        else:
            b_x, b_y = self.b_x_cm, self.b_y_cm

        self.b_target = Target(b_x, b_y, self.cruise_height_cm, yaw_target)
        self.active_target = Target(
            self.a_pose.x, self.a_pose.y, self.cruise_height_cm, yaw_target
        )
        self.route_kick_remaining = 1
        self._enter_state("TAKEOFF", "任务启动，保持 A 点水平位置并爬升")

        response.success = True
        response.message = (
            f"启动成功：A=({self.a_pose.x:.1f},{self.a_pose.y:.1f})cm, "
            f"B=({b_x:.1f},{b_y:.1f})cm, 高度={self.cruise_height_cm:.1f}cm"
        )
        return response

    def _abort_hold_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self._update_pose()
        if not self._sensors_fresh():
            response.success = False
            response.message = "无法取得新鲜位姿/高度，飞控端看门狗将负责速度归零"
            return response

        assert self.current_pose is not None and self.current_height_cm is not None
        self.active_target = Target(
            self.current_pose.x,
            self.current_pose.y,
            max(self.current_height_cm, self.landing_height_cm),
            self.current_pose.yaw_deg,
        )
        self.route_kick_remaining = 1
        self._enter_state("ABORT_HOLD", "任务中止：保持当前位置，等待人工处理")
        response.success = True
        response.message = "已切换为当前位置悬停；不会自动锁桨"
        return response

    def _stop_output_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self._update_pose()
        if not self._sensors_fresh():
            response.success = False
            response.message = "定位或高度数据不新鲜，拒绝停止控制输出"
            return response
        if not 0.0 <= self.current_height_cm <= self.stop_output_max_height_cm:
            response.success = False
            response.message = (
                f"当前高度不在 0~{self.stop_output_max_height_cm:.1f}cm 安全范围，"
                "拒绝停止控制输出"
            )
            return response
        self.mission_complete_pub.publish(Empty())
        self.active_target = None
        self._enter_state("FINISHED", "已在近地高度停止速度输出，请人工锁桨")
        response.success = True
        response.message = "已发送零速度和任务结束帧，请确认落地后人工锁桨"
        return response

    def _enter_state(self, state: str, detail: str) -> None:
        self.state = state
        self.state_entered_monotonic = time.monotonic()
        self.reached_since = None
        self._publish_state(detail)
        self.get_logger().info(f"STATE -> {state}: {detail}")

    def _publish_state(self, detail: str = "") -> None:
        msg = String()
        msg.data = f"{self.state}: {detail}" if detail else self.state
        self.state_pub.publish(msg)

    def _publish_target(self) -> None:
        if self.active_target is None:
            return
        msg = Float32MultiArray()
        msg.data = [
            float(self.active_target.x),
            float(self.active_target.y),
            float(self.active_target.z),
            float(self.active_target.yaw_deg),
        ]
        self.target_pub.publish(msg)

    def _target_reached_stably(self) -> bool:
        if (
            self.active_target is None
            or self.current_pose is None
            or self.current_height_cm is None
        ):
            self.reached_since = None
            return False

        xy_error, z_error, yaw_error = target_error(
            self.current_pose.x,
            self.current_pose.y,
            self.current_height_cm,
            self.current_pose.yaw_deg,
            self.active_target.x,
            self.active_target.y,
            self.active_target.z,
            self.active_target.yaw_deg,
        )
        inside = (
            xy_error <= self.position_tolerance_cm
            and z_error <= self.height_tolerance_cm
            and yaw_error <= self.yaw_tolerance_deg
        )
        if not inside:
            self.reached_since = None
            return False
        if self.reached_since is None:
            self.reached_since = time.monotonic()
            return False
        return time.monotonic() - self.reached_since >= self.reached_stable_sec

    def _fault_hold(self, reason: str) -> None:
        if self.current_pose is not None and self.current_height_cm is not None:
            self.active_target = Target(
                self.current_pose.x,
                self.current_pose.y,
                max(self.current_height_cm, self.landing_height_cm),
                self.current_pose.yaw_deg,
            )
        self._enter_state("FAULT_HOLD", reason)

    def _timer_callback(self) -> None:
        self._update_pose()

        if self.route_kick_remaining > 0:
            msg = UInt8()
            msg.data = self.route_choice
            self.route_pub.publish(msg)
            self.route_kick_remaining -= 1

        if self.state not in self.ACTIVE_STATES:
            return

        if not self._sensors_fresh():
            if self.state != "FAULT_HOLD":
                self._fault_hold("定位或高度数据超时；保持最后目标并等待飞控看门狗")
            return

        self._publish_target()

        if self.state == "TAKEOFF" and self._target_reached_stably():
            assert self.b_target is not None
            self.active_target = self.b_target
            self._enter_state("GOTO_B", "达到巡航高度，飞向 B 点")
        elif self.state == "GOTO_B" and self._target_reached_stably():
            self._enter_state("HOLD_B", "到达 B 点，持续闭环悬停")
        elif self.state == "HOLD_B":
            if (
                self.terminal_behavior == "land"
                and time.monotonic() - self.state_entered_monotonic
                >= self.pre_descent_hold_sec
            ):
                assert self.b_target is not None
                self.active_target = Target(
                    self.b_target.x,
                    self.b_target.y,
                    self.landing_height_cm,
                    self.b_target.yaw_deg,
                )
                self._enter_state("DESCEND_B", "B 点垂直下降")
        elif self.state == "DESCEND_B":
            if self.abort_descent_on_obstacle and self.obstacle_below:
                assert self.b_target is not None
                self.active_target = self.b_target
                self._enter_state("OBSTACLE_HOLD", "下方检测到障碍，返回巡航高度悬停")
            elif self._target_reached_stably():
                self._enter_state("LANDED_HOLD", "达到近地目标高度，等待人工停止输出和锁桨")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointToPointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
