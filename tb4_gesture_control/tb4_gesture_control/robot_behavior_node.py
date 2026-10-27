"""Lớp kết nối ROS 2 cho bộ điều khiển trong behavior.py.

Đầu vào: /detected_gesture (String), /odom (Odometry), /scan (LaserScan).
Đầu ra: /cmd_vel (Twist), /robot_behavior/status (String).
Timer đồng hồ thực gọi bộ điều khiển mỗi 0,05 s, kể cả khi Gazebo tạm dừng.
"""

from dataclasses import fields
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .behavior import BehaviorConfig, BehaviorController


class RobotBehaviorNode(Node):
    """Nhận dữ liệu ROS, cập nhật lõi điều khiển và publish lệnh vận tốc."""
    def __init__(self):
        """Đọc tham số, tạo publisher/subscriber và khởi động timer 20 Hz."""
        super().__init__('robot_behavior_node')
        defaults = BehaviorConfig()
        values = {}
        # Dùng các trường dataclass làm tên tham số ROS, tránh khai báo hai nơi bị lệch.
        for field in fields(defaults):
            values[field.name] = self.declare_parameter(
                field.name, getattr(defaults, field.name)).value
        self.controller = BehaviorController(BehaviorConfig(**values))
        # Queue sâu 1 hạn chế tích lũy lệnh cũ. String là nhãn, Twist là lệnh vận tốc.
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/robot_behavior/status', 1)
        self.create_subscription(String, '/detected_gesture', self.on_gesture, 1)
        # QoS cảm biến phù hợp các nguồn odometry/LiDAR dạng best-effort của mô phỏng.
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        # Không dùng /clock cho watchdog: Gazebo pause vẫn phải xử lý được dừng.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.on_timer, clock=self.steady_clock)
        self.last_status = None
        self.get_logger().info(
            'Ready: select then Thumb_Up. Open_Palm pauses; Thumb_Up resumes; fist cancels.')

    def on_gesture(self, message):
        """Đưa nhãn vào máy trạng thái; PAUSED và mọi trạng thái ngoài RUNNING đều phát 0 ngay."""
        self.controller.receive(message.data, time.monotonic())
        # Phát 0 ngay trong callback tạm dừng/hủy, không chờ thêm lượt timer 50 ms.
        if self.controller.state != 'RUNNING':
            self.publish_velocity(0.0, 0.0)

    def on_odom(self, message):
        """Chuẩn hóa quaternion, đổi sang yaw (rad), đưa vị trí/hướng vào bộ đo tiến độ."""
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        # Quaternion hợp lệ phải có độ dài khác 0; chuẩn hóa trước khi tính yaw.
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-6:
            self.controller.stop('Invalid odometry quaternion')
            self.controller.last_odom = None
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        # Yaw là hướng quay quanh trục z, đơn vị radian; robot chỉ chạy trên mặt phẳng.
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        self.controller.update_pose(p.x, p.y, yaw, time.monotonic())

    def on_scan(self, message):
        """Tìm vật cản gần nhất từ scan gần đủ 360°, cho cả tiến, lùi và quay.

        Yêu cầu ít nhất 80% mẫu hợp lệ. +inf được hiểu là không có phản hồi trong tầm;
        NaN và giá trị âm bị bỏ. Scan không đáng tin được quy thành khoảng cách 0.
        """
        ranges = message.ranges
        valid = [r for r in ranges if r >= 0 and not math.isnan(r)]
        # angle_increment tính bằng rad; 5,5 rad là ngưỡng gần đủ một vòng scan.
        coverage = abs(message.angle_increment) * max(0, len(ranges)-1)
        if (not ranges or len(valid) < len(ranges)*0.8 or coverage < 5.5
                or not math.isfinite(message.range_max) or message.range_max <= 0):
            # Không có scan đáng tin không được xem là đường trống: quy về 0 để chặn chạy.
            clearance = 0.0
        else:
            clearance = min(valid)
        self.controller.update_scan(clearance, time.monotonic())

    def publish_velocity(self, linear, angular):
        """Gán vận tốc tiến/lùi vào linear.x, vận tốc quay vào angular.z của Twist."""
        message = Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        self.velocity.publish(message)

    def on_timer(self):
        """Tính vận tốc, publish trạng thái mỗi lượt; chỉ ghi log khi trạng thái thay đổi."""
        # Dấu * tách cặp (v, omega) do lõi trả về thành hai đối số của publish_velocity.
        self.publish_velocity(*self.controller.step(time.monotonic()))
        status = self.controller.status
        self.status_pub.publish(String(data=status))
        # Vẫn publish trạng thái đều đặn cho UI, nhưng không in log 20 lần mỗi giây.
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


def main(args=None):
    """Khởi tạo ROS, xử lý callback đến khi dừng và giải phóng node/context."""
    rclpy.init(args=args)
    node = None
    try:
        node = RobotBehaviorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            # Nếu context ROS còn sống thì gửi 0 trước khi thoát. Khi process bị kill
            # hoặc context đã tắt, lớp dừng dự phòng là timeout của controller bánh xe.
            if rclpy.ok():
                node.publish_velocity(0.0, 0.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
