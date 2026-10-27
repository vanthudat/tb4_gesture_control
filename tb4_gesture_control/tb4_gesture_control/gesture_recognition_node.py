"""Node Phần 2: webcam → MediaPipe → lọc cử chỉ → /detected_gesture.

Node này chỉ phát nhãn, không phát vận tốc. Việc robot được chạy hay không do
robot_behavior_node quyết định. Dashboard chỉ quan sát kết quả và trạng thái.
"""

import math
import time
from pathlib import Path

import cv2
import mediapipe as mp
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .camera import open_camera
from .dashboard import GestureDashboard
from .gesture_filter import (
    CAMERA_ERROR, NO_HAND, UNKNOWN, StableGesture, WaveDetector, open_hand_geometry,
)
from .model import default_model_path


WINDOW = 'TurtleBot 4 | Gesture Control'


class GestureRecognitionNode(Node):
    """Sở hữu webcam, model MediaPipe, bộ lọc thời gian và cửa sổ giao diện."""
    def __init__(self):
        """Đọc tham số, kiểm tra model, mở camera và tạo timer đọc ảnh."""
        super().__init__('gesture_recognition_node')
        # Tham số có thể đọc từ YAML hoặc ghi đè bằng --ros-args -p khi chạy node.
        camera_index = self.declare_parameter('camera_index', 0).value
        model_path = self.declare_parameter('model_path', '').value
        self.show_window = self.declare_parameter('show_window', True).value
        self.threshold = self.declare_parameter('confidence_threshold', 0.65).value
        hold = self.declare_parameter('stable_seconds', 0.35).value
        wave_window = self.declare_parameter('wave_window', 2.0).value
        excursion = self.declare_parameter('wave_excursion', 0.04).value
        tilt_excursion = self.declare_parameter('wave_tilt_excursion', 0.30).value
        if (not 0 < self.threshold <= 1 or hold <= 0 or wave_window <= 0
                or excursion <= 0 or tilt_excursion <= 0):
            raise ValueError('Invalid confidence/temporal filter parameters')
        # Không tải model trong lúc demo; báo rõ nếu chưa chạy lệnh tải trước đó.
        path = Path(model_path).expanduser() if model_path else default_model_path()
        if not path.is_file():
            raise FileNotFoundError(
                f'Model missing: {path}. Run: ros2 run tb4_gesture_control download_gesture_model')
        # Publish đều mỗi frame, kể cả No_Hand/Unknown; node hành vi tự đánh giá mất tín hiệu.
        self.publisher = self.create_publisher(String, '/detected_gesture', 1)
        self.status = 'Waiting for robot_behavior_node'
        self.last_status_at = None
        self.dashboard = GestureDashboard(threshold=self.threshold)
        self.create_subscription(String, '/robot_behavior/status', self.on_status, 1)
        # Bộ lọc nhãn tĩnh và bộ phát hiện Wave giữ bộ nhớ qua nhiều frame.
        self.filter = StableGesture(hold_seconds=hold)
        self.wave = WaveDetector(window=wave_window, excursion=excursion,
                                 tilt_excursion=tilt_excursion)
        self.capture = None
        self.recognizer = None
        self.last_timestamp_ms = -1
        try:
            # VIDEO dùng recognize_for_video đồng bộ. Tìm tối đa hai tay để tay thứ hai
            # vẫn được dừng/hủy, nhưng chỉ một tay được phép chọn hoặc xác nhận.
            options = mp.tasks.vision.GestureRecognizerOptions(
                base_options=mp.tasks.BaseOptions(model_asset_path=str(path)),
                running_mode=mp.tasks.vision.RunningMode.VIDEO,
                num_hands=2,
                min_hand_detection_confidence=0.6,
                min_hand_presence_confidence=0.6,
                min_tracking_confidence=0.6)
            self.recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
            self.capture = open_camera(camera_index)
            if self.show_window:
                # Cửa sổ được đổi kích thước; GUI_NORMAL bỏ thanh công cụ ảnh mặc định.
                cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
                cv2.resizeWindow(WINDOW, self.dashboard.width, self.dashboard.height)
        except Exception:
            self.close_resources()
            raise
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        # Mục tiêu 20 lượt/giây; FPS thật còn phụ thuộc webcam và thời gian suy luận.
        self.create_timer(1/20, self.on_frame, clock=self.steady_clock)
        self.get_logger().info('Webcam active. Use one hand. No keyboard commands needed.')

    def on_status(self, message):
        """Lưu trạng thái robot và thời điểm nhận để dashboard biết khi nào dữ liệu cũ."""
        self.status = message.data
        self.last_status_at = time.monotonic()

    def classify(self, result, now):
        """Kết hợp nhãn AI, hình học bàn tay và lịch sử chuyển động.

        Trả (nhãn, score model tĩnh), không coi score đó là confidence riêng của Wave.
        Mất/nhiều tay sẽ reset Wave; một trong hai tay vẫn được phép dừng/hủy.
        Khi robot RUNNING/PAUSED, bàn tay xòe ưu tiên tạm dừng thay vì chọn Wave.
        """
        # Không có tay: trả No_Hand ngay, không tái sử dụng kết quả cũ.
        if not result.hand_landmarks:
            self.wave.update(None, now, False)
            return NO_HAND, 0.0
        # Mỗi tay có danh sách category; phần tử đầu là nhãn tĩnh đứng đầu của tay đó.
        candidates = []
        for categories in result.gestures:
            top = categories[0] if categories else None
            candidates.append((top.category_name, top.score) if top else (UNKNOWN, 0.0))
        # Ưu tiên dừng trước CANCEL. Với nhiều tay/robot đang chạy hoặc tạm dừng, không dùng Wave
        # để trì hoãn một bàn tay xòe đã được AI nhận chắc chắn.
        for stop in ('Open_Palm', 'Closed_Fist'):
            if any(label == stop and score >= self.threshold for label, score in candidates):
                if len(result.hand_landmarks) > 1 or self.status.startswith(('RUNNING', 'PAUSED')):
                    self.wave.update(None, now, False)
                    return stop, max(score for label, score in candidates if label == stop)
        # Hai tay không được tạo lựa chọn/xác nhận; cũng không nối lịch sử Wave giữa hai tay.
        if len(result.hand_landmarks) != 1 or not candidates:
            self.wave.update(None, now, False)
            return UNKNOWN, 0.0
        label, score = candidates[0]
        # Score thấp thành Unknown; hình học bàn tay mở bên dưới vẫn có thể hỗ trợ Wave.
        if score < self.threshold:
            label = UNKNOWN
        points = result.hand_landmarks[0]
        # Lấy trung bình cổ tay và bốn gốc ngón để đo vị trí ngang của lòng bàn tay.
        center_x = sum(points[i].x for i in (0, 5, 9, 13, 17))/5
        palm_length = math.hypot(points[9].x - points[0].x, points[9].y - points[0].y)
        finger_x = sum(points[i].x for i in (8, 12, 16, 20))/4
        # Độ nghiêng = lệch ngang trung bình đầu ngón so với cổ tay / chiều dài lòng bàn tay.
        # Chia theo kích thước tay giúp lắc cổ tay gần/xa camera có ngưỡng tương đối giống nhau.
        tilt = (finger_x - points[0].x) / max(palm_length, 0.03)
        # Chỉ dùng hình học khi model chưa rõ; không ghi đè một cử chỉ khác đã được nhận chắc.
        geometry_palm = (label in {UNKNOWN, 'None'} and open_hand_geometry(points))
        is_palm = label == 'Open_Palm' or geometry_palm
        # Đang chạy/tạm dừng thì bàn tay mở giữ robot dừng; không thu thập Wave
        # để vô tình thay kế hoạch. Muốn chọn hành vi khác phải nắm tay hủy trước.
        if self.status.startswith(('RUNNING', 'PAUSED')):
            self.wave.update(None, now, False)
            return ('Open_Palm' if is_palm else label), score
        # Unknown được giữ lịch sử ngắn; cử chỉ hợp lệ khác truyền None để xóa lịch sử Wave.
        wave_x = center_x if is_palm or label in {UNKNOWN, 'None'} else None
        waving = self.wave.update(wave_x, now, is_palm, tilt=tilt)
        if waving:
            return 'Wave', score
        return ('Open_Palm' if is_palm else label), score

    def on_frame(self):
        """Xử lý một frame: đọc → lật → RGB → suy luận → lọc → publish → vẽ.

        Timestamp MediaPipe phải tăng dần. Khi đọc/suy luận/giao diện lỗi thì gửi
        Camera_Error, đóng tài nguyên rồi kết thúc; node hành vi có watchdog độc lập.
        """
        try:
            ok, frame = self.capture.read()
            if not ok:
                raise RuntimeError('Webcam read failed')
            # Ảnh như gương giúp người dùng dễ thao tác; MediaPipe cần RGB thay cho BGR của OpenCV.
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Đổi giây sang mili-giây và buộc tăng ít nhất 1 ms theo yêu cầu VIDEO của MediaPipe.
            timestamp = max(self.last_timestamp_ms+1, int(time.monotonic()*1000))
            self.last_timestamp_ms = timestamp
            result = self.recognizer.recognize_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp)
            now = time.monotonic()
            raw, _ = self.classify(result, now)
            # raw là kết quả hiện tại; accepted là nhãn đã qua độ ổn định và được gửi lên ROS.
            accepted = self.filter.update(raw, now)
            self.publisher.publish(String(data=accepted))
            if self.show_window:
                status_age = None if self.last_status_at is None else now-self.last_status_at
                display = self.dashboard.render(
                    frame, result, raw, accepted, self.status, status_age=status_age)
                cv2.imshow(WINDOW, display)
                # Chỉ xử lý sự kiện cửa sổ, không đọc phím để điều khiển robot.
                cv2.waitKey(1)
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    raise RuntimeError('Webcam window closed')
        except Exception as error:
            # Cố thông báo lỗi trước khi thoát; nếu bản tin không tới, watchdog bên hành vi vẫn dừng.
            self.publisher.publish(String(data=CAMERA_ERROR))
            # Không tiếp tục dùng model/camera đã lỗi; để process kết thúc sau khi dọn tài nguyên.
            self.get_logger().error(str(error))
            self.close_resources()
            raise

    def close_resources(self):
        """Đóng camera, model và cửa sổ; có thể gọi lại nếu tài nguyên đã đóng."""
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if self.recognizer is not None:
            self.recognizer.close()
            self.recognizer = None
        if self.show_window:
            cv2.destroyAllWindows()


def main(args=None):
    """Chạy vòng callback ROS và giải phóng tài nguyên khi thoát hoặc nhận Ctrl+C."""
    rclpy.init(args=args)
    node = None
    try:
        node = GestureRecognitionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publisher.publish(String(data=CAMERA_ERROR))
            node.close_resources()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
