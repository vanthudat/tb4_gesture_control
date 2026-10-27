"""Lõi điều khiển hành vi: chọn → xác nhận → thực hiện → trở về chờ.

File này không gọi ROS hay webcam. Node ROS đưa cử chỉ/cảm biến vào lớp
BehaviorController rồi gọi step() để lấy cặp vận tốc (m/s, rad/s).
Thời gian now luôn là giây từ time.monotonic(), không phải thời gian Gazebo.
"""

from dataclasses import dataclass
import math

from .gesture_filter import CAMERA_ERROR, GESTURES, NO_HAND, UNKNOWN


@dataclass
class BehaviorConfig:
    """Các giá trị mặc định; node ROS có thể ghi đè bằng config/gestures.yaml."""
    linear_speed: float = 0.30  # m/s; dương là tiến, âm là lùi khi tạo chặng.
    angular_speed: float = 1.0  # rad/s; dương quay trái, âm quay phải.
    travel_distance: float = 0.30  # Mét cho MỖI chặng tiến hoặc lùi.
    cycles: int = 4  # 4 chu kỳ tiến rồi lùi, tổng cộng 8 chặng.
    circle_radius: float = 0.60  # Bán kính vòng tròn, đơn vị mét.
    wave_angle: float = 0.40  # Góc chào A, đơn vị rad; các chặng là +A, -2A, +A.
    signal_timeout: float = 0.80  # Giây tối đa không có nhãn điều khiển hợp lệ.
    odom_timeout: float = 0.50  # Giây tối đa không cập nhật odometry.
    scan_timeout: float = 0.50  # Giây tối đa không cập nhật LiDAR.
    selection_timeout: float = 8.0  # Giây chờ xác nhận kể từ lúc chọn hành vi.
    segment_timeout: float = 60.0  # Giới hạn giây cho một chặng để tránh chạy mãi khi kẹt.
    obstacle_distance: float = 0.50  # Khoảng trống tối thiểu từ LiDAR, đơn vị mét.
    rearm_seconds: float = 0.20  # Giây ở nhãn trung tính trước khi nhận lại cùng cử chỉ.
    require_scan: bool = True  # Mặc định phải có LiDAR hợp lệ mới cho robot chạy.

    def __post_init__(self):
        """Kiểm tra tham số ngay khi tạo cấu hình để không chạy với giá trị sai."""
        # Chặn NaN, vô cực, số âm/0 trước khi tạo kế hoạch chuyển động.
        for name, value in vars(self).items():
            if name == 'require_scan':
                continue
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not isinstance(self.cycles, int):
            raise ValueError('cycles must be an integer')
        if self.linear_speed > 0.3 or self.angular_speed > 1.0:
            raise ValueError('Demo speed limits: linear <= 0.3 m/s, angular <= 1 rad/s')


@dataclass
class Segment:
    """Một chặng: chạy đủ quãng đường hoặc góc đích với vận tốc đã đặt."""
    kind: str  # 'distance': đo mét; 'angle': đo radian.
    target: float  # Ngưỡng kết thúc chặng, cùng đơn vị với kind.
    linear: float  # Vận tốc linear.x cho chặng, m/s.
    angular: float  # Vận tốc angular.z cho chặng, rad/s.


class BehaviorController:
    """Máy trạng thái IDLE → SELECTED → RUNNING ↔ PAUSED.

    IDLE: chờ chọn. SELECTED: đã chọn nhưng chưa chạy. RUNNING: thực hiện kế hoạch.
    PAUSED: vận tốc bằng 0, giữ kế hoạch/tiến độ, đợi Thumb_Up để tiếp tục.
    Closed_Fist và lỗi an toàn xóa kế hoạch, đưa về IDLE.
    Kế hoạch chỉ được tạo khi nhận Thumb_Up và odometry/LiDAR còn hợp lệ.
    """
    def __init__(self, config=None):
        """Khởi tạo trạng thái, mốc cảm biến, bộ đo tiến độ và bộ nhớ chống lặp."""
        self.cfg = config or BehaviorConfig()
        # Trạng thái và lựa chọn độc lập: SELECTED vẫn chưa được phép chuyển động.
        self.state = 'IDLE'
        self.selected = None
        self.reason = 'Waiting for selection'
        self.selected_at = 0.0
        # Mốc nhận gần nhất của từng nguồn, dùng cho các watchdog riêng.
        self.last_signal = None
        self.last_odom = None
        self.last_scan = None
        self.clearance = 0.0
        # Tổng quãng đường và góc quay được tích lũy từ các pose liên tiếp.
        # Mỗi chặng lưu mốc ban đầu rồi lấy hiệu; không cần reset bộ đo chung.
        self.pose = None
        self.path_length = 0.0
        self.yaw_total = 0.0
        # plan chứa các Segment; index là chặng đang thực hiện.
        self.plan = []
        self.index = 0
        self.segment_start = None
        self.segment_at = 0.0
        self.paused_at = None  # Mốc tạm dừng để không tính thời gian nghỉ vào timeout chặng.
        # Nhớ cử chỉ vừa xử lý để giữ nguyên tay không kích hoạt nhiều lần.
        self.last_event = None
        self.neutral_since = None

    @property
    def status(self):
        """Chuỗi STATE | lựa chọn | lý do, để node ROS publish và dashboard hiển thị."""
        action = self.selected or '-'
        return f'{self.state} | {action} | {self.reason}'

    def stop(self, reason):
        """Hủy hoàn toàn kế hoạch/lựa chọn khi có Closed_Fist, lỗi hoặc hoàn thành.

        Hàm này chỉ đổi trạng thái; node ROS sẽ phát vận tốc 0 từ trạng thái đó.
        """
        self.state = 'IDLE'
        self.selected = None
        self.plan = []
        self.segment_start = None
        self.paused_at = None
        self.reason = reason

    def pause(self, now):
        """Giữ nguyên chặng và mốc odometry; node ROS lập tức phát vận tốc 0.

        Chỉ lưu paused_at lần đầu: giữ nguyên bàn tay xòe không làm đổi mốc nghỉ.
        Odometry vẫn được cập nhật nên phần robot trôi khi giảm tốc vẫn được tính.
        """
        if self.state == 'RUNNING':
            self.state = 'PAUSED'
            self.paused_at = now
            self.reason = (f'Paused at segment {self.index+1}/{len(self.plan)}; '
                           'Thumb_Up resumes, fist cancels')
        elif self.state != 'PAUSED':
            # Chưa có chuyển động để giữ tiến độ thì chỉ xóa lựa chọn như trước.
            self.stop('STOP: open palm; no running behavior')

    def resume(self, now):
        """Tiếp tục đúng kế hoạch cũ sau một lần xác nhận mới và kiểm tra cảm biến.

        Không gọi _make_plan(), không reset index hoặc segment_start.
        Thời gian nghỉ được loại khỏi timeout; thời gian chạy trước nghỉ vẫn giữ.
        """
        if self.state != 'PAUSED':
            return
        problem = self._ready(now)
        if problem:
            self.reason = f'Paused: {problem}; show Thumb_Up again when ready'
            return
        if self.segment_start is not None:
            self.segment_at += now - self.paused_at
        self.paused_at = None
        self.state = 'RUNNING'
        self.reason = 'Resumed; keep a recognized hand visible'

    def update_pose(self, x, y, yaw, now):
        """Nhận pose mới, cộng quãng đường/góc quay và phát hiện odometry nhảy bất thường."""
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.stop('Invalid odometry')
            self.last_odom = None
            return
        if self.pose is not None:
            px, py, pa = self.pose
            distance = math.hypot(x - px, y - py)
            # Đưa chênh lệch góc về [-pi, pi] để qua biên yaw không bị nhảy gần 2*pi.
            angle = math.atan2(math.sin(yaw - pa), math.cos(yaw - pa))
            # Pose nhảy quá lớn giữa hai bản tin có thể do reset/teleport; hủy lượt chạy.
            if distance > 0.5 or abs(angle) > 0.7:
                self.stop('Odometry jump: select and confirm again')
            else:
                self.path_length += distance
                self.yaw_total += angle
        self.pose = (x, y, yaw)
        self.last_odom = now

    def update_scan(self, clearance, now):
        """Lưu khoảng cách vật cản gần nhất; NaN hoặc số âm được xem là không an toàn."""
        self.clearance = 0.0 if math.isnan(clearance) or clearance < 0 else clearance
        self.last_scan = now

    def _ready(self, now):
        """Trả None nếu cảm biến đủ mới và đủ khoảng trống; ngược lại trả lý do từ chối."""
        if self.last_odom is None or now - self.last_odom > self.cfg.odom_timeout:
            return 'Odometry missing/stale'
        if self.cfg.require_scan:
            if self.last_scan is None or now - self.last_scan > self.cfg.scan_timeout:
                return 'Laser scan missing/stale'
            if self.clearance < self.cfg.obstacle_distance:
                return 'Obstacle too close'
        return None

    def _make_plan(self, selection):
        """Chuyển lựa chọn thành danh sách các chặng; chưa phát lệnh chuyển động ở đây."""
        c = self.cfg
        # Mỗi chu kỳ có một chặng tiến (+v), một chặng lùi (-v).
        if selection == 'Pointing_Up':
            return [segment for _ in range(c.cycles) for segment in (
                Segment('distance', c.travel_distance, c.linear_speed, 0.0),
                Segment('distance', c.travel_distance, -c.linear_speed, 0.0))]
        if selection == 'Victory':
            # Quan hệ đi tròn: omega = v/R. Giới hạn v để omega không vượt cấu hình.
            speed = min(c.linear_speed, c.angular_speed * c.circle_radius)
            return [Segment('angle', 2 * math.pi, speed, speed / c.circle_radius)]
        # Lựa chọn còn lại hợp lệ là Wave: tổng góc +A - 2A + A gần bằng 0.
        return [Segment('angle', c.wave_angle, 0.0, c.angular_speed),
                Segment('angle', 2 * c.wave_angle, 0.0, -c.angular_speed),
                Segment('angle', c.wave_angle, 0.0, c.angular_speed)]

    def receive(self, label, now):
        """Xử lý một nhãn nhận diện đã lọc cùng thời điểm nhận.

        Thứ tự: kiểm tra nhãn/mất tín hiệu → ưu tiên dừng/hủy → chống lặp → chọn/xác nhận.
        No_Hand và Unknown không gia hạn tín hiệu hợp lệ; Camera_Error hủy ngay.
        Một Thumb_Up giữ nguyên không được coi là nhiều lần xác nhận.
        """
        # Không cho chuỗi lạ từ topic đóng vai tín hiệu duy trì chuyển động.
        if label not in GESTURES | {NO_HAND, UNKNOWN, CAMERA_ERROR}:
            self.stop('Invalid gesture message')
            return
        # Khoảng mất tín hiệu đã vượt hạn thì hủy lựa chọn trước khi xử lý nhãn mới.
        # Giữ last_event để không biến Thumbs Up đang giữ thành một lần xác nhận mới.
        # PAUSED đã có vận tốc 0: cho phép nghỉ/bỏ tay lâu mà vẫn giữ tiến độ.
        # Khi đang chạy, mất tín hiệu vẫn hủy hành vi như trước, không tự tiếp tục.
        if (self.state != 'PAUSED' and self.last_signal is not None
                and now - self.last_signal > self.cfg.signal_timeout):
            self.stop('Control signal timeout')
        # Không cập nhật last_signal khi mất tay. Camera_Error hủy ngay; mất tay
        # được phép gián đoạn ngắn rồi timer sẽ dừng sau signal_timeout.
        if label in {NO_HAND, CAMERA_ERROR}:
            if label == CAMERA_ERROR:
                self.stop('Camera error')
            elif (self.state != 'PAUSED' and self.last_signal is not None
                  and now - self.last_signal > self.cfg.signal_timeout):
                self.stop('Hand lost')
            self._neutral(now)
            return
        # Cho phép đổi tư thế tay ngắn hạn; Unknown kéo dài không duy trì quyền chạy.
        if label == UNKNOWN:
            self._neutral(now)
            return
        # Chỉ nhãn thuộc nhóm cử chỉ hợp lệ mới gia hạn tín hiệu điều khiển.
        self.last_signal = now
        self.neutral_since = None
        # Wave cũng dùng bàn tay xòe: nếu nhãn đến muộn lúc đang chạy thì vẫn dừng.
        if label == 'Wave' and self.state in {'RUNNING', 'PAUSED'}:
            self.last_event = label
            self.pause(now)
            return
        # Dừng/hủy được xử lý trước bộ chống lặp để luôn có quyền ưu tiên.
        if label in {'Open_Palm', 'Closed_Fist'}:
            self.last_event = label
            if label == 'Open_Palm':
                self.pause(now)
            else:
                self.stop('CANCEL: closed fist')
            return
        # Cùng cử chỉ đã xử lý thì bỏ qua sự kiện, nhưng last_signal vẫn vừa được
        # cập nhật ở trên nên giữ Thumbs Up vẫn duy trì quyền chạy.
        if label == self.last_event:
            return
        self.last_event = label
        # Khi tạm dừng, chỉ Thumb_Up được tiếp tục; muốn chọn hành vi khác thì
        # phải Closed_Fist để hủy trước. Không vô tình thay kế hoạch đã giữ lại.
        if self.state == 'PAUSED':
            if label == 'Thumb_Up':
                self.resume(now)
            return
        # Trong lúc chạy, không cho cử chỉ chọn khác tự thay đổi kế hoạch.
        if self.state == 'RUNNING':
            return
        if label in {'Wave', 'Pointing_Up', 'Victory'}:
            self.selected, self.selected_at = label, now
            self.state = 'SELECTED'
            self.reason = 'Show Thumb_Up to confirm'
        # Xác nhận chỉ có tác dụng nếu đã chọn trước đó; kiểm tra cả hạn chọn
        # và cảm biến rồi mới chuyển sang RUNNING.
        elif label == 'Thumb_Up' and self.selected is not None:
            if now - self.selected_at > self.cfg.selection_timeout:
                self.stop('Selection expired')
                return
            problem = self._ready(now)
            if problem:
                self.stop(problem)
                return
            self.plan = self._make_plan(self.selected)
            self.index = 0
            self.segment_start = None
            self.state = 'RUNNING'
            self.reason = 'Executing once; keep a recognized hand visible'

    def _neutral(self, now):
        """Cho phép nhận lại cùng cử chỉ sau một khoảng No_Hand/Unknown đủ lâu."""
        if self.neutral_since is None:
            self.neutral_since = now
        if now - self.neutral_since >= self.cfg.rearm_seconds:
            self.last_event = None

    def step(self, now):
        """Tính một bước điều khiển, trả (linear.x, angular.z) cho timer 20 Hz.

        Mỗi lần gọi đều kiểm tra watchdog trước khi cho chạy. Tiến độ lấy từ odometry.
        Không sleep hay chạy trọn hành vi trong hàm này để callback dừng vẫn được xử lý.
        """
        # PAUSED là dừng chủ động: không hết hạn vì mất tay hay lựa chọn quá lâu.
        # Mọi lỗi dữ liệu odometry/camera vẫn hủy qua các callback tương ứng.
        if self.state in {'SELECTED', 'RUNNING'}:
            if self.last_signal is None or now - self.last_signal > self.cfg.signal_timeout:
                self.stop('Control signal lost: select and confirm again')
            elif self.state == 'SELECTED' and now - self.selected_at > self.cfg.selection_timeout:
                self.stop('Selection expired')
        # IDLE/SELECTED/PAUSED luôn trả vận tốc 0, chỉ RUNNING mới được chạy.
        if self.state != 'RUNNING':
            return 0.0, 0.0
        problem = self._ready(now)
        if problem:
            self.stop(problem)
            return 0.0, 0.0
        segment = self.plan[self.index]
        # Chụp mốc tiến độ một lần khi bắt đầu chặng mới, không cập nhật mỗi tick.
        if self.segment_start is None:
            self.segment_start = (self.path_length, self.yaw_total)
            self.segment_at = now
        if now - self.segment_at > self.cfg.segment_timeout:
            self.stop('Movement timeout (blocked robot or bad odometry)')
            return 0.0, 0.0
        # Quãng đường là tổng chiều dài đã đi. Góc cần nhân dấu chiều quay
        # để cả chặng quay trái và quay phải đều có tiến độ dương.
        if segment.kind == 'distance':
            progress = self.path_length - self.segment_start[0]
        else:
            progress = (self.yaw_total - self.segment_start[1]) * (1 if segment.angular > 0 else -1)
        # Đạt đích thì chuyển chặng; phát 0 ở lượt này, lượt timer sau mới chạy tiếp.
        if progress >= segment.target:
            self.index += 1
            self.segment_start = None
            if self.index == len(self.plan):
                self.stop('Completed; select a new behavior')
            return 0.0, 0.0
        return segment.linear, segment.angular
