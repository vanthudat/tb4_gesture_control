"""Lọc nhãn theo thời gian và phát hiện Wave từ chuỗi landmarks.

Không phụ thuộc ROS/OpenCV. Các nhãn phải thống nhất với node hành vi.
Các ngưỡng thời gian tính bằng giây; tọa độ ảnh của MediaPipe được chuẩn hóa.
"""

from collections import deque
import math


# Tên nhãn phải giữ nguyên để node nhận diện và node hành vi hiểu cùng giao thức.
GESTURES = {'Wave', 'Pointing_Up', 'Victory', 'Thumb_Up', 'Open_Palm', 'Closed_Fist'}
NO_HAND = 'No_Hand'
UNKNOWN = 'Unknown'
CAMERA_ERROR = 'Camera_Error'


class StableGesture:
    """Chỉ chấp nhận cử chỉ chọn/xác nhận khi ổn định liên tục đủ thời gian."""

    def __init__(self, hold_seconds=0.35, max_gap=0.25):
        """hold_seconds là thời gian giữ nhãn; max_gap là khoảng gián đoạn frame cho phép."""
        self.hold_seconds = hold_seconds
        self.max_gap = max_gap
        # candidate: nhãn đang theo dõi; since: bắt đầu giữ; last: frame gần nhất.
        self.candidate = None
        self.since = 0.0
        self.last = None

    def update(self, label, now):
        """Nhận nhãn thô, trả nhãn ổn định hoặc Unknown nếu còn đang chờ.

        Không giữ nhãn cũ khi mất tay. Dừng/hủy đi thẳng qua bộ lọc; Wave đã có
        bộ kiểm tra chuỗi chuyển động riêng nên cũng không chờ thêm hold_seconds.
        """
        if label in {NO_HAND, CAMERA_ERROR}:
            self.candidate = None
            self.last = now
            return label
        # Không làm trễ STOP/CANCEL; Wave đã qua bộ kiểm tra chuyển động riêng.
        if label in {'Open_Palm', 'Closed_Fist', 'Wave'}:
            self.candidate = None
            self.last = now
            return label
        # Đổi nhãn hoặc mất frame lâu thì bắt đầu đếm ổn định lại từ đầu.
        if (label != self.candidate or self.last is None
                or now - self.last > self.max_gap):
            self.candidate, self.since = label, now
        self.last = now
        if label in GESTURES and now - self.since >= self.hold_seconds:
            return label
        return UNKNOWN


def open_hand_geometry(points):
    """Kiểm tra bàn tay mở khi model tĩnh khó nhận do bàn tay nghiêng.

    Ít nhất ba ngón ngoài ngón cái phải có dấu hiệu duỗi theo khoảng cách 2D.
    Đây là quy tắc hình học hỗ trợ, không phải xác suất phân loại của AI.
    """
    def distance(a, b):
        """Khoảng cách Euclid giữa hai landmark trong mặt phẳng ảnh chuẩn hóa."""
        return math.hypot(points[a].x - points[b].x, points[a].y - points[b].y)

    extended = 0
    # Chỉ số landmark: 0 là cổ tay; mỗi bộ là gốc ngón (MCP), khớp (PIP),
    # đầu ngón (TIP) của trỏ, giữa, áp út, út. So khoảng cách để chịu được nghiêng tay.
    for base, joint, tip in ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)):
        if (distance(tip, 0) > 1.15 * distance(joint, 0)
                and distance(tip, base) > 1.5 * max(distance(joint, base), 1e-6)):
            extended += 1
    return extended >= 3


class WaveDetector:
    """Nhận vẫy ngang hoặc lắc cổ tay bằng các lần đổi hướng đủ biên độ.

    Hai tín hiệu được đếm riêng: vị trí ngang x và độ lệch ngón/cổ tay tilt.
    Không cộng một lần đổi hướng của x với một lần của tilt để kết luận Wave.
    """

    def __init__(self, window=2.0, excursion=0.04, reversals=2, max_gap=0.35,
                 tilt_excursion=0.30, label_grace=0.20):
        """Lưu cửa sổ lịch sử và các ngưỡng phát hiện.

        window: giây; excursion: phần chiều rộng ảnh; reversals: số lần đảo chiều.
        tilt_excursion: biên độ theo chiều dài lòng bàn tay; label_grace: giây hụt nhãn.
        max_gap: khoảng frame/bằng chứng bàn tay mở bị gián đoạn quá lâu phải reset.
        """
        self.window, self.excursion = window, excursion
        self.reversals, self.max_gap = reversals, max_gap
        self.tilt_excursion, self.label_grace = tilt_excursion, label_grace
        # deque lưu (thời điểm, vị trí ngang, độ nghiêng); bỏ mẫu cũ từ đầu rất nhanh.
        self.samples = deque()
        self.last_frame = None
        self.last_palm = None

    def update(self, x, now, open_palm, tilt=None):
        """Nhận mẫu hiện tại và trả True chỉ khi đủ bằng chứng Wave.

        x=None dùng để xóa lịch sử khi mất tay, nhiều tay hoặc chuyển sang cử chỉ khác.
        open_palm=False với x còn hợp lệ chỉ giữ lịch sử ngắn, không phát Wave lúc đó.
        """
        # Mất tay hoặc gián đoạn frame quá lâu: không nối chuyển động cũ với tay mới.
        if (x is None or self.last_frame is not None
                and now - self.last_frame > self.max_gap):
            self.samples.clear()
            self.last_palm = None
        self.last_frame = now
        if x is None:
            return False
        # Hụt nhãn ngắn chỉ giữ lịch sử; frame này vẫn trả False để không phát Wave giả.
        if not open_palm:
            if self.last_palm is None or now - self.last_palm > self.label_grace:
                self.samples.clear()
            return False
        if self.last_palm is not None and now - self.last_palm > self.max_gap:
            self.samples.clear()
        self.last_palm = now
        self.samples.append((now, x, tilt))
        # Chỉ xét chuyển động trong cửa sổ thời gian gần nhất, không tích lũy vô hạn.
        while self.samples and now - self.samples[0][0] > self.window:
            self.samples.popleft()
        # Hai kiểu vẫy: dịch cả tay ngang hoặc lắc các ngón so với cổ tay tại chỗ.
        translated = self._turns([sample[1] for sample in self.samples], self.excursion)
        tilted = self._turns([sample[2] for sample in self.samples if sample[2] is not None],
                             self.tilt_excursion)
        return max(translated, tilted) >= self.reversals

    @staticmethod
    def _turns(values, excursion):
        """Đếm số lần đảo chiều sau khi đã vượt ngưỡng excursion.

        anchor bám cực trị theo hướng đang đi; dao động nhỏ hơn ngưỡng không được đếm.
        """
        if not values:
            return 0
        anchor = values[0]
        # direction = 0: chưa đủ biên độ; +1/-1: hướng đang tăng/giảm.
        direction = 0
        turns = 0
        for value in values:
            if direction == 0:
                if abs(value - anchor) >= excursion:
                    direction = 1 if value > anchor else -1
                    anchor = value
            # Vẫn cùng hướng thì bám điểm xa nhất, để đo biên độ đảo chiều từ cực trị.
            elif direction * (value - anchor) > 0:
                anchor = value
            # Chỉ tính đảo chiều khi đã đi ngược đủ ngưỡng; rung nhỏ không được cộng.
            elif direction * (value - anchor) <= -excursion:
                direction *= -1
                turns += 1
                anchor = value
        return turns
