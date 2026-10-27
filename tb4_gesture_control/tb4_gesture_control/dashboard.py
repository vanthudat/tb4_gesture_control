"""Vẽ dashboard OpenCV từ ảnh, kết quả nhận diện và trạng thái robot.

Không đọc webcam, không subscribe/publish ROS và không thay đổi hành vi robot.
Hệ tọa độ vẽ: gốc trên-trái, x sang phải, y xuống dưới, đơn vị pixel.
Màu dùng thứ tự BGR của OpenCV; chữ ASCII tránh lỗi font có dấu.
"""

import math
import time

import cv2
import numpy as np


def color(hex_value):
    """Đổi chuỗi màu RRGGBB sang bộ ba BGR để dùng trong OpenCV."""
    return tuple(int(hex_value[i:i+2], 16) for i in (4, 2, 0))


# Bảng màu dùng chung: nền tối, chữ sáng, xanh hoạt động, vàng chờ, đỏ dừng/hủy.
BG = color('0b1120')
CARD = color('131e30')
EDGE = color('2a3850')
TEXT = color('eff5ff')
MUTED = color('91a2b9')
TEAL = color('37d9c5')
BLUE = color('70a9ff')
AMBER = color('ffca75')
RED = color('ff6e7b')
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Chỉ đổi cách hiển thị cho dễ đọc; các nhãn ROS gốc không bị đổi tên.
NAMES = {
    'No_Hand': 'No hand detected', 'Unknown': 'Recognizing...',
    'None': 'Unrecognized', 'Wave': 'Wave', 'Pointing_Up': 'Pointing Up',
    'Victory': 'Victory', 'Thumb_Up': 'Thumbs Up', 'Closed_Fist': 'Closed Fist',
    'Open_Palm': 'Open Palm', 'Camera_Error': 'Camera error',
}
# Các cặp chỉ số landmark cần nối để vẽ khung xương 21 điểm bàn tay.
CONNECTIONS = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),
               (5,9),(9,10),(10,11),(11,12),(9,13),(13,14),(14,15),(15,16),
               (13,17),(0,17),(17,18),(18,19),(19,20)]


def rounded(canvas, x, y, width, height, fill, radius=14):
    """Vẽ khối bo góc bằng hình chữ nhật và bốn hình tròn, không cần thư viện UI khác."""
    # Giới hạn bán kính để thanh confidence rất ngắn vẫn vẽ được bo góc hợp lệ.
    radius = min(radius, width//2, height//2)
    cv2.rectangle(canvas, (x+radius, y), (x+width-radius, y+height), fill, -1)
    cv2.rectangle(canvas, (x, y+radius), (x+width, y+height-radius), fill, -1)
    for cx, cy in ((x+radius, y+radius), (x+width-radius, y+radius),
                   (x+radius, y+height-radius), (x+width-radius, y+height-radius)):
        cv2.circle(canvas, (cx, cy), radius, fill, -1, cv2.LINE_AA)


def text(canvas, value, x, y, scale=0.5, ink=TEXT, weight=1, max_width=None):
    """Vẽ một dòng; giảm cỡ chữ rồi cắt bớt nếu vẫn dài hơn vùng max_width."""
    value = str(value)
    if max_width:
        # Đo chiều rộng chữ thật bằng OpenCV thay vì ước lượng theo số ký tự.
        while scale > 0.3 and cv2.getTextSize(value, FONT, scale, weight)[0][0] > max_width:
            scale -= 0.02
        while value and cv2.getTextSize(value, FONT, scale, weight)[0][0] > max_width:
            value = value[:-1]
    cv2.putText(canvas, value, (x, y), FONT, scale, ink, weight, cv2.LINE_AA)


def wrapped(canvas, value, x, y, width, lines=2, scale=0.46, ink=MUTED):
    """Tự xuống dòng theo chiều rộng pixel, giới hạn số dòng và thêm dấu ... khi dư."""
    words = str(value).split()
    line = ''
    rows = []
    for word in words:
        candidate = f'{line} {word}'.strip()
        if line and cv2.getTextSize(candidate, FONT, scale, 1)[0][0] > width:
            rows.append(line)
            line = word
        else:
            line = candidate
    if line:
        rows.append(line)
    if len(rows) > lines:
        rows = rows[:lines]
        rows[-1] = rows[-1].rstrip('.') + '...'
    for i, row in enumerate(rows):
        text(canvas, row, x, y+i*21, scale, ink, max_width=width)


class GestureDashboard:
    """Bố cục cố định 1200×800; OpenCV có thể co giãn cửa sổ khi hiển thị."""
    width = 1200
    height = 800

    def __init__(self, threshold=0.65):
        """Lưu ngưỡng confidence và bộ nhớ tính FPS, không tạo cửa sổ ở đây."""
        self.threshold = threshold
        self.last_frame = None
        self.fps = 0.0

    def render(self, frame, result, label, accepted, status, status_age=None):
        """Tạo canvas BGR rồi ghép camera, nhận diện, trạng thái và hướng dẫn.

        frame/result là dữ liệu hiện tại; label là nhãn thô, accepted là nhãn đã lọc.
        status_age=None nghĩa là chưa có trạng thái; quá 1,5 s sẽ báo dữ liệu cũ.
        Trả ảnh hoàn chỉnh cho node gọi cv2.imshow(), không sửa frame đầu vào.
        """
        now = time.monotonic()
        # FPS từ khoảng cách giữa các lượt render; làm mượt để con số không nhảy quá nhanh.
        if self.last_frame is not None:
            instantaneous = 1/max(now-self.last_frame, 1e-6)
            self.fps = instantaneous if self.fps == 0 else 0.85*self.fps+0.15*instantaneous
        self.last_frame = now
        # Ảnh OpenCV có dạng (cao, rộng, 3 kênh BGR); mỗi kênh uint8 nằm trong 0..255.
        canvas = np.full((self.height, self.width, 3), BG, dtype=np.uint8)
        hands = len(result.hand_landmarks)
        accent = RED if label in {'Open_Palm', 'Closed_Fist', 'Camera_Error'} else TEAL
        self._header(canvas)
        self._camera(canvas, frame, result.hand_landmarks, accent)
        self._recognition(canvas, result, label, accepted, hands, accent)
        state = self._behavior(canvas, status, status_age)
        self._guide(canvas, accepted, state)
        self._footer(canvas, state)
        return canvas

    def _header(self, canvas):
        """Vẽ biểu tượng robot và tiêu đề sản phẩm ở đầu cửa sổ."""
        rounded(canvas, 24, 24, 44, 44, TEAL, 12)
        cv2.rectangle(canvas, (34, 36), (58, 55), BG, 2, cv2.LINE_AA)
        for x in (41, 51):
            cv2.circle(canvas, (x, 44), 2, BG, -1, cv2.LINE_AA)
        cv2.line(canvas, (46, 30), (46, 35), BG, 2, cv2.LINE_AA)
        text(canvas, 'TURTLEBOT 4', 84, 43, 0.70, TEXT, 2)
        text(canvas, 'Hand gesture control', 84, 68, 0.48, MUTED)
        rounded(canvas, 967, 31, 209, 34, CARD, 10)
        cv2.circle(canvas, (985, 48), 4, TEAL, -1, cv2.LINE_AA)
        text(canvas, 'NON-VERBAL INTERACTION', 999, 53, 0.35, TEAL)
        cv2.line(canvas, (24, 89), (1176, 89), EDGE, 1)

    def _camera(self, canvas, frame, hands, accent):
        """Vẽ khung xương 21 landmark trên bản sao ảnh, rồi đặt ảnh vào panel camera."""
        rounded(canvas, 24, 112, 752, 604, CARD)
        cv2.circle(canvas, (45, 134), 4, TEAL, -1, cv2.LINE_AA)
        text(canvas, 'LIVE CAMERA', 58, 139, 0.42, TEXT, 1)
        text(canvas, f'{len(hands)} hand(s)  /  {self.fps:.0f} FPS', 590, 139, 0.41, MUTED)
        # Vẽ lên bản sao, không làm thay đổi ảnh gốc mà luồng nhận diện đang sử dụng.
        image = frame.copy()
        h, w = image.shape[:2]
        for hand in hands:
            # Đổi tọa độ chuẩn hóa của MediaPipe về pixel của ảnh webcam gốc.
            pixels = [(int(p.x*w), int(p.y*h)) for p in hand]
            for a, b in CONNECTIONS:
                cv2.line(image, pixels[a], pixels[b], (25, 35, 45), 5, cv2.LINE_AA)
                cv2.line(image, pixels[a], pixels[b], accent, 2, cv2.LINE_AA)
            for point in pixels:
                cv2.circle(image, point, 4, accent, -1, cv2.LINE_AA)
                cv2.circle(image, point, 2, TEXT, -1, cv2.LINE_AA)
        # Chọn tỷ lệ nhỏ hơn để ảnh lọt khung 728×540; phần dư để viền, không kéo méo ảnh.
        scale = min(728/w, 540/h)
        rw, rh = max(1, round(w*scale)), max(1, round(h*scale))
        x, y = 36+(728-rw)//2, 164+(540-rh)//2
        canvas[y:y+rh, x:x+rw] = cv2.resize(image, (rw, rh))
        if not hands:
            for cx, cy, dx, dy in ((x+24,y+24,1,1), (x+rw-24,y+24,-1,1),
                                   (x+24,y+rh-24,1,-1), (x+rw-24,y+rh-24,-1,-1)):
                cv2.line(canvas, (cx,cy), (cx+dx*24,cy), TEAL, 2, cv2.LINE_AA)
                cv2.line(canvas, (cx,cy), (cx,cy+dy*24), TEAL, 2, cv2.LINE_AA)
            rounded(canvas, 177, 642, 446, 40, CARD, 10)
            text(canvas, 'Place one open hand inside the camera view', 195, 668, 0.46, TEXT)

    def _recognition(self, canvas, result, label, accepted, hands, accent):
        """Hiển thị nhãn, score model tĩnh, vạch ngưỡng và kết quả đã publish.

        Chỉ dùng MODEL CONFIDENCE khi nhãn hiển thị đúng nhãn model. Với Wave hoặc
        nhãn suy ra bằng hình học thì ghi BASE MODEL để không gán nhầm confidence.
        """
        rounded(canvas, 800, 112, 376, 230, CARD)
        text(canvas, 'RECOGNITION', 820, 142, 0.42, MUTED)
        text(canvas, NAMES.get(label, label.replace('_', ' ')), 820, 182,
             0.90, TEXT, 2, max_width=336)
        note = 'Keep one hand visible'
        if hands == 0:
            note = 'Waiting for your hand'
        elif hands > 1:
            note = 'Use one hand to select or confirm'
        elif label == 'Wave':
            note = 'Motion pattern detected'
        text(canvas, note, 820, 207, 0.44, MUTED, max_width=336)

        # Tìm score từ kết quả model gốc, không dùng điểm của quy tắc Wave/hình học làm xác suất.
        candidates = [category for hand in result.gestures for category in hand[:1]]
        match = next((c for c in candidates if c.category_name == label), None)
        model = match or (candidates[0] if candidates else None)
        score = None if model is None or not math.isfinite(model.score) else min(1, max(0, model.score))
        exact = model is not None and model.category_name == label and label not in {'Wave', 'Unknown', 'None'}
        model_name = NAMES.get(model.category_name, model.category_name.replace('_', ' ')) if model else '-'
        # Chỉ gọi là confidence của nhãn đang hiện khi nó thực sự khớp nhãn model.
        caption = 'MODEL CONFIDENCE' if exact else f'BASE MODEL: {model_name}'
        text(canvas, caption, 820, 236, 0.36, MUTED, max_width=257)
        text(canvas, f'{score:.0%}' if score is not None else '--', 1098, 236,
             0.55, accent if score is not None else MUTED, 2)
        rounded(canvas, 820, 248, 336, 8, EDGE, 4)
        if score is not None and score > 0:
            rounded(canvas, 820, 248, max(2, round(336*score)), 8,
                    accent if score >= self.threshold else AMBER, 4)
        # Vạch trắng đánh dấu ngưỡng chấp nhận; chiều dài thanh thể hiện score hiện tại.
        threshold_x = 820+round(336*self.threshold)
        cv2.line(canvas, (threshold_x,245), (threshold_x,259), TEXT, 1)
        detail = 'Confidence of the static gesture model'
        if label == 'Wave':
            detail = 'Wave uses motion, not an AI probability'
        elif label == 'Open_Palm' and not exact:
            detail = 'Open palm supported by hand geometry'
        elif hands == 0:
            detail = 'No prediction available'
        text(canvas, detail, 820, 279, 0.36, MUTED, max_width=336)
        cv2.line(canvas, (820,291), (1156,291), EDGE, 1)
        # Published là kết quả lọc, không có nghĩa robot đã được xác nhận hoặc đang chạy.
        active = accepted not in {'No_Hand', 'Unknown', 'None'}
        cv2.circle(canvas, (825,315), 4, accent if active else MUTED, -1, cv2.LINE_AA)
        text(canvas, f'Published: {NAMES.get(accepted, accepted)}', 839, 320,
             0.44, accent if active else MUTED, max_width=316)

    def _behavior(self, canvas, status, status_age):
        """Phân tích chuỗi trạng thái ROS, chọn màu hiển thị và trả trạng thái cho thanh bước."""
        rounded(canvas, 800, 358, 376, 154, CARD)
        text(canvas, 'ROBOT STATUS', 820, 386, 0.42, MUTED)
        parts = status.split('|', 2)
        # Trạng thái quá cũ chỉ bị đánh dấu trên giao diện; watchdog chuyển động nằm ở node hành vi.
        connected = status_age is not None and status_age <= 1.5 and len(parts) == 3
        if connected:
            state, selected, reason = [part.strip() for part in parts]
            reason = reason.replace('Thumb_Up', 'Thumbs Up')
            title, ink = {'IDLE': ('Idle', MUTED), 'SELECTED': ('Awaiting confirmation', AMBER),
                          'RUNNING': ('Executing behavior', TEAL),
                          'PAUSED': ('Paused - progress saved', AMBER)}.get(state, ('Unknown state', AMBER))
            if 'STOP' in reason or 'CANCEL' in reason:
                title, ink = 'Stopped' if 'STOP' in reason else 'Cancelled', RED
            action = NAMES.get(selected, selected.replace('_', ' '))
            text(canvas, title, 836, 420, 0.59, ink, 2, max_width=320)
            cv2.circle(canvas, (825,414), 4, ink, -1, cv2.LINE_AA)
            if state in {'SELECTED', 'RUNNING', 'PAUSED'}:
                reason = f'{action}. {reason}'
        else:
            # Ở behavior:=false, chưa có tín hiệu hành vi là bình thường; AI vẫn hoạt động.
            state = 'OFFLINE'
            title = 'No behavior signal' if status_age is None else 'Behavior signal lost'
            text(canvas, title, 820, 420, 0.59, AMBER, 2, max_width=336)
            reason = ('Recognition is available. Start the behavior node to control the robot.'
                      if status_age is None else 'Robot status is stale. Check the behavior node.')
        wrapped(canvas, reason, 820, 453, 336, lines=3, scale=0.43)
        return state

    def _guide(self, canvas, accepted, state):
        """Vẽ sáu cử chỉ và làm nổi dòng trùng nhãn đã được bộ lọc chấp nhận."""
        rounded(canvas, 800, 528, 376, 188, CARD)
        text(canvas, 'GESTURE GUIDE', 820, 557, 0.42, MUTED)
        rows = [('Pointing_Up', '1 finger', 'Back / forth'),
                ('Victory', '2 fingers', 'Circle'), ('Wave', 'Wave', 'Greeting'),
                ('Thumb_Up', 'Thumbs up', 'Resume' if state == 'PAUSED' else 'Confirm'),
                ('Open_Palm', 'Open palm', 'Pause'), ('Closed_Fist', 'Fist', 'Cancel')]
        for i, (label, gesture, action) in enumerate(rows):
            y = 581+i*23
            ink = RED if label == 'Closed_Fist' else AMBER if label == 'Open_Palm' else TEAL if label == 'Thumb_Up' else MUTED
            # Làm nổi cử chỉ đã publish, giúp phân biệt nhãn thô với kết quả sau lọc.
            if label == accepted:
                rounded(canvas, 812, y-15, 352, 21, EDGE, 5)
                ink = RED if label == 'Closed_Fist' else AMBER if label == 'Open_Palm' else TEAL
            text(canvas, gesture, 824, y, 0.43, TEXT)
            text(canvas, action, 1014, y, 0.43, ink)

    def _footer(self, canvas, state):
        """Làm nổi bước chọn/xác nhận/thực hiện; nhắc giữ tay và cử chỉ dừng."""
        rounded(canvas, 24, 740, 1152, 40, CARD, 10)
        for x, value, active in ((42, '01  SELECT', state == 'IDLE'),
                                 (198, '02  RESUME' if state == 'PAUSED' else '02  CONFIRM',
                                  state in {'SELECTED', 'PAUSED'}),
                                 (365, '03  EXECUTE', state == 'RUNNING')):
            text(canvas, value, x, 766, 0.42, TEAL if active else MUTED)
        text(canvas, 'Keep hand visible. Palm: pause. Thumb: resume. Fist: cancel.',
             648, 766, 0.40, MUTED, max_width=510)
