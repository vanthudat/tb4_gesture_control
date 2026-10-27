"""Mở webcam dùng chung và chương trình xem ảnh cho Phần 1.

webcam_check chỉ kiểm tra ảnh OpenCV, không chạy MediaPipe hay điều khiển robot.
"""

import cv2


def open_camera(index, width=640, height=480):
    """Mở /dev/video{index} bằng V4L2 trên Ubuntu và trả đối tượng VideoCapture.

    Yêu cầu ảnh 640×480, buffer nhỏ để hạn chế ảnh cũ; thiết bị có thể không hỗ trợ
    mọi thiết lập nên kích thước/FPS thực tế vẫn tùy driver webcam.
    """
    # V4L2 là backend camera của Linux; index=0 thường ứng với /dev/video0.
    capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(
            f'Cannot open /dev/video{index}. Check camera_index, permissions, '
            'and close other applications using the webcam.')
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Yêu cầu buffer nhỏ để giảm độ trễ; driver có thể bỏ qua thiết lập này.
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def main():
    """Đọc --camera-index, xem ảnh như gương và đóng tài nguyên khi thoát cửa sổ."""
    import argparse
    parser = argparse.ArgumentParser(description='Part 1: webcam preview only')
    parser.add_argument('--camera-index', type=int, default=0)
    args = parser.parse_args()
    capture = open_camera(args.camera_index)
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError('Webcam stopped delivering frames')
            # Lật ngang để hiển thị như gương; dữ liệu vẫn chỉ dùng xem camera, không phát lệnh ROS.
            cv2.imshow('Webcam check (close window to exit)', cv2.flip(frame, 1))
            # Bơm sự kiện để cửa sổ không bị treo; đóng cửa sổ là cách kết thúc chương trình.
            cv2.waitKey(1)
            if cv2.getWindowProperty('Webcam check (close window to exit)', cv2.WND_PROP_VISIBLE) < 1:
                break
    # Luôn trả camera cho ứng dụng khác, cả khi đọc ảnh lỗi hoặc người dùng thoát.
    finally:
        capture.release()
        cv2.destroyAllWindows()
