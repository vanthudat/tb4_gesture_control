"""Lệnh tải model Gesture Recognizer chính thức trước khi demo.

Được gọi bởi ros2 run tb4_gesture_control download_gesture_model.
Không tải tự động trong vòng lặp nhận diện để tránh phụ thuộc mạng khi demo.
"""

import argparse
import os
from pathlib import Path
import tempfile
import urllib.request
import zipfile


# URL bundle Gesture Recognizer phiên bản 1 từ kho model chính thức của Google.
MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/gesture_recognizer/'
             'gesture_recognizer/float16/1/gesture_recognizer.task')


def default_model_path():
    """Ưu tiên XDG_CACHE_HOME, nếu chưa đặt thì dùng ~/.cache của người dùng."""
    cache = Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache'))
    return cache / 'tb4_gesture_control' / 'gesture_recognizer.task'


def main():
    """Tải từng khối vào file tạm, kiểm tra định dạng bundle rồi đổi tên sang đích.

    --output cho phép chọn nơi lưu khác. File tạm nằm cùng thư mục đích để việc
    đổi tên sau khi tải xong không để lộ model mới đang còn tải dở.
    """
    parser = argparse.ArgumentParser(description='Download the official MediaPipe model')
    parser.add_argument('--output', type=Path, default=default_model_path())
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        # Dùng file tạm riêng để tải thất bại không ghi đè model đang dùng.
        with tempfile.NamedTemporaryFile(dir=args.output.parent, delete=False) as stream:
            temporary = Path(stream.name)
            with urllib.request.urlopen(MODEL_URL, timeout=60) as response:
                while True:
                    # Đọc từng khối 1 MiB thay vì giữ toàn bộ model trong RAM.
                    chunk = response.read(1024*1024)
                    if not chunk:
                        break
                    stream.write(chunk)
        # Bundle .task có dạng ZIP. Đây chỉ là kiểm tra định dạng; tính tương thích
        # với GestureRecognizer còn được MediaPipe kiểm tra khi nạp model.
        if not zipfile.is_zipfile(temporary):
            raise RuntimeError('Downloaded file is not a MediaPipe task bundle')
        # Chỉ đổi sang tên đích sau khi tải xong, tránh để node nhận diện đọc file dở.
        temporary.replace(args.output)
        print(f'Model saved: {args.output}')
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


if __name__ == '__main__':
    main()
