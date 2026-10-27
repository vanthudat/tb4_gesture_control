# Điều khiển TurtleBot 4 bằng cử chỉ tay

Bài tập 6 – Giao tiếp phi ngôn ngữ giữa người và robot. Hệ thống dùng webcam, OpenCV và MediaPipe để nhận diện cử chỉ, sau đó điều khiển TurtleBot 4 trong Gazebo qua ROS 2.

```text
Webcam → gesture_recognition_node → /detected_gesture
        → robot_behavior_node → /cmd_vel → TurtleBot 4
```

## 1. Yêu cầu môi trường

- Ubuntu 22.04, ROS 2 Humble và Python 3.10.
- Gazebo Fortress và TurtleBot 4 Simulator đã được cài đặt từ bài thực hành trước.
- Webcam laptop hoặc USB hoạt động trên Ubuntu.
- Workspace `~/ros2_ws`, với mã nguồn đặt tại `~/ros2_ws/src/tb4_gesture_control`.
- Trong workspace đang sử dụng, giữ package `tb4_sim` của bài trước để dùng các cấu hình mô phỏng đã chuẩn bị.
- Có mạng khi cài thư viện và tải model lần đầu; khi chạy nhận diện, model được đọc từ máy.

README hướng dẫn chạy trên môi trường ROS 2 và TurtleBot 4 Simulator đã cài sẵn. Kiểm tra các package mô phỏng trước khi tiếp tục:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix turtlebot4_ignition_bringup
ros2 pkg prefix irobot_create_ignition_bringup
ros2 pkg prefix ros_gz_sim
```

Mỗi lệnh cần trả về đường dẫn cài package. Nếu báo `Package not found`, cần hoàn tất cài đặt hoặc source workspace chứa TurtleBot 4 Simulator trước.

## 2. Cài đặt thư viện

Cài công cụ Python và build nếu máy chưa có:

```bash
sudo apt update
sudo apt install python3-pip python3-colcon-common-extensions
```

Cài thư viện nhận diện bằng Python 3.10 của hệ thống:

```bash
cd ~/ros2_ws/src/tb4_gesture_control
python3 -m pip install --user -r requirements.txt
```

Các phiên bản được quản lý trong `requirements.txt`:

| Thư viện | Phiên bản |
| --- | --- |
| NumPy | `>=1.23,<2` |
| MediaPipe | `0.10.20` |
| OpenCV contrib | `>=4.8,<4.12` |

Dùng cùng môi trường Python khi cài thư viện, build và chạy ROS. Tránh cài thêm các wheel `opencv-python` hoặc `opencv-python-headless` song song với `opencv-contrib-python`, vì chúng cùng cung cấp module `cv2`.

Kiểm tra khả năng import:

```bash
python3 -c 'import cv2, mediapipe, numpy; print("OpenCV:", cv2.__version__); print("MediaPipe:", mediapipe.__version__); print("NumPy:", numpy.__version__)'
```

## 3. Build và tải model

Build từ thư mục gốc workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_sim tb4_gesture_control --symlink-install
source ~/ros2_ws/install/setup.bash
```

Lệnh trên dành cho workspace có cả hai package như phần yêu cầu môi trường. Nếu chỉ build lại package cử chỉ trong workspace đã chuẩn bị, dùng:

```bash
colcon build --packages-select tb4_gesture_control --symlink-install
source ~/ros2_ws/install/setup.bash
```

Tải model MediaPipe một lần trước khi chạy:

```bash
ros2 run tb4_gesture_control download_gesture_model
```

Khi thành công, chương trình in `Model saved:` kèm đường dẫn. Mặc định model nằm tại:

```text
~/.cache/tb4_gesture_control/gesture_recognizer.task
```

Nếu đặt biến `XDG_CACHE_HOME`, model nằm trong thư mục `tb4_gesture_control` bên dưới đường dẫn cache đó. Node nhận diện không tự tải model khi khởi chạy.

## 4. Chuẩn bị terminal

**Chạy ba lệnh sau trong mỗi terminal mới**, trước các lệnh ROS bên dưới:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=24
```

Các terminal chạy Gazebo, nhận diện, điều khiển và quan sát topic phải dùng cùng `ROS_DOMAIN_ID`.

## 5. Chạy chương trình

### 5.1. Khởi chạy Gazebo – Terminal 1

```bash
ros2 launch tb4_gesture_control gesture_sim.launch.py
```

Đợi TurtleBot 4 xuất hiện và mô phỏng chạy. World `gesture_arena` đặt robot ở giữa sân, dock cách xa vùng demo. Không khởi chạy thêm một simulation hoặc chương trình khác đang phát `/cmd_vel`.

### 5.2. Kiểm tra webcam – Terminal 2

```bash
ros2 run tb4_gesture_control webcam_check --camera-index 0
```

Đóng cửa sổ kiểm tra trước khi chạy nhận diện để giải phóng webcam. Nếu webcam dùng thiết bị khác, thay `0` bằng index tương ứng.

### 5.3. Chạy nhận diện và điều khiển – Terminal 2

```bash
ros2 launch tb4_gesture_control gesture_control.launch.py camera_index:=0
```

Launch này mở hai node riêng: `gesture_recognition_node` đọc webcam và publish cử chỉ; `robot_behavior_node` nhận cử chỉ, đọc cảm biến và phát lệnh vận tốc. Giao diện hiển thị ảnh webcam, nhãn, thông tin độ tin cậy và trạng thái robot.

Để chỉ kiểm tra nhận diện, không phát lệnh chuyển động, dùng lệnh sau **thay cho** lệnh trên:

```bash
ros2 launch tb4_gesture_control gesture_control.launch.py behavior:=false camera_index:=0
```

Ở chế độ chỉ nhận diện, chưa có trạng thái từ node hành vi là bình thường. Dừng phiên này trước khi chạy chế độ điều khiển; chỉ mở một chương trình sử dụng webcam tại một thời điểm.

### 5.4. Thao tác điều khiển

| Cử chỉ | Nhãn trên `/detected_gesture` | Tác dụng |
| --- | --- | --- |
| Giơ một ngón trỏ | `Pointing_Up` | Chọn tiến–lùi 4 chu kỳ, tổng 8 chặng |
| Giơ hai ngón | `Victory` | Chọn đi một vòng tròn |
| Vẫy tay | `Wave` | Chọn chào bằng cách quay trái–phải |
| Giơ ngón cái | `Thumb_Up` | Xác nhận lựa chọn hoặc tiếp tục sau tạm dừng |
| Xòe bàn tay | `Open_Palm` | Gửi lệnh dừng, giữ tiến độ nếu đang thực hiện |
| Nắm tay | `Closed_Fist` | Dừng, hủy kế hoạch và trở về trạng thái chờ |

1. Chọn hành vi bằng một ngón, hai ngón hoặc vẫy tay. Giao diện hiện `Awaiting confirmation`; robot chưa chạy.
2. Giơ ngón cái để xác nhận. Khi cảm biến hợp lệ, giao diện chuyển sang `Executing behavior`.
3. Giữ một cử chỉ hợp lệ trong khung hình khi robot chạy; giữ ngón cái là cách thao tác đơn giản nhất.
4. Xòe tay để tạm dừng. Khi hiện `Paused – progress saved`, giơ ngón cái để tiếp tục phần còn lại.
5. Nắm tay để hủy hoàn toàn. Sau khi hủy, muốn chạy lại cần chọn và xác nhận từ đầu. Muốn đổi hành vi đang tạm dừng cũng cần hủy trước.
6. Sau khi hoàn thành, giữ nguyên ngón cái không làm robot tự chạy thêm một lượt.

Wave cũng cần xác nhận. Để chọn Wave, vẫy ngang hoặc lắc cổ tay qua lại khi robot đang chờ, đến khi thấy nhãn `Wave`. Khi đang chạy hoặc tạm dừng, bàn tay mở được ưu tiên làm lệnh dừng.

Không dùng teleop hoặc lệnh publish vận tốc để điều khiển trong demo. Khi đang chạy, nếu mất cử chỉ hợp lệ quá 0,80 giây, robot nhận lệnh dừng và kế hoạch bị hủy. Riêng trạng thái tạm dừng cho phép bỏ tay khỏi webcam mà vẫn giữ tiến độ. Để kết thúc chương trình, dừng robot bằng cử chỉ rồi nhấn `Ctrl+C` trong các terminal chạy launch.

### 5.5. Quan sát topic – Terminal 3, tùy chọn

Chạy từng lệnh để xem dữ liệu; nhấn `Ctrl+C` trước khi chuyển sang lệnh khác:

```bash
ros2 topic echo /detected_gesture
ros2 topic echo /robot_behavior/status
ros2 topic echo /cmd_vel
ros2 topic hz /odom
ros2 topic hz /scan
```

Các lệnh này chỉ quan sát, không điều khiển chuyển động.

## 6. Thay đổi cấu hình

Tham số nằm trong `config/gestures.yaml`. Các giá trị mặc định chính:

| Tham số | Giá trị | Ý nghĩa |
| --- | --- | --- |
| `confidence_threshold` | `0.65` | Ngưỡng điểm cử chỉ tĩnh |
| `stable_seconds` | `0.35` s | Thời gian giữ ổn định cử chỉ chọn/xác nhận |
| `linear_speed` | `0.30` m/s | Tốc độ tiến–lùi và đi tròn |
| `angular_speed` | `1.0` rad/s | Tốc độ quay chào |
| `travel_distance` | `0.30` m | Quãng đường mỗi chặng tiến hoặc lùi |
| `cycles` | `4` | Số chu kỳ tiến–lùi |
| `circle_radius` | `0.60` m | Bán kính đường tròn; tốc độ quay tương ứng `0.50` rad/s |
| `signal_timeout` | `0.80` s | Ngưỡng mất tín hiệu khi đang chạy/chờ xác nhận |
| `obstacle_distance` | `0.50` m | Khoảng trống LiDAR tối thiểu |

Node giới hạn tốc độ tịnh tiến ở 0,30 m/s và tốc độ quay ở 1,0 rad/s. Robot cần odometry và LiDAR hợp lệ; cảm biến quá hạn 0,50 giây sẽ chặn chuyển động. Kiểm tra LiDAR là cơ chế dừng, chưa phải chức năng tự đi vòng qua vật cản. Thực hiện trong vùng trống, xa dock và tường.

Sau khi sửa launch, config hoặc thông tin đóng gói, build lại rồi source workspace theo mục 3 và khởi động lại launch. Có thể giữ một bản YAML riêng và truyền trực tiếp:

```bash
ros2 launch tb4_gesture_control gesture_control.launch.py \
  params_file:=/duong/dan/gestures.yaml camera_index:=0
```

Dùng model tại vị trí khác:

```bash
ros2 run tb4_gesture_control download_gesture_model --output /duong/dan/gesture_recognizer.task
ros2 launch tb4_gesture_control gesture_control.launch.py \
  model_path:=/duong/dan/gesture_recognizer.task
```

Thay `/duong/dan/...` bằng đường dẫn thực tế trên máy.

## 7. Lỗi thường gặp

| Hiện tượng | Cách kiểm tra |
| --- | --- |
| Không tìm thấy `tb4_gesture_control` | Kiểm tra package nằm trong `src`, build thành công và đã source `~/ros2_ws/install/setup.bash` trong terminal hiện tại. |
| Thiếu `cv2`, `mediapipe` hoặc `numpy` | Cài lại từ `requirements.txt` bằng đúng Python dùng để chạy ROS. |
| Không mở được webcam | Dùng `ls /dev/video*`, thử `camera_index:=1`, kiểm tra quyền truy cập và đóng các ứng dụng khác đang dùng camera. Nếu chạy máy ảo, cần chuyển webcam vào máy ảo. |
| Không tìm thấy model | Chạy `download_gesture_model`; nếu dùng vị trí riêng, truyền `model_path`. |
| Có nhận diện nhưng robot không chạy | Kiểm tra `behavior:=true`, bước xác nhận, trạng thái trên giao diện, dữ liệu `/odom`, `/scan` và domain giữa các terminal. |
| Robot dừng với `Control signal timeout` | Giữ một tay có cử chỉ hợp lệ trong tầm nhìn khi chạy. Sau khi bị hủy do mất tín hiệu, chọn và xác nhận lại. |
| Wave khó nhận | Đưa toàn bộ bàn tay vào ảnh, vẫy qua lại đủ biên độ với ít nhất hai lần đổi hướng. Chọn Wave khi robot đang chờ. |
| Gazebo có robot nhưng chưa điều khiển được | Xem log mô phỏng và trạng thái controller bằng `ros2 control list_controllers -c /controller_manager`; kiểm tra `joint_state_broadcaster` và `diffdrive_controller` ở trạng thái `active`. |

Điểm hiển thị với Wave là điểm của nhãn tĩnh nền từ MediaPipe, không phải xác suất riêng của bộ phát hiện vẫy tay. Cử chỉ Wave được xác định thêm từ lịch sử chuyển động bàn tay.
