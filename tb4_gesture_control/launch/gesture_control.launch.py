"""Khởi chạy Phần 2/3: nhận diện và hành vi là hai process riêng.

behavior:=false chỉ chạy nhận diện. Khi behavior:=true, node hành vi vẫn có
thể dừng robot bằng watchdog nếu process webcam/suy luận bị kẹt hoặc kết thúc.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Khai báo đối số dòng lệnh và danh sách node do ros2 launch khởi chạy."""
    share = get_package_share_directory('tb4_gesture_control')
    return LaunchDescription([
        # Các đối số truyền sau ros2 launch, ví dụ behavior:=false camera_index:=1.
        DeclareLaunchArgument('params_file', default_value=share + '/config/gestures.yaml'),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('behavior', default_value='true',
                              description='false: perception only; publishes no velocity'),
        # Process thứ nhất sở hữu webcam và publish nhãn.
        Node(
            package='tb4_gesture_control', executable='gesture_recognition_node',
            name='gesture_recognition_node', output='screen',
            # Đọc YAML trước; các giá trị ở dict sau ghi đè giá trị cùng tên trong YAML.
            # ParameterValue ép kiểu rõ ràng từ chuỗi đối số launch sang int/bool/str.
            parameters=[LaunchConfiguration('params_file'), {
                'camera_index': ParameterValue(LaunchConfiguration('camera_index'), value_type=int),
                'model_path': ParameterValue(LaunchConfiguration('model_path'), value_type=str),
                'show_window': ParameterValue(LaunchConfiguration('show_window'), value_type=bool),
            }]),
        # Process thứ hai đọc nhãn và điều khiển robot; bỏ qua khi behavior:=false.
        Node(
            package='tb4_gesture_control', executable='robot_behavior_node',
            name='robot_behavior_node', output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('behavior'))),
    ])
