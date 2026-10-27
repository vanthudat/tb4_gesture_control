"""Cấu hình cài package ament_python khi chạy colcon build.

setup.py phải ở gốc package. File này đăng ký module, tài nguyên và lệnh ROS;
không trực tiếp chạy webcam, nhận diện hoặc điều khiển robot.
"""

from glob import glob
from setuptools import setup

# Tên phải khớp package.xml, thư mục Python và file đánh dấu trong resource/.
NAME = 'tb4_gesture_control'
SHARE = f'share/{NAME}'

setup(
    name=NAME,
    version='0.1.0',
    # Cài các module Python của thư mục tb4_gesture_control/.
    packages=[NAME],
    # Các file không phải module Python được cài vào share để ros2 launch tìm thấy.
    data_files=[
        # File marker có thể rỗng: tên file đăng ký package vào ament index.
        ('share/ament_index/resource_index/packages', [f'resource/{NAME}']),
        (SHARE, ['package.xml', 'README.md']),
        (f'{SHARE}/launch', glob('launch/*.launch.py')),
        (f'{SHARE}/config', glob('config/*.yaml')),
        (f'{SHARE}/worlds', glob('worlds/*.sdf')),
    ],
    # Thư viện AI được quản lý trong requirements.txt; phụ thuộc ROS trong package.xml.
    install_requires=['setuptools'],
    license='Apache-2.0',
    # Mỗi dòng ánh xạ tên lệnh ros2 run tới module:hàm khởi động main.
    # Ví dụ: ros2 run tb4_gesture_control webcam_check sẽ gọi camera.main().
    entry_points={'console_scripts': [
        f'gesture_recognition_node = {NAME}.gesture_recognition_node:main',
        f'robot_behavior_node = {NAME}.robot_behavior_node:main',
        f'webcam_check = {NAME}.camera:main',
        f'download_gesture_model = {NAME}.model:main',
    ]},
)
