"""Khởi chạy sân trống Gazebo Fortress, TurtleBot 4 và các cầu nối ROS.

Robot ở (0,0); dock ở (-4,-4) để không cản các chặng tiến/lùi tại vùng giữa sân.
Tái sử dụng description, bridge và node Create 3 của các package đã cài.
Không khởi chạy Nav2, teleop hoặc node nhận diện trong launch này.
"""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, file, **arguments):
    """Tìm file launch trong share của package và chuyển tiếp các đối số cần thiết."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', file)),
        launch_arguments=arguments.items())


def generate_launch_description():
    """Ghép tài nguyên Gazebo, world, robot, dock, clock, bridges và node nền."""
    package = share('tb4_gesture_control')
    tb4 = share('turtlebot4_ignition_bringup')
    model = LaunchConfiguration('model')
    # Gazebo cần tìm mesh theo package URI; thêm thư mục cha chứa các share package.
    paths = [os.path.dirname(share(name)) for name in (
        'turtlebot4_description', 'irobot_create_description')]
    resource_path = os.pathsep.join(paths + [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    # Đường dẫn plugin giao diện TurtleBot/Create 3; giữ thêm đường dẫn đã có của người dùng.
    gui_plugins = os.pathsep.join([
        share('turtlebot4_ignition_gui_plugins') + '/lib',
        share('irobot_create_ignition_plugins') + '/lib',
        os.environ.get('IGN_GUI_PLUGIN_PATH', ''),
    ])
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
        DeclareLaunchArgument('headless', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='gesture_arena', choices=['gesture_arena']),
        DeclareLaunchArgument('gazebo', default_value='ignition'),
        # Namespace rỗng để dùng các topic /cmd_vel, /odom, /scan như đề bài.
        DeclareLaunchArgument('namespace', default_value=''),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', gui_plugins),
        # Nạp world riêng và chạy vật lý (-r). headless:=true thêm -s để chỉ chạy server.
        include('ros_gz_sim', 'gz_sim.launch.py',
                gz_args=[package + '/worlds/gesture_arena.sdf -r -v 2',
                         PythonExpression(["' -s' if '", LaunchConfiguration('headless'),
                                           "' == 'true' else ''"]),
                         ' --gui-config ', tb4 + '/gui/', model, '/gui.config'],
                on_exit_shutdown='true'),
        # Cầu nối /clock một chiều từ Gazebo sang ROS cho các node dùng thời gian mô phỏng.
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        # Nạp URDF robot, joint states và TF; các node mô phỏng dùng use_sim_time=true.
        include('turtlebot4_description', 'robot_description.launch.py',
                model=model, use_sim_time='true', namespace=''),
        # xacro tạo URDF dock; ép thành chuỗi để nội dung XML không bị hiểu thành YAML.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='dock_state_publisher', parameters=[{
                 'use_sim_time': True,
                 'robot_description': ParameterValue(Command([
                     'xacro ', share('irobot_create_description'),
                     '/urdf/dock/standard_dock.urdf.xacro gazebo:=ignition']), value_type=str),
             }], remappings=[('robot_description', 'standard_dock_description')]),
        # TF dock phải khớp vị trí dock được spawn ở (-4,-4), không dùng pose dock mặc định.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='gesture_dock_tf',
             arguments=['-4', '-4', '0', '0', '0', '0', 'odom', 'std_dock_link']),
        # Spawn robot giữa sân từ topic robot_description, dock từ standard_dock_description.
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'gesture_arena', '-name', 'turtlebot4',
                        '-x', '0', '-y', '0', '-z', '0', '-topic', 'robot_description']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'gesture_arena', '-name', 'standard_dock',
                        '-x', '-4', '-y', '-4', '-z', '0', '-topic', 'standard_dock_description']),
        # Dùng bridge có sẵn để nối vận tốc/cảm biến giữa ROS 2 và Gazebo.
        # world/robot_name/dock_name phải khớp tên đối tượng ở trên.
        include('turtlebot4_ignition_bringup', 'ros_ign_bridge.launch.py',
                model=model, robot_name='turtlebot4', dock_name='standard_dock',
                namespace='', world='gesture_arena'),
        include('turtlebot4_ignition_bringup', 'turtlebot4_nodes.launch.py', model=model,
                param_file=tb4 + '/config/turtlebot4_node.yaml'),
        # Các node nền Create 3 bao gồm controller bánh xe và xử lý trạng thái/cảm biến.
        include('irobot_create_common_bringup', 'create3_nodes.launch.py',
                namespace='', gazebo='ignition'),
        include('irobot_create_ignition_bringup', 'create3_ignition_nodes.launch.py',
                robot_name='turtlebot4', dock_name='standard_dock'),
        # Nối tên frame LiDAR từ Gazebo với rplidar_link của mô hình ROS.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='gesture_lidar_tf',
             arguments=['0', '0', '0', '0', '0', '0', 'rplidar_link',
                        'turtlebot4/rplidar_link/rplidar']),
    ])
