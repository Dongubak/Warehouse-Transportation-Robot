# ~/ws_diff_drive/src/diff_drive_fusion/launch/lidar_slam.launch.py
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # LiDAR 인자 (네가 쓰던 그대로)
    channel_type      = LaunchConfiguration('channel_type', default='serial')
    serial_port       = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate   = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id          = LaunchConfiguration('frame_id', default='laser')
    inverted          = LaunchConfiguration('inverted', default='false')
    angle_compensate  = LaunchConfiguration('angle_compensate', default='true')
    scan_mode         = LaunchConfiguration('scan_mode', default='Standard')

    # 고정 TF 인자(원하면 런치 실행 시 바꿀 수 있게)
    lx = LaunchConfiguration('laser_x', default='0.0')
    ly = LaunchConfiguration('laser_y', default='0.0')
    lz = LaunchConfiguration('laser_z', default='0.30')
    lroll  = LaunchConfiguration('laser_roll',  default='0')
    lpitch = LaunchConfiguration('laser_pitch', default='0')
    lyaw   = LaunchConfiguration('laser_yaw',   default='0')

    cfg_path = os.path.join(
        get_package_share_directory('diff_drive_fusion'),
        'config', 'slam_toolbox.yaml'
    )

    return LaunchDescription([
        # LiDAR args
        DeclareLaunchArgument('channel_type',     default_value=channel_type),
        DeclareLaunchArgument('serial_port',      default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate',  default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id',         default_value=frame_id),
        DeclareLaunchArgument('inverted',         default_value=inverted),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate),
        DeclareLaunchArgument('scan_mode',        default_value=scan_mode),

        # TF args
        DeclareLaunchArgument('laser_x',    default_value=lx),
        DeclareLaunchArgument('laser_y',    default_value=ly),
        DeclareLaunchArgument('laser_z',    default_value=lz),
        DeclareLaunchArgument('laser_roll', default_value=lroll),
        DeclareLaunchArgument('laser_pitch',default_value=lpitch),
        DeclareLaunchArgument('laser_yaw',  default_value=lyaw),

        # 1) LiDAR
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode,
            }],
            output='screen',
        ),

        # 2) 고정 TF(base_link→laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[lx, ly, lz, lroll, lpitch, lyaw, 'base_link', frame_id],
            output='screen',
        ),

        # 3) 오도메 융합 노드
        Node(
            package='diff_drive_fusion',
            executable='fusion_node',
            name='diff_drive_fusion',
            parameters=[{
                'wheel_base': 0.28,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
            output='screen',
        ),

        # 4) slam_toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[cfg_path],
            output='screen',
        ),
    ])