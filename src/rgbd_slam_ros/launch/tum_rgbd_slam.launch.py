"""启动 TUM/Bonn 数据发布、RTAB-Map RGB-D 里程计与建图."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


IMAGE_REMAPS = [
    ('rgb/image', '/camera/color/image_raw'),
    ('depth/image', '/camera/depth/image_raw'),
    ('rgb/camera_info', '/camera/color/camera_info'),
    ('odom', '/odom'),
]


def generate_launch_description():
    """构造 map→odom→camera_optical_frame 的单次播放流程."""
    dataset = LaunchConfiguration('dataset')
    database_path = LaunchConfiguration('database_path')
    hz = LaunchConfiguration('hz')
    max_frames = LaunchConfiguration('max_frames')
    camera_profile = LaunchConfiguration('camera_profile')
    stamp_map_path = LaunchConfiguration('stamp_map_path')
    recent_frame = LaunchConfiguration('always_process_most_recent_frame')

    odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        parameters=[{
            'frame_id': 'camera_optical_frame',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': False,
            # Bonn 实验设为 false，避免日志中证实的调度抖动主动丢帧。
            'always_process_most_recent_frame': ParameterValue(
                recent_frame, value_type=bool),
            # Fast DDS 的 system default 订阅可能是 Best Effort，会丢大图像。
            'qos': 1,  # Reliable：与发布节点默认发布 QoS 一致。
            'qos_camera_info': 1,
        }],
        remappings=IMAGE_REMAPS,
    )
    slam = Node(
        package='rtabmap_slam', executable='rtabmap', namespace='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'camera_optical_frame',
            'map_frame_id': 'map',
            'publish_tf': True,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': False,
            'approx_sync': False,
            'qos': 1,
            'qos_camera_info': 1,
            'database_path': database_path,
        }],
        remappings=IMAGE_REMAPS,
    )
    publisher = Node(
        package='rgbd_slam_ros', executable='tum_rgbd_publisher', output='screen',
        arguments=['--dataset', dataset, '--hz', hz,
                   '--max-frames', max_frames,
                   '--camera-profile', camera_profile,
                   '--stamp-map', stamp_map_path],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dataset', description='TUM 或 Bonn 数据集目录'),
        DeclareLaunchArgument('database_path', description='新的 RTAB-Map 数据库路径'),
        DeclareLaunchArgument('hz', default_value='5'),
        DeclareLaunchArgument('max_frames', default_value='0'),
        DeclareLaunchArgument('camera_profile', default_value='tum'),
        DeclareLaunchArgument('stamp_map_path', default_value=''),
        DeclareLaunchArgument('always_process_most_recent_frame',
                              default_value='true'),
        odom,
        slam,
        # 先启动订阅者，再开始播放，减少首帧初始化期间的丢失。
        TimerAction(period=5.0, actions=[publisher]),
        RegisterEventHandler(OnProcessExit(
            target_action=publisher,
            on_exit=[TimerAction(period=3.0, actions=[
                EmitEvent(event=Shutdown(reason='数据集播放完成')),
            ])],
        )),
    ])
