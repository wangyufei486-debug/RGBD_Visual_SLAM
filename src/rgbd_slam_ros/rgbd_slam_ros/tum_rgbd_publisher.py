"""在 ROS 2 Jazzy 中同步发布 TUM/Bonn RGB-D 数据与相机标定."""

import argparse
from bisect import bisect_left
import csv
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image

# 相对路径按启动时的工作目录解析；Launch 通常传入绝对数据集路径。
DATASET = Path('data/rgbd_dataset_freiburg1_xyz')
BONN_K = np.array([[542.822841, 0., 315.593520],
                   [0., 542.576870, 237.756098],
                   [0., 0., 1.]], dtype=np.float64)
BONN_D = np.array([0.039903, -0.099343, -0.000730,
                   -0.000144, 0.], dtype=np.float64)


def read_index(path):
    """读取 TUM 格式索引，返回时间戳与相对路径的记录列表."""
    records = []
    with path.open(encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            timestamp, relative_path = line.split()[:2]
            records.append((float(timestamp), relative_path))
    return records


def match_rgb_depth(rgb_records, depth_records, max_dt=0.02):
    """按时间戳配对 RGB 与最近的深度帧；返回差值不超过 max_dt 的帧."""
    depth_times = [timestamp for timestamp, _ in depth_records]
    matched = []
    for rgb_time, rgb_path in rgb_records:
        j = bisect_left(depth_times, rgb_time)
        candidates = [k for k in (j - 1, j) if 0 <= k < len(depth_records)]
        if not candidates:
            continue
        k = min(candidates, key=lambda idx: abs(depth_times[idx] - rgb_time))
        if abs(depth_times[k] - rgb_time) <= max_dt:
            matched.append((rgb_time, rgb_path, depth_records[k][1]))
    return matched


class TumRGBDPublisher(Node):
    """用同一 ROS 时间戳发布 RGB、深度、标定，并记录原始时间映射."""

    def __init__(self, dataset, hz=5.0, max_frames=100, loop=False,
                 camera_profile='tum', stamp_map_path=None):
        super().__init__('tum_rgbd_publisher')
        self.dataset = dataset
        self.bridge = CvBridge()
        self.loop = loop
        self.camera_profile = camera_profile
        if camera_profile == 'bonn':
            self.k = BONN_K
            # Bonn 官方给出非零畸变；两幅已配准图像必须一起校正。
            self.rectify_map = cv2.initUndistortRectifyMap(
                BONN_K, BONN_D, None, BONN_K, (640, 480), cv2.CV_32FC1)
        else:
            self.k = np.array([[525., 0., 319.5],
                               [0., 525., 239.5],
                               [0., 0., 1.]], dtype=np.float64)
            self.rectify_map = None
        self.stamp_map_file = None
        if stamp_map_path:
            stamp_map_path.parent.mkdir(parents=True, exist_ok=True)
            self.stamp_map_file = stamp_map_path.open('w', newline='', encoding='utf-8')
            self.stamp_writer = csv.writer(self.stamp_map_file)
            self.stamp_writer.writerow(
                ['frame_index', 'ros_stamp_ns', 'dataset_stamp', 'rgb_path', 'depth_path'])
        self.frames = match_rgb_depth(
            read_index(dataset / 'rgb.txt'), read_index(dataset / 'depth.txt')
        )
        if max_frames > 0:
            self.frames = self.frames[:max_frames]
        if not self.frames:
            raise RuntimeError('没有找到 RGB-Depth 匹配帧，请检查数据集和时间戳')

        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.index = 0
        self.timer = self.create_timer(1.0 / hz, self.publish_frame)
        self.get_logger().info(f'找到 {len(self.frames)} 对 RGB-D 数据，发布频率 {hz:g} Hz')
        self.get_logger().info(f'相机配置：{camera_profile}；'
                               f'Bonn 图像校正：{self.rectify_map is not None}')

    def publish_frame(self):
        if self.index >= len(self.frames):
            if self.loop:
                self.index = 0
                self.get_logger().info('序列播放结束，从第一帧重新发布')
            else:
                self.get_logger().info('已发布全部数据，停止节点')
                self.timer.cancel()
                rclpy.shutdown()
                return

        dataset_time, rgb_path, depth_path = self.frames[self.index]
        rgb = cv2.imread(str(self.dataset / rgb_path), cv2.IMREAD_COLOR)
        depth_raw = cv2.imread(str(self.dataset / depth_path), cv2.IMREAD_UNCHANGED)
        if rgb is None or depth_raw is None:
            raise RuntimeError(f'第 {self.index} 帧图像读取失败：{rgb_path}, {depth_path}')
        if rgb.shape[:2] != (480, 640) or depth_raw.shape[:2] != (480, 640):
            raise ValueError('预期 640x480 RGB-D 图像，请核对数据集')

        if depth_raw.dtype != np.uint16:
            raise ValueError(f'预期 16 位深度 PNG，实际为 {depth_raw.dtype}')
        if self.rectify_map is not None:
            rgb = cv2.remap(rgb, *self.rectify_map, cv2.INTER_LINEAR)
            depth_raw = cv2.remap(depth_raw, *self.rectify_map, cv2.INTER_NEAREST)
        # TUM 格式 PNG 以 1/5000 m 存储；ROS 16UC1 约定以 mm 存储。
        # 保留 0 无效值，同时将深度消息大小从 1.2 MB 降至 0.6 MB。
        depth_mm = np.rint(depth_raw / 5.0).astype(np.uint16)

        # 三条消息使用同一 ROS 时间戳，RTAB-Map 才能精确同步；CSV 保留原始时间。
        stamp = self.get_clock().now().to_msg()
        frame_id = 'camera_optical_frame'
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
        rgb_msg.header.stamp = depth_msg.header.stamp = stamp
        rgb_msg.header.frame_id = depth_msg.header.frame_id = frame_id

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width, info.height = 640, 480
        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]
        info.distortion_model = 'plumb_bob'
        # Bonn 已对 RGB 和深度同时校正，故输出图像的剩余畸变为零。
        info.d = [0.0] * 5
        info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)
        if self.stamp_map_file is not None:
            ros_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            self.stamp_writer.writerow(
                [self.index, ros_ns, f'{dataset_time:.9f}', rgb_path, depth_path])
            self.stamp_map_file.flush()
        if self.index % 20 == 0:
            self.get_logger().info(
                f'发布第 {self.index:04d} 帧 | 原始时间戳 {dataset_time:.6f}'
                f' | 深度 {depth_mm[240, 320] / 1000:.3f} m'
            )
        self.index += 1

    def close_stamp_map(self):
        if self.stamp_map_file is not None:
            self.stamp_map_file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, default=DATASET)
    parser.add_argument('--hz', type=float, default=5.0)
    parser.add_argument('--max-frames', type=int, default=100, help='0 表示全部帧')
    parser.add_argument('--loop', action='store_true', help='循环播放，供 RTAB-Map 调试')
    parser.add_argument('--camera-profile', choices=['tum', 'bonn'], default='tum')
    parser.add_argument('--stamp-map', default='', help='可选 CSV：ROS 与原始时间戳对应表')
    args = parser.parse_args(remove_ros_args()[1:])
    if args.hz <= 0:
        parser.error('--hz 必须大于 0')
    dataset = args.dataset.expanduser().resolve()
    if not (dataset / 'rgb.txt').is_file() or not (dataset / 'depth.txt').is_file():
        parser.error(f'无法找到 RGB-D 索引文件：{dataset}')

    rclpy.init()
    node = None
    try:
        stamp_map = Path(args.stamp_map).expanduser().resolve() if args.stamp_map else None
        node = TumRGBDPublisher(dataset, args.hz, args.max_frames, args.loop,
                                args.camera_profile, stamp_map)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_stamp_map()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
