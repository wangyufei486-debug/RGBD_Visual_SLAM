"""用现有 RTAB-Map 数据库评估 freiburg1_room 的回环和位姿图优化。"""

import csv
from functools import lru_cache
from pathlib import Path
import re
import sqlite3

import cv2
import matplotlib.pyplot as plt
import numpy as np

from evaluate_trajectory import (
    match_poses, quaternion_to_rotation, read_groundtruth, rigid_align,
)

ROOT = Path(__file__).resolve().parent
DATASET = ROOT / 'data/rgbd_dataset_freiburg1_room'
DB = ROOT / 'outputs/rtabmap_room_verified.db'
LOG = ROOT / 'outputs/rtabmap_room_verified.log'
LOOP_LOG = ROOT / 'outputs/rtabmap_room_loop_ids.log'
OUTPUT = ROOT / 'outputs/rtabmap_room_eval'


def rgb_index():
    """读取 room 的原始 RGB 时间戳与文件路径。"""
    records = []
    for line in (DATASET / 'rgb.txt').read_text().splitlines():
        if line and not line.startswith('#'):
            stamp, path = line.split()[:2]
            records.append((float(stamp), DATASET / path))
    return records


def publisher_anchors():
    """从旧发布日志提取每 20 帧的 ROS 时间锚点。"""
    pattern = r'\[tum_rgbd_publisher-\d+\] \[INFO\] \[(\d+\.\d+)\].*?发布第 (\d+) 帧'
    anchors = [(int(i), float(t)) for t, i in re.findall(pattern, LOG.read_text())]
    if not anchors:
        raise ValueError('发布日志没有帧号与 ROS 时间戳锚点')
    return anchors


def match_database_images(connection, rgb):
    """用数据库 JPEG 与原始 PNG 核对帧号；日志时间只限定搜索窗口。"""
    anchors = publisher_anchors()
    thumbnails = []
    for _, path in rgb:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'无法读取 {path}')
        thumbnails.append(cv2.resize(image, (96, 72), interpolation=cv2.INTER_AREA))

    @lru_cache(maxsize=40)
    def original(index):
        return cv2.imread(str(rgb[index][1]), cv2.IMREAD_COLOR)

    mapping = {}
    query = 'SELECT n.id, n.stamp, d.image FROM Node n JOIN Data d ON n.id=d.id ORDER BY n.id'
    for node_id, ros_stamp, blob in connection.execute(query):
        image = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'数据库节点 {node_id} 图像无法解码')
        anchor_id, anchor_stamp = min(anchors, key=lambda x: abs(x[1] - ros_stamp))
        guess = anchor_id + round((ros_stamp - anchor_stamp) * 5)
        thumb = cv2.resize(image, (96, 72), interpolation=cv2.INTER_AREA)
        candidates = range(max(0, guess - 12), min(len(rgb), guess + 13))
        rough = sorted((cv2.absdiff(thumb, thumbnails[i]).mean(), i)
                       for i in candidates)[:4]
        precise = sorted((cv2.absdiff(image, original(i)).mean(), i)
                         for _, i in rough)
        best_error, frame = precise[0]
        second_error = precise[1][0]
        # 同一帧的 JPEG 误差约 2–4；相邻帧明显更大。模糊匹配必须报错。
        if best_error >= 5 or second_error - best_error <= 3:
            raise ValueError(f'节点 {node_id} 的原始帧匹配不可靠：{precise}')
        mapping[node_id] = (ros_stamp, frame, rgb[frame][0], best_error, second_error)
    if len({x[1] for x in mapping.values()}) != len(mapping):
        raise ValueError('多个数据库节点匹配到同一原始 RGB 帧')
    return mapping


def read_export(path):
    """读取 RTAB-Map 格式 11 的相机位姿，以节点 ID 为键。"""
    data = np.loadtxt(path)
    return {int(row[8]): row for row in data}


def pose_matrix(row):
    """把导出行中的平移和四元数还原为 4×4 相机位姿。"""
    pose = np.eye(4)
    pose[:3, :3] = quaternion_to_rotation(*row[4:8])
    pose[:3, 3] = row[1:4]
    return pose


def aligned_error(poses, groundtruth_xyz):
    """按相同真值采样对齐轨迹，返回对齐位置、逐节点误差和 ATE。"""
    xyz = poses[:, :3, 3]
    rotation, translation = rigid_align(xyz, groundtruth_xyz)
    aligned = (rotation @ xyz.T).T + translation
    errors = np.linalg.norm(aligned - groundtruth_xyz, axis=1)
    return aligned, errors, float(np.sqrt(np.mean(errors ** 2)))


def make_plot(gt_xyz, odom_xyz, optimized_xyz, before, after):
    """绘制真值、原始里程计和优化后轨迹的 XY/XZ 对比图。"""
    fig, (ax_xy, ax_xz) = plt.subplots(1, 2, figsize=(12, 5),
                                       constrained_layout=True)
    tracks = [('TUM Groundtruth', gt_xyz, 'black'),
              ('Odometry, SE(3) aligned', odom_xyz, '#d95f02'),
              ('Optimized, SE(3) aligned', optimized_xyz, '#1b9e77')]
    for label, xyz, color in tracks:
        ax_xy.plot(xyz[:, 0], xyz[:, 1], label=label, color=color, linewidth=1.5)
        ax_xz.plot(xyz[:, 0], xyz[:, 2], label=label, color=color, linewidth=1.5)
    ax_xy.scatter(*gt_xyz[0, :2], color='blue', s=35, zorder=4)
    ax_xz.scatter(gt_xyz[0, 0], gt_xyz[0, 2], color='blue', s=35, zorder=4)
    ax_xy.set(xlabel='X (m)', ylabel='Y (m)', title='Top view (XY)')
    ax_xz.set(xlabel='X (m)', ylabel='Z (m)', title='Side view (XZ)')
    ax_xy.set_aspect('equal', adjustable='box')
    for ax in (ax_xy, ax_xz):
        ax.grid(alpha=.25)
    ax_xy.legend(loc='best', fontsize=8)
    fig.suptitle(f'freiburg1_room ATE RMSE: {before * 100:.2f} cm -> {after * 100:.2f} cm')
    fig.savefig(OUTPUT / 'trajectory_comparison.png', dpi=180)
    plt.close(fig)


def make_loop_examples(connection, pairs):
    """从数据库读取代表性历史/当前节点 RGB 图像进行并排展示。"""
    fig, axes = plt.subplots(len(pairs), 2, figsize=(11, 3.3 * len(pairs)))
    for row, (history, current) in enumerate(pairs):
        for col, node_id in enumerate((history, current)):
            blob = connection.execute('SELECT image FROM Data WHERE id=?',
                                      (node_id,)).fetchone()[0]
            image = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
            axes[row, col].imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            axes[row, col].set_title(f'{"History" if col == 0 else "Current"} node {node_id}')
            axes[row, col].axis('off')
    fig.tight_layout()
    fig.savefig(OUTPUT / 'loop_examples.png', dpi=140)
    plt.close(fig)


def main():
    """验证时间映射与约束类型，并评估同一批图节点的优化前后 ATE。"""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(f'file:{DB}?mode=ro', uri=True)
    rgb = rgb_index()
    mapping = match_database_images(connection, rgb)

    with (OUTPUT / 'node_timestamps.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['node_id', 'ros_stamp', 'rgb_frame_index', 'tum_stamp',
                         'image_mae', 'second_best_mae'])
        for node_id, (ros, frame, tum, best, second) in sorted(mapping.items()):
            writer.writerow([node_id, f'{ros:.6f}', frame, f'{tum:.6f}',
                             f'{best:.3f}', f'{second:.3f}'])

    raw = read_export(OUTPUT / 'room_odom_camera_poses.txt')
    optimized = read_export(OUTPUT / 'room_optimized_camera_poses.txt')
    if raw.keys() != optimized.keys():
        raise ValueError('优化前后的节点 ID 不一致')
    node_ids = sorted(raw)
    times = np.array([mapping[node][2] for node in node_ids])
    for node in node_ids:
        if abs(raw[node][0] - optimized[node][0]) > 1e-6:
            raise ValueError(f'节点 {node} 的导出时间戳不一致')
        if abs(raw[node][0] - mapping[node][0]) > 2e-6:
            raise ValueError(f'节点 {node} 的数据库时间戳不一致')
    raw_poses = np.stack([pose_matrix(raw[node]) for node in node_ids])
    optimized_poses = np.stack([pose_matrix(optimized[node]) for node in node_ids])
    gt = read_groundtruth(DATASET / 'groundtruth.txt')
    matched_raw, matched_gt, matched_times, deltas = match_poses(times, raw_poses, gt)
    selected = np.searchsorted(times, matched_times)
    if not np.array_equal(times[selected], matched_times):
        raise ValueError('Groundtruth 匹配后的节点索引不一致')
    matched_opt = optimized_poses[selected]
    gt_xyz = matched_gt[:, :3, 3]
    before_xyz, before_errors, before = aligned_error(matched_raw, gt_xyz)
    after_xyz, after_errors, after = aligned_error(matched_opt, gt_xyz)
    improvement = 100 * (before - after) / before
    make_plot(gt_xyz, before_xyz, after_xyz, before, after)

    with (OUTPUT / 'ate_per_node.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['node_id', 'tum_stamp', 'groundtruth_delta_ms',
                         'ate_odom_m', 'ate_optimized_m'])
        for i, j in enumerate(selected):
            writer.writerow([node_ids[j], f'{times[j]:.6f}', f'{deltas[i] * 1000:.3f}',
                             f'{before_errors[i]:.6f}', f'{after_errors[i]:.6f}'])

    link_counts = dict(connection.execute('SELECT type, COUNT(*) FROM Link GROUP BY type'))
    loops = sorted({tuple(sorted((a, b))) for a, b in
                    connection.execute('SELECT from_id, to_id FROM Link WHERE type=1')})
    with (OUTPUT / 'loop_constraints.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['current_node', 'history_node', 'current_rgb_frame',
                         'history_rgb_frame', 'current_tum_stamp', 'history_tum_stamp'])
        for history, current in loops:
            writer.writerow([current, history, mapping[current][1], mapping[history][1],
                             f'{mapping[current][2]:.6f}', f'{mapping[history][2]:.6f}'])
    reports = [int(x) for x in re.findall(r'^(\d+)\s*$', LOOP_LOG.read_text(), re.M)
               if int(x) > 0]
    reports_match = sorted(reports) == sorted(history for history, _ in loops)
    if not reports_match:
        raise ValueError('非零回环报告与数据库全局回环约束不一致')
    make_loop_examples(connection, [(5, 178), (39, 213), (42, 234)])

    loop_rows = '\n'.join(f'| {current} | {history} | {mapping[current][1]} | {mapping[history][1]} |'
                          for history, current in loops)
    report = f"""# freiburg1_room：回环与位姿图优化评估

## 数据与方法

- 现有数据库包含 {len(mapping)} 个节点；`rtabmap-export --poses_camera --opt 3/0 --poses_format 11` 从同一数据库分别导出 {len(raw)} 个图节点。其余 {len(mapping)-len(raw)} 个节点的 `weight=-9`，不在导出的位姿图中。
- 数据库 JPEG 与 TUM 原始 RGB PNG 逐帧比对恢复时间戳。最大同帧像素 MAE 为 {max(x[3] for x in mapping.values()):.2f}，候选次优 MAE 最小为 {min(x[4] for x in mapping.values()):.2f}；没有模糊匹配。
- 两条导出轨迹的节点 ID 相同，ROS 时间戳在导出精度内一致；{len(matched_times)} 个图节点均匹配到 Groundtruth，最大时间差 {max(deltas)*1000:.2f} ms。
- 使用 `--poses_camera` 导出相机位姿。此项目的 `frame_id=camera_optical_frame`，核查 `--poses` 与 `--poses_camera` 的坐标差小于 1 微米。[TUM 官方格式说明](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)指出 Groundtruth 平移为 RGB 相机光心在动捕世界坐标系中的位置。ATE 对两条轨迹分别使用同一个 SVD SE(3) 刚体对齐方法，**不调整尺度**。

## 回环约束

数据库 `Link.type=0` 为相邻里程计约束 {link_counts.get(0, 0)//2} 条，`type=1` 为全局视觉回环约束 {len(loops)} 条，`type=2` 为局部空间回环约束 {link_counts.get(2, 0)//2} 条（类型名来自已安装 RTAB-Map 的 `Link.h`）。SQLite 对每条约束保存两个方向，因此原始行数分别是 {link_counts.get(0, 0)}、{link_counts.get(1, 0)}、{link_counts.get(2, 0)}。

| 当前节点 | 历史节点 | 当前 RGB 帧 | 历史 RGB 帧 |
|---:|---:|---:|---:|
{loop_rows}

`/rtabmap/info` 的非零 `loop_closure_id` 共 {len(reports)} 次；其历史节点 ID 多重集合与数据库 {len(loops)} 条全局回环完全一致（匹配：{reports_match}）。代表性图像见 `loop_examples.png`，可见同一显示器、办公桌和键盘等场景从不同视角被再次观察。RTAB-Map 先以外观词袋检索历史节点，再以几何配准验证回环并写入 `type=1` 约束；位置已接近的节点可另形成 `type=2` 约束。

## ATE 平移 RMSE

| 指标 | 优化前 Odometry | 全局图优化后 |
|---|---:|---:|
| ATE 平移 RMSE | {before:.6f} m ({before*100:.2f} cm) | {after:.6f} m ({after*100:.2f} cm) |

误差降低 **{improvement:.2f}%**。位姿图优化利用回环等图约束修正了累积漂移；轨迹见 `trajectory_comparison.png`。该对比体现完整图优化的效果，不能把全部改善单独归因于 14 条全局视觉回环，因为图中还包含 19 条局部空间回环约束。ATE 只覆盖 174 个导出图节点，不代表全部 1362 对输入图像的误差。剩余误差可能来自 RGB-D 噪声、相机标定近似及特征与约束误差。此处没有用 Groundtruth 代替位姿估计，也没有按节点序号推断时间戳。

## 复现命令（只读取现有数据库）

```bash
cd ~/RGBD_Visual_SLAM
source /opt/ros/jazzy/setup.bash
rtabmap-export --poses_camera --opt 3 --poses_format 11 --output room_odom --output_dir outputs/rtabmap_room_eval outputs/rtabmap_room_verified.db
rtabmap-export --poses_camera --opt 0 --poses_format 11 --output room_optimized --output_dir outputs/rtabmap_room_eval outputs/rtabmap_room_verified.db
python3 evaluate_rtabmap_room.py
```
"""
    (OUTPUT / 'report.md').write_text(report, encoding='utf-8')
    print(f'图节点 {len(raw)}；全局回环 {len(loops)}；非零回环报告 {len(reports)}；一致={reports_match}')
    print(f'ATE RMSE: {before:.6f} m -> {after:.6f} m，降低 {improvement:.2f}%')
    print(f'结果目录：{OUTPUT}')
    connection.close()


if __name__ == '__main__':
    main()
