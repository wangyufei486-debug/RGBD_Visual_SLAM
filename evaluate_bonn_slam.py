"""用发布时保存的时间戳表评估 Bonn RTAB-Map 图节点轨迹。"""

import argparse
from bisect import bisect_left
import csv
import json
from pathlib import Path
import re
import sqlite3
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from evaluate_trajectory import (
    bonn_camera_groundtruth, match_poses, read_groundtruth, rigid_align,
)


def export_poses(database, output):
    """使用本机 RTAB-Map 导出同一数据库的原始与优化位姿。"""
    for name, opt in [('slam_odom', '3'), ('slam_optimized', '0')]:
        subprocess.run([
            'rtabmap-export', '--poses_camera', '--opt', opt,
            '--poses_format', '11', '--output', name,
            '--output_dir', str(output), str(database),
        ], check=True, stdout=subprocess.DEVNULL)
    arrays = [np.atleast_2d(np.loadtxt(output / f'{name}_camera_poses.txt'))
              for name in ('slam_odom', 'slam_optimized')]
    return [{int(row[8]): row for row in array} for array in arrays]


def timestamp_map(path):
    """读取 ROS 纳秒时间与 Bonn 原始时间、帧号的对应表。"""
    with path.open(newline='', encoding='utf-8') as file:
        records = [(int(row['ros_stamp_ns']), float(row['dataset_stamp']),
                    int(row['frame_index']), row['rgb_path'])
                   for row in csv.DictReader(file)]
    return sorted(records)


def read_odom_quality(log_path, stamps, output):
    """由日志处理时间减去 delay 恢复输入时间，再用 CSV 精确定位帧号。"""
    pattern = (r'\[rgbd_odometry-1\] \[INFO\] \[(\d+\.\d+)\]'
               r'.*?Odom: quality=(\d+).*?delay=(\d+\.\d+)s')
    text = log_path.read_text()
    ros_ns = [item[0] for item in stamps]
    rows = []
    for wall_time, quality, delay in re.findall(pattern, text):
        input_ns = round((float(wall_time) - float(delay)) * 1e9)
        j = bisect_left(ros_ns, input_ns)
        candidates = [k for k in (j - 1, j) if 0 <= k < len(stamps)]
        k = min(candidates, key=lambda k: abs(ros_ns[k] - input_ns))
        error_ms = abs(ros_ns[k] - input_ns) / 1e6
        if error_ms > 1.0:
            raise ValueError(f'无法可靠定位 odom 日志到发布帧：{error_ms:.3f} ms')
        rows.append((stamps[k][2], stamps[k][3], stamps[k][1],
                     int(quality), error_ms))
    if len({row[0] for row in rows}) != len(rows):
        raise ValueError('多个 odom 质量日志对应到同一发布帧')
    with (output / 'odom_quality.csv').open(
            'w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['publisher_frame_index', 'rgb_path', 'dataset_stamp',
                         'quality', 'log_stamp_delta_ms'])
        writer.writerows(rows)
    return rows, text


def aligned_ate(xyz, gt_xyz):
    """对估计位置做无尺度 SE(3) 对齐，返回轨迹、逐帧误差及 RMSE。"""
    rotation, translation = rigid_align(xyz, gt_xyz)
    aligned = (rotation @ xyz.T).T + translation
    error = np.linalg.norm(aligned - gt_xyz, axis=1)
    return aligned, error, float(np.sqrt(np.mean(error ** 2)))


def main():
    """导出同一数据库的两条图轨迹，并按发布时保存的时间映射评估。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--database', type=Path, required=True)
    parser.add_argument('--stamp-map', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    dataset = args.dataset.expanduser().resolve()
    database = args.database.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    raw, optimized = export_poses(database, output)
    if raw.keys() != optimized.keys():
        raise ValueError('原始与优化后图节点集合不一致')
    stamps = timestamp_map(args.stamp_map.expanduser().resolve())
    quality_rows, log_text = read_odom_quality(
        output / 'rtabmap.log', stamps, output)
    ros_ns = [item[0] for item in stamps]
    connection = sqlite3.connect(f'file:{database}?mode=ro', uri=True)
    db_stamps = dict(connection.execute('SELECT id, stamp FROM Node'))
    node_ids = sorted(raw)
    dataset_times = []
    map_deltas = []
    for node_id in node_ids:
        db_ns = round(db_stamps[node_id] * 1_000_000_000)
        j = bisect_left(ros_ns, db_ns)
        candidates = [k for k in (j - 1, j) if 0 <= k < len(stamps)]
        k = min(candidates, key=lambda k: abs(ros_ns[k] - db_ns))
        delta_ms = abs(ros_ns[k] - db_ns) / 1e6
        if delta_ms > 1.0:
            raise ValueError(f'节点 {node_id} 未找到可靠的 ROS→Bonn 时间映射')
        if abs(raw[node_id][0] - optimized[node_id][0]) > 1e-6:
            raise ValueError(f'节点 {node_id} 的两条轨迹时间不一致')
        dataset_times.append(stamps[k][1])
        map_deltas.append(delta_ms)
    if np.any(np.diff(dataset_times) <= 0):
        raise ValueError('图节点的 Bonn 原始时间戳不是严格递增的')

    # Bonn 官方 T_m 为动捕标记到传感器的变换；评估相机光心。
    groundtruth = bonn_camera_groundtruth(
        read_groundtruth(dataset / 'groundtruth.txt'))
    raw_xyz = np.array([raw[i][1:4] for i in node_ids])
    opt_xyz = np.array([optimized[i][1:4] for i in node_ids])
    raw_poses = np.repeat(np.eye(4)[None], len(node_ids), axis=0)
    raw_poses[:, :3, 3] = raw_xyz
    matched_raw, matched_gt, matched_times, gt_deltas = match_poses(
        np.array(dataset_times), raw_poses, groundtruth)
    selected = np.searchsorted(dataset_times, matched_times)
    if not np.array_equal(np.array(dataset_times)[selected], matched_times):
        raise ValueError('Groundtruth 匹配后的节点索引不一致')
    gt_xyz = matched_gt[:, :3, 3]
    before_xyz, before_errors, before = aligned_ate(
        matched_raw[:, :3, 3], gt_xyz)
    after_xyz, after_errors, after = aligned_ate(opt_xyz[selected], gt_xyz)

    with (output / 'slam_ate_per_node.csv').open(
            'w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        writer.writerow(['node_id', 'bonn_stamp', 'gt_delta_ms',
                         'odom_ate_m', 'optimized_ate_m'])
        for row, index in enumerate(selected):
            writer.writerow([node_ids[index], f'{dataset_times[index]:.6f}',
                             f'{gt_deltas[row]*1000:.3f}',
                             f'{before_errors[row]:.6f}',
                             f'{after_errors[row]:.6f}'])

    full_gt = np.array([pose[:3, 3] for stamp, pose in groundtruth
                        if dataset_times[0] <= stamp <= dataset_times[-1]])
    fig, (xy, xz) = plt.subplots(1, 2, figsize=(12, 5),
                                 constrained_layout=True)
    tracks = [('Groundtruth (full)', full_gt, '0.6', '--'),
              ('Groundtruth (matched)', gt_xyz, 'black', '-'),
              ('Raw RTAB-Map odometry', before_xyz, '#d95f02', '-'),
              ('Optimized RTAB-Map', after_xyz, '#1b9e77', '-')]
    for label, xyz, color, style in tracks:
        xy.plot(xyz[:, 0], xyz[:, 1], style, label=label, color=color)
        xz.plot(xyz[:, 0], xyz[:, 2], style, label=label, color=color)
    xy.set(xlabel='X (m)', ylabel='Y (m)', title='Top view')
    xz.set(xlabel='X (m)', ylabel='Z (m)', title='Side view')
    xy.set_aspect('equal', adjustable='box')
    for ax in (xy, xz):
        ax.grid(alpha=.25)
    xy.legend(fontsize=8)
    fig.suptitle(f'{dataset.name}: ATE {before:.3f} m → {after:.3f} m')
    fig.savefig(output / 'slam_trajectory_comparison.png', dpi=180)
    plt.close(fig)

    links = dict(connection.execute(
        'SELECT type, COUNT(*) FROM Link GROUP BY type'))
    logged_frames = {row[0] for row in quality_rows}
    unlogged_frames = [row[2] for row in stamps
                       if row[2] not in logged_frames]
    positive_quality = [(row[0], row[3]) for row in quality_rows
                        if row[3] > 0]
    rejected_loops = re.findall(
        r'Loop closure (\d+)->(\d+) rejected!', log_text)
    metrics = {
        'dataset': dataset.name,
        'published_rgbd_pairs': len(stamps),
        'odom_quality_entries': len(quality_rows),
        'odom_positive_frames': sum(row[3] > 0 for row in quality_rows),
        'odom_zero_frames': [row[0] for row in quality_rows if row[3] == 0],
        'odom_unlogged_frame_indices': unlogged_frames,
        'odom_positive_quality_median': float(np.median(
            [quality for _, quality in positive_quality])),
        'odom_positive_quality_min_frame': list(min(
            positive_quality, key=lambda item: item[1])),
        'odom_log_stamp_match_max_ms': max(row[4] for row in quality_rows),
        'rtabmap_processed_frames': len(re.findall(
            r'rtabmap \(\d+\): Rate=', log_text)),
        'explicit_drop_warnings': log_text.count('Dropping image/scan data'),
        'rejected_global_loop_candidates': len(rejected_loops),
        'database_nodes': len(db_stamps),
        'graph_nodes': len(node_ids),
        'groundtruth_matched_nodes': len(selected),
        'max_ros_map_delta_ms': max(map_deltas),
        'max_groundtruth_delta_ms': float(max(gt_deltas) * 1000),
        'odom_ate_translation_rmse_m': before,
        'optimized_ate_translation_rmse_m': after,
        'neighbor_links': links.get(0, 0) // 2,
        'global_loop_links': links.get(1, 0) // 2,
        'local_space_links': links.get(2, 0) // 2,
    }
    (output / 'slam_metrics.json').write_text(
        json.dumps(metrics, indent=2) + '\n', encoding='utf-8')
    connection.close()
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
