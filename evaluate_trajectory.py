"""比较 ORB+PnP RGB-D VO 与 TUM/Bonn Groundtruth 的相机轨迹。

默认保持原有 Freiburg1 XYZ 输入输出；Bonn 用命令行指定数据和输出目录。
只做离线评估：ATE 使用无尺度变化的 SE(3) 对齐。
"""
from pathlib import Path
import argparse
import csv
import json

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "data" / "rgbd_dataset_freiburg1_xyz"
OUTPUT = ROOT / "outputs"
MAX_TIME_DIFF = 0.02  # 秒：RGB 与 Groundtruth 时间戳最大差值


def quaternion_to_rotation(qx, qy, qz, qw):
    """把 Groundtruth 的 (x, y, z, w) 单位四元数转成旋转矩阵。"""
    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def read_groundtruth(path):
    """读取 TUM 格式时间戳与 4×4 位姿，返回按时间排序的记录。"""
    records = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            vals = [float(x) for x in line.split()]
            if len(vals) != 8:
                continue
            stamp, tx, ty, tz, qx, qy, qz, qw = vals
            T = np.eye(4)
            T[:3, :3] = quaternion_to_rotation(qx, qy, qz, qw)
            T[:3, 3] = [tx, ty, tz]
            records.append((stamp, T))
    if not records:
        raise ValueError(f"无法读取 Groundtruth: {path}")
    return sorted(records, key=lambda item: item[0])


def bonn_camera_groundtruth(records):
    """按 Bonn 官方 T_ros^-1 * T_gt * T_ros * T_m 转为 RGB-D 相机位姿。"""
    t_ros = np.array([[-1., 0., 0., 0.],
                      [0., 0., 1., 0.],
                      [0., 1., 0., 0.],
                      [0., 0., 0., 1.]])
    t_m = np.array([[1.0157, 0.1828, -0.2389, 0.0113],
                    [0.0009, -0.8431, -0.6413, -0.0098],
                    [-0.3009, 0.6147, -0.8085, 0.0111],
                    [0., 0., 0., 1.]])
    # 官方 3x3 矩阵不严格正交；仅投影到 SO(3) 供旋转 RPE 使用。
    # ATE 使用的平移项仍是官方给出的数值。
    u, _, vt = np.linalg.svd(t_m[:3, :3])
    t_m[:3, :3] = u @ vt
    return [(stamp, t_ros @ pose @ t_ros @ t_m) for stamp, pose in records]


def match_poses(times, poses, gt_records):
    """按最近时间戳关联估计轨迹和真值，返回成对位姿与时间差。"""
    gt_times = np.array([t for t, _ in gt_records])
    chosen_vo = []
    chosen_gt = []
    matched_times = []
    time_deltas = []
    for time, pose in zip(times, poses):
        idx = int(np.searchsorted(gt_times, time))
        candidates = [j for j in (idx - 1, idx) if 0 <= j < len(gt_times)]
        best = min(candidates, key=lambda j: abs(gt_times[j] - time))
        delta = abs(gt_times[best] - time)
        if delta <= MAX_TIME_DIFF:
            chosen_vo.append(pose)
            chosen_gt.append(gt_records[best][1])
            matched_times.append(time)
            time_deltas.append(delta)
    if len(chosen_vo) < 3:
        raise ValueError("时间戳匹配不足3帧，请检查数据集或 MAX_TIME_DIFF")
    return np.stack(chosen_vo), np.stack(chosen_gt), np.array(matched_times), np.array(time_deltas)


def rigid_align(source, target):
    """用 SVD 求 source→target 的 SE(3) 刚体对齐，返回旋转和平移，不缩放。"""
    center_s = source.mean(axis=0)
    center_t = target.mean(axis=0)
    H = (source - center_s).T @ (target - center_t)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = center_t - R @ center_s
    return R, t


def rotation_angle_deg(R):
    """返回旋转矩阵对应的角度，单位为度。"""
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def main():
    """读取保存的 VO 位姿并计算同一时间关联下的 ATE、RPE。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, default=DATASET)
    parser.add_argument('--output-dir', type=Path, default=OUTPUT)
    parser.add_argument('--groundtruth-profile', choices=['tum', 'bonn'],
                        default='tum')
    args = parser.parse_args()
    dataset = args.dataset.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    vo_file = output / "vo_poses.npz"
    if not vo_file.is_file():
        raise FileNotFoundError(f"未找到 {vo_file}，请先运行 visual_odometry.py")
    saved = np.load(vo_file)
    vo_poses = saved["poses"]
    times = saved["timestamps"]
    gt = read_groundtruth(dataset / "groundtruth.txt")
    if args.groundtruth_profile == 'bonn':
        gt = bonn_camera_groundtruth(gt)
    valid_vo_count = len(times)
    vo_poses, gt_poses, times, deltas = match_poses(times, vo_poses, gt)

    vo_xyz = vo_poses[:, :3, 3]
    gt_xyz = gt_poses[:, :3, 3]
    R_align, t_align = rigid_align(vo_xyz, gt_xyz)
    aligned_xyz = (R_align @ vo_xyz.T).T + t_align
    ate_per_frame = np.linalg.norm(aligned_xyz - gt_xyz, axis=1)
    ate_rmse = float(np.sqrt(np.mean(ate_per_frame ** 2)))

    # RPE：用相邻成功帧的相对运动评估局部里程计误差，不做轨迹形状对齐。
    rpe_trans = []
    rpe_rot = []
    for i in range(len(times) - 1):
        delta_vo = np.linalg.inv(vo_poses[i]) @ vo_poses[i + 1]
        delta_gt = np.linalg.inv(gt_poses[i]) @ gt_poses[i + 1]
        error_T = np.linalg.inv(delta_gt) @ delta_vo
        rpe_trans.append(np.linalg.norm(error_T[:3, 3]))
        rpe_rot.append(rotation_angle_deg(error_T[:3, :3]))
    rpe_trans = np.asarray(rpe_trans)
    rpe_rot = np.asarray(rpe_rot)

    output.mkdir(parents=True, exist_ok=True)
    with (output / "trajectory_errors.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "ate_translation_m", "rpe_translation_m", "rpe_rotation_deg"])
        for i, t in enumerate(times):
            writer.writerow([t, ate_per_frame[i],
                             "" if i == 0 else rpe_trans[i - 1],
                             "" if i == 0 else rpe_rot[i - 1]])

    rgb_times = [float(line.split()[0]) for line in
                 (dataset / 'rgb.txt').read_text().splitlines()
                 if line.strip() and not line.lstrip().startswith('#')]
    full_gt_xyz = np.stack([pose[:3, 3] for stamp, pose in gt
                            if rgb_times[0] <= stamp <= rgb_times[-1]])

    if args.groundtruth_profile == 'bonn':
        fig, (xy, xz) = plt.subplots(1, 2, figsize=(12, 5),
                                     constrained_layout=True)
        tracks = [('Groundtruth (full)', full_gt_xyz, '0.6', '--'),
                  ('Groundtruth (matched)', gt_xyz, 'black', '-'),
                  ('ORB + PnP VO', aligned_xyz, '#1f77b4', '-')]
        for label, xyz, color, style in tracks:
            xy.plot(xyz[:, 0], xyz[:, 1], style, color=color, label=label)
            xz.plot(xyz[:, 0], xyz[:, 2], style, color=color, label=label)
        xy.set(xlabel='X (m)', ylabel='Y (m)', title='Top view')
        xz.set(xlabel='X (m)', ylabel='Z (m)', title='Side view')
        xy.set_aspect('equal', adjustable='box')
        for ax in (xy, xz):
            ax.grid(alpha=.25)
        xy.legend(fontsize=8)
        fig.suptitle(f'{dataset.name}: VO ATE {ate_rmse*100:.2f} cm')
    else:
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(*full_gt_xyz.T, color='0.6', linestyle='--',
                label='Groundtruth (full sequence)')
        ax.plot(*gt_xyz.T, color='black', label='Groundtruth (matched)')
        ax.plot(*aligned_xyz.T, label="ORB + PnP VO (SE3 aligned)")
        ax.scatter(*gt_xyz[0], marker="o", s=40, label="Start")
        ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
               title=f"{dataset.name}: VO vs Groundtruth")
        all_xyz = np.vstack([gt_xyz, aligned_xyz])
        spread = np.maximum(np.ptp(all_xyz, axis=0), 0.05)
        ax.set_box_aspect(spread)
        ax.legend()
        fig.tight_layout()
    path = output / "vo_vs_groundtruth.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)

    total_rgb = sum(bool(line.strip()) and not line.lstrip().startswith('#')
                    for line in (dataset / 'rgb.txt').read_text().splitlines())
    failures_file = output / 'vo_failures.csv'
    failure_count = (max(0, sum(1 for _ in failures_file.open()) - 1)
                     if failures_file.exists() else 0)
    metrics = {
        'dataset': dataset.name,
        'total_rgb_frames': total_rgb,
        'candidate_rgb_frames': int(saved['candidate_frame_count'])
        if 'candidate_frame_count' in saved else total_rgb,
        'paired_rgbd_frames': int(saved['paired_frame_count'])
        if 'paired_frame_count' in saved else valid_vo_count,
        'unpaired_rgb_frames': int(len(saved['unpaired_frame_indices']))
        if 'unpaired_frame_indices' in saved else 0,
        'valid_vo_frames': valid_vo_count,
        'failed_attempts': failure_count,
        'unprocessed_paired_frames': (int(saved['paired_frame_count']) -
                                      valid_vo_count - failure_count)
        if 'paired_frame_count' in saved else 0,
        'groundtruth_matched_frames': len(times),
        'max_groundtruth_time_delta_ms': float(max(deltas) * 1000),
        'ate_translation_rmse_m': ate_rmse,
        'rpe_translation_rmse_m': float(np.sqrt(np.mean(rpe_trans ** 2))),
        'rpe_rotation_rmse_deg': float(np.sqrt(np.mean(rpe_rot ** 2))),
        'groundtruth_profile': args.groundtruth_profile,
    }
    (output / 'vo_metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')

    print(f"时间戳成功匹配：{len(times)} 帧；最大时间差：{max(deltas)*1000:.2f} ms")
    print("ATE (SE3刚体对齐、不校正尺度)：")
    print(f"  平移 RMSE：{ate_rmse:.4f} m ({ate_rmse*100:.2f} cm)")
    print(f"  中位数：{np.median(ate_per_frame)*100:.2f} cm")
    print("RPE (相邻已匹配帧)：")
    print(f"  平移 RMSE：{np.sqrt(np.mean(rpe_trans**2))*100:.2f} cm")
    print(f"  旋转 RMSE：{np.sqrt(np.mean(rpe_rot**2)):.3f} deg")
    print(f"图像：{path}")
    print(f"逐帧数据：{output / 'trajectory_errors.csv'}")
    print("注意：ATE是最优刚体对齐后的整段轨迹误差；RPE只评估相邻处理帧的相对运动。")


if __name__ == "__main__":
    main()
