"""ORB + RGB-D + PnP/RANSAC 多帧视觉里程计。

默认保持原有 TUM 设置；Bonn 使用 --camera-profile bonn 和独立输出目录。
位姿估计不读取 Groundtruth。
"""
from pathlib import Path
import argparse
import csv

import cv2
import matplotlib.pyplot as plt
import numpy as np

from test_pnp import (
    DATASET, OUTPUT, K, DEPTH_SCALE, DEPTH_MAX_M, read_index,
)

# 默认 TUM 实验：从第 0 帧开始，每隔 2 帧取 1 帧。
START_FRAME = 0
NUM_FRAMES = 200
FRAME_STEP = 2

ORB_FEATURES = 1500
RATIO_THRESHOLD = 0.75
MIN_CORRESPONDENCES = 8
MIN_INLIERS = 8
BONN_K = np.array([[542.822841, 0., 315.593520],
                   [0., 542.576870, 237.756098],
                   [0., 0., 1.]], dtype=np.float64)
BONN_D = np.array([0.039903, -0.099343, -0.000730,
                   -0.000144, 0.], dtype=np.float64)


def load_frame(dataset, frame_index, rgb_records, depth_records, rectify_map,
               max_dt):
    """按 RGB 帧号读取最近深度；返回灰度和深度，超出 max_dt 则报错。"""
    rgb_time, rgb_path = rgb_records[frame_index]
    depth_time, depth_path = min(depth_records,
                                 key=lambda item: abs(item[0] - rgb_time))
    if abs(depth_time - rgb_time) > max_dt:
        raise RuntimeError(f'第 {frame_index} 帧 RGB/Depth 时间差超过 {max_dt*1000:g} ms')
    gray = cv2.imread(str(dataset / rgb_path), cv2.IMREAD_GRAYSCALE)
    depth = cv2.imread(str(dataset / depth_path), cv2.IMREAD_UNCHANGED)
    if gray is None or depth is None:
        raise FileNotFoundError(f'第 {frame_index} 帧图像读取失败')
    if depth.dtype != np.uint16:
        raise ValueError(f'预期 uint16 深度图，实际 {depth.dtype}')
    if rectify_map is not None:
        gray = cv2.remap(gray, *rectify_map, cv2.INTER_LINEAR)
        depth = cv2.remap(depth, *rectify_map, cv2.INTER_NEAREST)
    return gray, depth


def estimate_relative_pose(prev_kp, prev_des, prev_depth, cur_kp, cur_des, matcher,
                           camera_k=K, depth_scale=DEPTH_SCALE):
    """由上一帧 3D 点与当前帧 2D 点估计 T_cur_prev，并返回匹配/内点统计。"""
    if prev_des is None or cur_des is None:
        raise ValueError("ORB未提取到描述子")

    pairs = matcher.knnMatch(prev_des, cur_des, k=2)
    good_matches = []
    for pair in pairs:
        if len(pair) == 2 and pair[0].distance < RATIO_THRESHOLD * pair[1].distance:
            good_matches.append(pair[0])

    points_3d, pixels_2d = [], []
    h, w = prev_depth.shape
    fx, fy, cx, cy = camera_k[0, 0], camera_k[1, 1], camera_k[0, 2], camera_k[1, 2]

    for m in good_matches:
        u1, v1 = prev_kp[m.queryIdx].pt
        u2, v2 = cur_kp[m.trainIdx].pt
        x, y = int(round(u1)), int(round(v1))
        if not (0 <= x < w and 0 <= y < h):
            continue
        # 深度 PNG 的 5000 对应 1 米；相机内参把像素反投影到光学坐标系。
        z = float(prev_depth[y, x]) / depth_scale
        if not (0.1 < z < DEPTH_MAX_M):
            continue

        points_3d.append([(u1 - cx) * z / fx, (v1 - cy) * z / fy, z])
        pixels_2d.append([u2, v2])

    if len(points_3d) < MIN_CORRESPONDENCES:
        raise ValueError(f"有效3D-2D对应点不足：{len(points_3d)}")
    pts3 = np.asarray(points_3d, dtype=np.float32)
    pts2 = np.asarray(pixels_2d, dtype=np.float32)

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        pts3, pts2, camera_k, None,
        iterationsCount=1000, reprojectionError=3.0,
        confidence=0.99, flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < MIN_INLIERS:
        count = 0 if inliers is None else len(inliers)
        raise ValueError(f"PnP/RANSAC失败或内点不足：{count} 内点/{len(pts3)} 对应点")

    ids = inliers.ravel()
    ok, rvec, tvec = cv2.solvePnP(
        pts3[ids], pts2[ids], camera_k, None,
        rvec=rvec, tvec=tvec, useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise ValueError("PnP内点迭代优化失败")

    projected, _ = cv2.projectPoints(pts3[ids], rvec, tvec, camera_k, None)
    residual = projected.reshape(-1, 2) - pts2[ids]
    rmse_px = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))

    R, _ = cv2.Rodrigues(rvec)
    T_cur_prev = np.eye(4)
    T_cur_prev[:3, :3] = R
    T_cur_prev[:3, 3] = tvec.ravel()
    if not np.isfinite(T_cur_prev).all():
        raise ValueError("位姿矩阵出现非有限值")

    return T_cur_prev, len(good_matches), len(pts3), len(ids), rmse_px


def main():
    """依次估计相邻 RGB-D 帧，累积相机位姿并保存逐帧质量记录。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', type=Path, default=DATASET)
    parser.add_argument('--output-dir', type=Path, default=OUTPUT)
    parser.add_argument('--camera-profile', choices=['tum', 'bonn'], default='tum')
    parser.add_argument('--start-frame', type=int, default=START_FRAME)
    parser.add_argument('--max-frames', type=int, default=NUM_FRAMES,
                        help='0 表示从起始帧到序列末尾')
    parser.add_argument('--frame-step', type=int, default=FRAME_STEP)
    args = parser.parse_args()
    if args.start_frame < 0 or args.max_frames < 0 or args.frame_step <= 0:
        parser.error('起始帧、最大帧数和帧间隔无效')
    dataset = args.dataset.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    camera_k = BONN_K if args.camera_profile == 'bonn' else K
    rectify_map = (cv2.initUndistortRectifyMap(
        BONN_K, BONN_D, None, BONN_K, (640, 480), cv2.CV_32FC1)
        if args.camera_profile == 'bonn' else None)
    rgb_records = read_index(dataset / "rgb.txt")
    depth_records = read_index(dataset / "depth.txt")
    stop = (len(rgb_records) if args.max_frames == 0 else
            min(args.start_frame + args.max_frames * args.frame_step,
                len(rgb_records)))
    frame_indices = list(range(
        args.start_frame, stop, args.frame_step,
    ))
    candidate_count = len(frame_indices)
    max_dt = 0.02 if args.camera_profile == 'bonn' else 0.03
    unpaired = [i for i in frame_indices
                if min(abs(depth_time - rgb_records[i][0])
                       for depth_time, _ in depth_records) > max_dt]
    if unpaired:
        print(f'RGB/Depth 未匹配帧 {len(unpaired)}：{unpaired[:20]}')
    unpaired_set = set(unpaired)
    frame_indices = [i for i in frame_indices if i not in unpaired_set]
    if len(frame_indices) < 2:
        raise ValueError("至少需要两帧图像")

    orb = cv2.ORB_create(nfeatures=ORB_FEATURES)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    # 第一帧坐标系作为“世界”参考系，位姿设为单位矩阵
    prev_index = frame_indices[0]
    prev_gray, prev_depth = load_frame(dataset, prev_index, rgb_records,
                                       depth_records, rectify_map, max_dt)
    prev_kp, prev_des = orb.detectAndCompute(prev_gray, None)
    T_world_prev = np.eye(4)
    trajectory = [T_world_prev.copy()]
    valid_indices = [prev_index]
    timestamps = [rgb_records[prev_index][0]]
    rows = [[prev_index, timestamps[0], 0.0, 0.0, 0.0, 0, 0, 0, 0.0]]
    failures = []
    print(f"第{prev_index:04d}帧：初始化参考坐标系")

    for frame_index in frame_indices[1:]:
        try:
            cur_gray, cur_depth = load_frame(dataset, frame_index, rgb_records,
                                              depth_records, rectify_map, max_dt)
            cur_kp, cur_des = orb.detectAndCompute(cur_gray, None)

            # PnP给出“上一帧 -> 当前帧”的坐标变换
            T_cur_prev, n_match, n_3d2d, n_inlier, rmse = estimate_relative_pose(
                prev_kp, prev_des, prev_depth, cur_kp, cur_des, matcher,
                camera_k=camera_k,
            )

            # 要在首帧坐标系表示当前相机位姿，需要将相对变换取逆再累乘
            T_world_cur = T_world_prev @ np.linalg.inv(T_cur_prev)
            x, y, z = T_world_cur[:3, 3]

        except (ValueError, RuntimeError, cv2.error, FileNotFoundError) as exc:
            failures.append([frame_index, rgb_records[frame_index][0], str(exc)])
            print(f"第{frame_index:04d}帧估计失败：{exc}。为避免错误位姿累积，停止本次序列。")
            break

        trajectory.append(T_world_cur.copy())
        valid_indices.append(frame_index)
        timestamps.append(rgb_records[frame_index][0])
        rows.append([frame_index, timestamps[-1], x, y, z,
                     n_match, n_3d2d, n_inlier, rmse])
        print(f"第{prev_index:04d} -> {frame_index:04d}帧 | "
              f"匹配{n_match} | 3D-2D {n_3d2d} | "
              f"内点{n_inlier} | RMSE {rmse:.3f}px | "
              f"相机位置({x:.3f}, {y:.3f}, {z:.3f})m")

        prev_index = frame_index
        prev_gray, prev_depth = cur_gray, cur_depth
        prev_kp, prev_des = cur_kp, cur_des
        T_world_prev = T_world_cur

    poses = np.stack(trajectory)
    positions = poses[:, :3, 3]
    np.savez(
        output / "vo_poses.npz",
        poses=poses,
        frame_indices=np.asarray(valid_indices),
        timestamps=np.asarray(timestamps),
        candidate_frame_count=candidate_count,
        paired_frame_count=len(frame_indices),
        unpaired_frame_indices=np.asarray(unpaired),
    )
    with (output / "vo_trajectory.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["frame_index", "timestamp", "x_m", "y_m", "z_m",
                         "matches", "points_3d2d", "ransac_inliers", "reproj_rmse_px"])
        writer.writerows(rows)
    with (output / "vo_failures.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["frame_index", "timestamp", "reason"])
        writer.writerows(failures)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], marker=".")
    ax.scatter(*positions[0], marker="o", s=70, label="Start")
    ax.scatter(*positions[-1], marker="x", s=70, label="End")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("RGB-D Visual Odometry (first frame as world)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "vo_trajectory.png", dpi=180)
    plt.close(fig)

    print(f"\n有效位姿 {len(trajectory)}/{len(frame_indices)} 帧；"
          f"失败 {len(failures)} 帧；未尝试 "
          f"{len(frame_indices)-len(trajectory)-len(failures)} 帧")
    print(f"结果：{output}/vo_poses.npz、vo_trajectory.csv、vo_failures.csv")
    print("注意：这只是VO估计轨迹；是否准确需要下一阶段与Groundtruth对比。")


if __name__ == "__main__":
    main()
