"""第二阶段：TUM RGB-D 两帧 ORB + 深度反投影 + PnP/RANSAC。

运行：python test_pnp.py
默认使用 freiburg1_xyz 的第 0、2 帧；不读取 Groundtruth 作为位姿输入。
"""
from pathlib import Path
import argparse

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "data" / "rgbd_dataset_freiburg1_xyz"
OUTPUT = ROOT / "outputs"
FRAME_1 = 0
FRAME_2 = 2
DEPTH_SCALE = 5000.0  # TUM 深度原始值：5000 = 1 米
DEPTH_MAX_M = 5.0
SYNC_MAX_S = 0.03

# 与之前 RGB-D 项目一致的 Freiburg1 RGB 相机内参
K = np.array([
    [517.3, 0, 318.6],
    [0, 516.5, 255.3],
    [0, 0, 1],
], dtype=np.float64)


def read_index(index_file):
    """解析 TUM rgb.txt/depth.txt：timestamp relative_filename。"""
    records = []
    with index_file.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            timestamp, filename = line.split(maxsplit=1)
            records.append((float(timestamp), filename))
    if not records:
        raise ValueError(f"索引没有有效记录：{index_file}")
    return records


def load_rgb_depth(frame_index, rgb_records, depth_records, dataset=DATASET):
    """按时间戳读取指定帧及最近的深度帧，返回灰度图和 uint16 深度图。"""
    if not 0 <= frame_index < len(rgb_records):
        raise IndexError(f"帧序号 {frame_index} 超出 rgb.txt 范围")
    rgb_t, rgb_rel = rgb_records[frame_index]
    depth_t, depth_rel = min(
        depth_records, key=lambda record: abs(record[0] - rgb_t)
    )
    if abs(depth_t - rgb_t) > SYNC_MAX_S:
        raise RuntimeError(
            f"第 {frame_index} 帧 RGB/Depth 时间差过大：{abs(depth_t-rgb_t):.4f}s"
        )
    gray = cv2.imread(str(dataset / rgb_rel), cv2.IMREAD_GRAYSCALE)
    depth = cv2.imread(str(dataset / depth_rel), cv2.IMREAD_UNCHANGED)
    if gray is None or depth is None:
        raise FileNotFoundError(f"无法读取第 {frame_index} 帧的 RGB 或深度图")
    if depth.dtype != np.uint16:
        raise ValueError(f"期望 TUM uint16 深度图，实际为 {depth.dtype}")
    return gray, depth


def main():
    """估计两帧之间的 T_2_1，并将数值与内点匹配图保存到输出目录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=DATASET)
    parser.add_argument('--frame-1', type=int, default=FRAME_1)
    parser.add_argument('--frame-2', type=int, default=FRAME_2)
    parser.add_argument('--output-dir', type=Path, default=OUTPUT)
    args = parser.parse_args()
    dataset = args.dataset.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    rgb_records = read_index(dataset / "rgb.txt")
    depth_records = read_index(dataset / "depth.txt")
    gray1, depth1 = load_rgb_depth(args.frame_1, rgb_records, depth_records, dataset)
    gray2, _ = load_rgb_depth(args.frame_2, rgb_records, depth_records, dataset)

    # 1. ORB 提取特征与描述子
    orb = cv2.ORB_create(nfeatures=1500)
    kp1, des1 = orb.detectAndCompute(gray1, None)
    kp2, des2 = orb.detectAndCompute(gray2, None)
    if des1 is None or des2 is None:
        raise RuntimeError("ORB 没有提取到有效描述子")

    # 2. KNN 匹配 + ratio test 过滤错误匹配
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    candidate_pairs = matcher.knnMatch(des1, des2, k=2)
    matches = [pair[0] for pair in candidate_pairs
               if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance]

    # 3. 将第 1 帧特征像素 + Depth 反投影为 3D，匹配第 2 帧像素 2D
    points_3d, pixels_2d, valid_matches = [], [], []
    h, w = depth1.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    for match in matches:
        u1, v1 = kp1[match.queryIdx].pt
        u2, v2 = kp2[match.trainIdx].pt
        x, y = int(round(u1)), int(round(v1))
        if not (0 <= x < w and 0 <= y < h):
            continue
        z = float(depth1[y, x]) / DEPTH_SCALE
        if not (0.1 < z < DEPTH_MAX_M):
            continue
        points_3d.append([(u1 - cx) * z / fx, (v1 - cy) * z / fy, z])
        pixels_2d.append([u2, v2])
        valid_matches.append(match)

    if len(points_3d) < 8:
        raise RuntimeError(
            f"有效 3D-2D 对应点只有 {len(points_3d)} 组，无法可靠估计位姿"
        )
    object_points = np.asarray(points_3d, dtype=np.float32)
    image_points = np.asarray(pixels_2d, dtype=np.float32)

    # 4. PnP + RANSAC：剔除明显错误的 3D-2D 对应点
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points, image_points, K, None,
        iterationsCount=1000,
        reprojectionError=3.0,
        confidence=0.99,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < 6:
        raise RuntimeError("PnP/RANSAC 求解失败或有效内点太少")

    inlier_idx = inliers.ravel()
    # 5. 使用 RANSAC 内点进行非线性位姿细化
    ok, rvec, tvec = cv2.solvePnP(
        object_points[inlier_idx], image_points[inlier_idx], K, None,
        rvec=rvec, tvec=tvec, useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("PnP 位姿细化失败")

    R, _ = cv2.Rodrigues(rvec)
    T_2_1 = np.eye(4, dtype=np.float64)
    T_2_1[:3, :3] = R
    T_2_1[:3, 3] = tvec.ravel()
    # 注意：PnP 求出第 1 帧相机点 → 第 2 帧相机坐标的变换。
    # 若把第 1 帧设为世界系，则第 2 帧的世界位姿应使用逆矩阵。
    T_1_2 = np.linalg.inv(T_2_1)

    projected, _ = cv2.projectPoints(object_points[inlier_idx], rvec, tvec, K, None)
    errors = projected.reshape(-1, 2) - image_points[inlier_idx]
    reprojection_rmse = np.sqrt(np.mean(np.sum(errors ** 2, axis=1)))

    print(f"图像帧：{args.frame_1} → {args.frame_2}")
    print(f"ORB特征数：{len(kp1)} / {len(kp2)}")
    print(f"Ratio Test有效匹配：{len(matches)}")
    print(f"具备有效深度的3D-2D对应点：{len(object_points)}")
    print(f"RANSAC内点：{len(inlier_idx)}，内点率：{len(inlier_idx)/len(object_points):.1%}")
    print(f"内点重投影RMSE：{reprojection_rmse:.3f} px")
    print("T_2_1（第1帧相机坐标 → 第2帧相机坐标）：\n", T_2_1)
    print("T_1_2（第2帧相机坐标 → 第1帧坐标，建轨迹会用）：\n", T_1_2)

    np.savez(
        output / "two_frame_pose.npz", T_2_1=T_2_1, T_1_2=T_1_2,
        frame_indices=[args.frame_1, args.frame_2], inlier_count=len(inlier_idx),
    )
    np.savetxt(output / "T_1_2.txt", T_1_2, fmt="%.8f")
    inlier_matches = [valid_matches[i] for i in inlier_idx[:80]]
    vis = cv2.drawMatches(
        gray1, kp1, gray2, kp2, inlier_matches, None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    cv2.imwrite(str(output / "pnp_inliers.png"), vis)
    print(f"结果保存到 {output}/two_frame_pose.npz、T_1_2.txt、pnp_inliers.png")


if __name__ == "__main__":
    main()
