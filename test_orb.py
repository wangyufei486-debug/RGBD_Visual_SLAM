"""两帧 TUM RGB 图像的 ORB 特征匹配示例，不使用深度或 Groundtruth。"""

import argparse
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent


def read_rgb_paths(dataset):
    """读取 rgb.txt，返回相对于数据集目录的图像路径列表。"""
    with (dataset / 'rgb.txt').open(encoding='utf-8') as index_file:
        paths = [line.split()[1] for line in index_file
                 if line.strip() and not line.lstrip().startswith('#')]
    if not paths:
        raise ValueError(f'RGB 索引为空：{dataset / "rgb.txt"}')
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path,
                        default=ROOT / 'data/rgbd_dataset_freiburg1_xyz')
    parser.add_argument('--frame-1', type=int, default=0)
    parser.add_argument('--frame-2', type=int, default=2)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'outputs')
    args = parser.parse_args()
    dataset = args.dataset.expanduser().resolve()
    paths = read_rgb_paths(dataset)
    if not 0 <= args.frame_1 < len(paths) or not 0 <= args.frame_2 < len(paths):
        parser.error('帧序号超出 rgb.txt 范围')

    image1 = cv2.imread(str(dataset / paths[args.frame_1]), cv2.IMREAD_GRAYSCALE)
    image2 = cv2.imread(str(dataset / paths[args.frame_2]), cv2.IMREAD_GRAYSCALE)
    if image1 is None or image2 is None:
        raise FileNotFoundError('RGB 图像读取失败')

    orb = cv2.ORB_create(nfeatures=1000)
    keypoints1, desc1 = orb.detectAndCompute(image1, None)
    keypoints2, desc2 = orb.detectAndCompute(image2, None)
    if desc1 is None or desc2 is None:
        raise RuntimeError('ORB 未提取到有效描述子')

    # KNN 为每个描述子找两个候选；ratio test 去掉明显歧义的匹配。
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = matcher.knnMatch(desc1, desc2, k=2)
    matches = []
    for pair in pairs:
        if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance:
            matches.append(pair[0])

    image = cv2.drawMatches(
        image1, keypoints1, image2, keypoints2, matches[:80], None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    path = output / 'orb_matches.png'
    if not cv2.imwrite(str(path), image):
        raise OSError(f'无法保存匹配图：{path}')
    print(f'图像特征点：{len(keypoints1)} / {len(keypoints2)}')
    print(f'有效匹配：{len(matches)}；结果：{path}')


if __name__ == '__main__':
    main()
