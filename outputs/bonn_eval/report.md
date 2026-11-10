# Bonn RGB-D 动态场景实验

## 数据与处理

实验只下载 Bonn 官网指定的两个单独序列：

| 序列 | 本地目录 | RGB | Depth | 20 ms 内配对 |
|---|---|---:|---:|---:|
| balloon | `data/rgbd_bonn_balloon/` | 439 | 440 | 438 |
| moving_obstructing_box | `data/rgbd_bonn_moving_obstructing_box/` | 590 | 592 | 590 |

两个 ZIP 均通过 CRC 检查。PNG 为 640×480；RGB 为 8 位彩色，深度为 16 位无符号整数。[Bonn 官方说明](https://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/)指出序列沿用 TUM 格式；[TUM 格式说明](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)规定深度 PNG 的 5000 表示 1 米。因此将深度原值除以 5000 得到米，发布时除以 5 转为 ROS 16UC1 毫米。Bonn 官方说明深度已配准到 RGB。balloon 原始 RGB 第 0 帧与最近深度相差 29.36 ms，超过已有 20 ms 门限，故实际从原始 RGB 第 1 帧开始发布；它没有被计入 438 对。

Bonn 模式使用官网给出的 K=(542.822841, 542.576870, 315.593520, 237.756098) 和 D=(0.039903, -0.099343, -0.000730, -0.000144, 0)。发布与独立 VO 都将**已配准的 RGB 和深度用同一映射校正**；因此实际 CameraInfo 使用 Bonn K/P，输出图像的剩余 D 为零。ROS 实测 CameraInfo 与此一致。Bonn 官网没有明确说 PNG 是否已预先去畸变，故“需要以官方非零 D 校正”是本实验的预处理假设，应在报告指标时保留。RTAB-Map [ROS 2 文档](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/doc/rtabmap.md)说明默认假设输入图像已校正。

Groundtruth 先按 Bonn 官网给出的 `T_ros^-1 · T_gt · T_ros · T_m` 变换到 RGB-D 相机位姿，再做时间戳最近邻匹配（最大 20 ms）和 SE(3) 刚体对齐；**没有尺度修正**。独立 VO 使用原始数据时间戳。ROS 发布节点额外记录 `stamp_map.csv`，将 ROS 纳秒时间戳、Bonn 时间戳、发布帧号和原始文件名逐帧对应；数据库图节点与该表的最大匹配误差仅 0.000127 ms。

## 结果

| 指标 | balloon | moving_obstructing_box |
|---|---:|---:|
| 原始 RGB 帧 / RGB-D 配对 | 439 / 438 | 590 / 590 |
| 独立 VO 有效位姿 / 配对 | 438 / 438 | 590 / 590 |
| 独立 VO 跟踪失败 | 0 | 0 |
| VO 参与 Groundtruth ATE 的帧 | 438 | 589（首帧不在 20 ms 范围内） |
| **独立 VO ATE 平移 RMSE** | **42.21 cm** | **61.88 cm** |
| ROS 里程计正质量日志 / 发布对 | 430 / 438 | 585 / 590 |
| ROS `quality=0` | 1 次，发布对 6（初始化） | 1 次，发布对 4（初始化） |
| RTAB-Map 实际处理帧 / 数据库节点 | 78 / 78 | 106 / 106 |
| 导出的有效图节点 / GT 匹配 | 23 / 23 | 42 / 42 |
| RTAB-Map 原始里程计 ATE（仅图节点） | 15.18 cm | 63.09 cm |
| RTAB-Map 优化后 ATE（同一批图节点） | 11.99 cm | 57.90 cm |
| 接受的全局视觉回环 / 局部空间约束 | 0 / 3 | 0 / 1 |
| 被拒绝的全局回环候选 | 6 | 19 |
| 三维地图 | 人物和气球多位置重影明显 | 纸箱点云进入地图；此截图视角不能量化重影 |

两次 ROS 实验关闭了 `always_process_most_recent_frame` 的主动丢帧模式；完整日志里没有 `Dropping image/scan data` 警告。balloon 未形成正质量日志的发布对为 0–5 和 437；moving_obstructing_box 为 0–3，集中在启动或结束边界。两次唯一的 `quality=0` 均是首个里程计初始化输入，后续没有持续跟踪丢失。ROS 正质量次数是日志统计，不等同于对 `/odom` 话题另行录包计数。

## 动态物体影响

- balloon 原始 RGB 第 57 帧可见人物运动和画面模糊；该帧独立 VO 只有 144 个匹配、133 个有效 3D-2D 点、46 个 RANSAC 内点（成功帧内点中位数 738），但仍成功估计位姿。RTAB-Map 最低正质量为 114，发生在发布对 55。RViz 截图可直接看到同一人物与气球的多位置点云。
- moving_obstructing_box 原始第 202–206 帧，大纸箱几乎遮满视野；第 202 帧只有 31 个匹配和 19 个 RANSAC 内点（成功帧内点中位数 789），RTAB-Map 最低正质量为 88（发布对 190）。算法仍连续跟踪，但误差明显累积。
- 两组序列的相机运动、场景几何和时长不同，**不能仅以 42.21 cm 与 61.88 cm 的 ATE 大小断言遮挡一定比气球运动更严重**。更直接的局部证据是遮挡区间的匹配和质量骤降。
- 两组没有接受全局回环。日志显示全局候选经图优化一致性检查被拒绝；数据库 `Link.type=1` 均为零。优化前后少量改善来自完整图约束（包括局部空间约束），不能称为全局回环带来的收益。

## 文件

- `balloon/vo_vs_groundtruth.png`、`balloon/slam_trajectory_comparison.png`、`balloon/dynamic_rgb_frame_0057.png`、`balloon/rtabmap_rviz.png`
- `moving_obstructing_box/vo_vs_groundtruth.png`、`moving_obstructing_box/slam_trajectory_comparison.png`、`moving_obstructing_box/dynamic_rgb_frame_0202.png`、`moving_obstructing_box/rtabmap_rviz.png`
- 每组的 `vo_metrics.json`、`slam_metrics.json`、`odom_quality.csv`、`stamp_map.csv` 和完整日志保存逐帧证据。

## 复现

终端 1，构建（两个序列只做一次）：

```bash
cd ~/RGBD_Visual_SLAM
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rgbd_slam_ros --symlink-install
source install/setup.bash
```

终端 2，离线 VO 与 ATE。使用新输出目录，保护本次实验成果：

```bash
cd ~/RGBD_Visual_SLAM
mkdir -p outputs/bonn_eval/balloon_rerun outputs/bonn_eval/moving_obstructing_box_rerun
python3 visual_odometry.py --dataset data/rgbd_bonn_balloon --camera-profile bonn --max-frames 0 --frame-step 1 --output-dir outputs/bonn_eval/balloon_rerun
python3 evaluate_trajectory.py --dataset data/rgbd_bonn_balloon --output-dir outputs/bonn_eval/balloon_rerun --groundtruth-profile bonn
python3 visual_odometry.py --dataset data/rgbd_bonn_moving_obstructing_box --camera-profile bonn --max-frames 0 --frame-step 1 --output-dir outputs/bonn_eval/moving_obstructing_box_rerun
python3 evaluate_trajectory.py --dataset data/rgbd_bonn_moving_obstructing_box --output-dir outputs/bonn_eval/moving_obstructing_box_rerun --groundtruth-profile bonn
```

终端 3，完整 ROS SLAM（balloon）：

```bash
cd ~/RGBD_Visual_SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
RUN=$PWD/outputs/bonn_eval/balloon_rerun
mkdir -p "$RUN/ros_logs"
set -o pipefail
ROS_DOMAIN_ID=77 ROS_LOG_DIR="$RUN/ros_logs" ros2 launch rgbd_slam_ros tum_rgbd_slam.launch.py dataset:=$PWD/data/rgbd_bonn_balloon camera_profile:=bonn always_process_most_recent_frame:=false stamp_map_path:=$RUN/stamp_map.csv database_path:=$RUN/rtabmap.db hz:=5 max_frames:=0 2>&1 | tee "$RUN/rtabmap.log"
```

终端 4，在终端 3 播放期间打开 RViz2：

```bash
cd ~/RGBD_Visual_SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=77 rviz2 -d rtabmap.rviz
```

终端 5，终端 3 结束后评估数据库：

```bash
cd ~/RGBD_Visual_SLAM
python3 evaluate_bonn_slam.py --dataset data/rgbd_bonn_balloon --database outputs/bonn_eval/balloon_rerun/rtabmap.db --stamp-map outputs/bonn_eval/balloon_rerun/stamp_map.csv --output-dir outputs/bonn_eval/balloon_rerun
```

第二组遮挡序列在终端 3 运行，终端 4 同时打开 RViz2（均用 ROS_DOMAIN_ID=82）：

```bash
cd ~/RGBD_Visual_SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
RUN=$PWD/outputs/bonn_eval/moving_obstructing_box_rerun
mkdir -p "$RUN/ros_logs"
set -o pipefail
ROS_DOMAIN_ID=82 ROS_LOG_DIR="$RUN/ros_logs" ros2 launch rgbd_slam_ros tum_rgbd_slam.launch.py dataset:=$PWD/data/rgbd_bonn_moving_obstructing_box camera_profile:=bonn always_process_most_recent_frame:=false stamp_map_path:=$RUN/stamp_map.csv database_path:=$RUN/rtabmap.db hz:=5 max_frames:=0 2>&1 | tee "$RUN/rtabmap.log"
```

```bash
cd ~/RGBD_Visual_SLAM
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=82 rviz2 -d rtabmap.rviz
```

SLAM 结束后在终端 5 评估第二组：

```bash
cd ~/RGBD_Visual_SLAM
python3 evaluate_bonn_slam.py --dataset data/rgbd_bonn_moving_obstructing_box --database outputs/bonn_eval/moving_obstructing_box_rerun/rtabmap.db --stamp-map outputs/bonn_eval/moving_obstructing_box_rerun/stamp_map.csv --output-dir outputs/bonn_eval/moving_obstructing_box_rerun
```

## 范围与限制

RTAB-Map ATE 仅对数据库中可导出的 23/42 个图节点计算，不能与独立 VO 的 438/589 帧 ATE 当作同一采样集合直接比较。本阶段已真实完成两组完整离线 VO、两组完整 ROS SLAM、数据库轨迹评估和 RViz2 地图截图；未做静态场景基线的受控消融、动态物体分割或点云几何误差定量评价。TUM 只做了 5 帧独立 VO、20 帧 ROS Launch 回归检查，未重跑此前已经完成的 TUM 全序列。
