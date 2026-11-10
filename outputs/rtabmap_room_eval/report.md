# freiburg1_room：回环与位姿图优化评估

## 数据与方法

- 现有数据库包含 239 个节点；`rtabmap-export --poses_camera --opt 3/0 --poses_format 11` 从同一数据库分别导出 174 个图节点。其余 65 个节点的 `weight=-9`，不在导出的位姿图中。
- 数据库 JPEG 与 TUM 原始 RGB PNG 逐帧比对恢复时间戳。最大同帧像素 MAE 为 3.63，候选次优 MAE 最小为 7.47；没有模糊匹配。
- 两条导出轨迹的节点 ID 相同，ROS 时间戳在导出精度内一致；174 个图节点均匹配到 Groundtruth，最大时间差 4.75 ms。
- 使用 `--poses_camera` 导出相机位姿。此项目的 `frame_id=camera_optical_frame`，核查 `--poses` 与 `--poses_camera` 的坐标差小于 1 微米。[TUM 官方格式说明](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)指出 Groundtruth 平移为 RGB 相机光心在动捕世界坐标系中的位置。ATE 对两条轨迹分别使用同一个 SVD SE(3) 刚体对齐方法，**不调整尺度**。

## 回环约束

数据库 `Link.type=0` 为相邻里程计约束 173 条，`type=1` 为全局视觉回环约束 14 条，`type=2` 为局部空间回环约束 19 条（类型名来自已安装 RTAB-Map 的 `Link.h`）。SQLite 对每条约束保存两个方向，因此原始行数分别是 346、28、38。

| 当前节点 | 历史节点 | 当前 RGB 帧 | 历史 RGB 帧 |
|---:|---:|---:|---:|
| 183 | 3 | 1038 | 18 |
| 179 | 4 | 1016 | 24 |
| 178 | 5 | 1011 | 29 |
| 213 | 39 | 1213 | 219 |
| 214 | 40 | 1218 | 225 |
| 216 | 41 | 1229 | 230 |
| 217 | 42 | 1234 | 235 |
| 234 | 42 | 1329 | 235 |
| 236 | 42 | 1341 | 235 |
| 219 | 43 | 1245 | 241 |
| 230 | 43 | 1307 | 241 |
| 130 | 111 | 734 | 624 |
| 126 | 113 | 711 | 636 |
| 128 | 113 | 723 | 636 |

`/rtabmap/info` 的非零 `loop_closure_id` 共 14 次；其历史节点 ID 多重集合与数据库 14 条全局回环完全一致（匹配：True）。代表性图像见 `loop_examples.png`，可见同一显示器、办公桌和键盘等场景从不同视角被再次观察。RTAB-Map 先以外观词袋检索历史节点，再以几何配准验证回环并写入 `type=1` 约束；位置已接近的节点可另形成 `type=2` 约束。

## ATE 平移 RMSE

| 指标 | 优化前 Odometry | 全局图优化后 |
|---|---:|---:|
| ATE 平移 RMSE | 0.182079 m (18.21 cm) | 0.062574 m (6.26 cm) |

误差降低 **65.63%**。位姿图优化利用回环等图约束修正了累积漂移；轨迹见 `trajectory_comparison.png`。该对比体现完整图优化的效果，不能把全部改善单独归因于 14 条全局视觉回环，因为图中还包含 19 条局部空间回环约束。ATE 只覆盖 174 个导出图节点，不代表全部 1362 对输入图像的误差。剩余误差可能来自 RGB-D 噪声、相机标定近似及特征与约束误差。此处没有用 Groundtruth 代替位姿估计，也没有按节点序号推断时间戳。

## 复现命令（只读取现有数据库）

```bash
cd ~/RGBD_Visual_SLAM
source /opt/ros/jazzy/setup.bash
rtabmap-export --poses_camera --opt 3 --poses_format 11 --output room_odom --output_dir outputs/rtabmap_room_eval outputs/rtabmap_room_verified.db
rtabmap-export --poses_camera --opt 0 --poses_format 11 --output room_optimized --output_dir outputs/rtabmap_room_eval outputs/rtabmap_room_verified.db
python3 evaluate_rtabmap_room.py
```
