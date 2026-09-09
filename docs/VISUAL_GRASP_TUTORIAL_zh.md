# reBot Arm B601-RS + Gemini 2 视觉抓取教程

面向仓库：[Seeed-Projects/reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2)

本教程说明：**在已有机械臂 ROS2 控制栈上，如何增量加入腕部相机视觉抓取（识别 → 6D → 抓取 → 放盒）**，并给出可复现的日常运行流程。

参考（几何思路）：[Seeed Wiki - reBot Arm ROS2](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_ros2_integration/)  
本仓库实现与 Wiki 的差异：Wiki Demo 多用 SDK + `hand_eye.npz`；本方案用 **URDF/TF 外参 + ROS topic**，便于和 `rebotarmcontroller` 的 `move_to_pose` / 夹爪服务直接对接。

---

## 1. 目标流程（你最终要跑通的）

```text
上电 / CAN / 相机 USB
        │
        ▼
① bringup（原仓库，不改指令）
        │
        ▼
② wrist_camera（本包：Gemini2 驱动 + 腕部外参 TF）
        │
        ▼
③ grasp_yolo（到观察位 → YOLOE 检测 → 发 6D）
        │     发布: /grasp_pose /pregrasp_pose /place_pose
        ▼
④ grasp_execute（Trigger 一次跑完整动作）
        │
        ▼
开爪 → 预抓取 → 抓取 → 合爪 → 观察位 → 盒子上方 → 开爪 → 观察位 → 合爪
```

---

## 2. 在上游仓库里「增加」了什么

### 2.1 设计原则（推荐）

- **不修改** `rebotarm_bringup` / `rebotarmcontroller` 等上游包  
- 只新增 / 上传 **`rebot_visual_grasp` 一个包**  
- 原启动命令保持不变：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
```

- 腕部相机驱动 + 外参 TF 用**另一条指令**启动（见 §6）

相机→基座的 TF 不靠改 URDF bringup，而由本包 `wrist_camera.launch.py` 发布：

```text
gripper_end → camera_mount_link → camera_link
（光学系由 Orbbec 驱动继续发布）
```

外参数值在：`config/wrist_extrinsics.yaml`

### 2.2 本包内容

```text
src/rebot_visual_grasp/
├── config/wrist_extrinsics.yaml   # 腕部相机外参（静态 TF）
├── description/                   # 可选：支架 mesh / 显示用 xacro
├── launch/
│   ├── wrist_camera.launch.py     # 相机驱动 + TF（单独启动）
│   ├── grasp_yolo.launch.py
│   └── grasp_execute.launch.py
├── rebot_visual_grasp/            # Python 节点
└── docs/
```

| 模块 | 职责 |
|------|------|
| `wrist_camera` | Gemini2 驱动 + `gripper_end`→相机静态 TF |
| `grasp_yolo` | YOLOE → Seeed 6D → 发布位姿 |
| `grasp_execute` | 调用上游 `move_to_pose` / 夹爪执行 |

**不要**提交：Orbbec SDK 整仓、`*.pt` / `*.ts` / 无关 `*.onnx`。

---

## 3. 系统架构（坐标与话题）

```text
Gemini2 彩色/深度
        │
        ▼
   grasp_yolo（光学系 6D）
        │  TF: camera_color_optical_frame → base_link
        ▼
 /grasp_pose  /pregrasp_pose  /place_pose   （geometry_msgs/PoseStamped, frame=base_link）
        │
        ▼
   grasp_execute
        │
        ├─ /rebotarm/enable
        ├─ /rebotarm/gripper/open|close
        └─ /rebotarm/move_to_pose  (action)
```

关键几何默认（当前实现）：

- **抓取姿态**：Seeed 风格，接近方向沿相机视线（`grasp_approach_mode:=camera`）
- **预抓取**：沿工具轴后退（`pregrasp_mode:=tool_x`）
- **放置**：`/place_pose` 的 XYZ + 默认用观察位姿态（更稳）

可选竖直抓取（易 IK 失败，按需）：

```bash
pregrasp_mode:=world_z grasp_approach_mode:=world_z
```

---

## 4. 环境准备

### 4.1 硬件

- reBot Arm **B601-RS**
- CAN（常用 `can0`）
- 腕部 **Orbbec Gemini 2** + 打印支架
- Ubuntu 22.04 + ROS 2 Humble

### 4.2 软件依赖

1. **本仓库**（含 `rebot_visual_grasp`）已 `colcon build`  
2. **Orbbec ROS2 相机包**（建议独立 `~/ros2_ws`，不要塞进本仓）  
3. **Python 环境跑 YOLO**（推荐 conda `yolov8`）——**详细步骤见**  
   [`YOLO_SETUP_zh.md`](./YOLO_SETUP_zh.md)（安装 Ultralytics、CLIP、权重、如何用 conda 启动 `grasp_yolo`）

```bash
conda activate yolov8
pip install -U ultralytics
pip uninstall -y clip
pip install git+https://github.com/ultralytics/CLIP.git
```

权重示例：

- `yoloe-26s-seg.pt`
- `mobileclip2_b.ts`（`set_classes` 用；可放在 ultralytics 目录或 weights）

4. 每个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/rebotarm_ros2/install/setup.bash
source ~/ros2_ws/install/setup.bash   # 相机包所在工作空间
```

编译视觉包：

```bash
cd ~/rebotarm_ros2
colcon build --packages-select rebot_visual_grasp --symlink-install
source install/setup.bash
```

---

## 5. 一次性标定 / 调参（建议顺序）

他人复用时，**先完成本节，再跑日常抓取**。

### 5.1 确认腕部相机 TF（不改 bringup）

终端 A（原指令，可按你的硬件加 `model`/`channel`）：

```bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
# RS + CAN 示例：
# ros2 launch rebotarm_bringup bringup.launch.py model:=rs channel:=can0 use_rviz:=true
```

终端 B（本包：驱动 + 外参 TF）：

```bash
ros2 launch rebot_visual_grasp wrist_camera.launch.py
```

检查：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 topic hz /camera/color/image_raw
```

粗调外参：编辑 `rebot_visual_grasp/config/wrist_extrinsics.yaml` 后重启 `wrist_camera`。  
细调：仍可用 `align_mount` / `align_camera`，调完把数值写回该 YAML（与 xacro 显示用数值保持一致更佳）。

### 5.2 录制观察位（重力补偿）

观察位决定「看桌面」的起始位姿，建议每人/每台机录一次。

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger {}
ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger {}
# 手拖到合适观察位并稳住
ros2 run tf2_ros tf2_echo base_link end_link
# 记下 Translation 与 Quaternion(xyzw)
ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger {}
```

写入启动参数（示例）：

```bash
observation_x:=0.205 observation_y:=0.007 observation_z:=0.312 \
observation_qx:=-0.025 observation_qy:=0.434 observation_qz:=0.031 observation_qw:=0.900
```

`grasp_yolo` 与 `grasp_execute` **必须用同一组** observation，否则抓完回不去同一点。

### 5.3 高度安全余量

非官方加长爪时，桌面抓取建议：

- `grasp_z_offset_m:=0.03`（整体抬高）
- 必要时 `insertion_depth_m:=0`

---

## 6. 日常运行（可复现 checklist）

> 先停重力补偿，再启动会 `move_to_pose` 的节点。

### 终端 A — 机械臂（原仓库指令，不变）

```bash
cd ~/rebotarm_ros2 && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true
# 按硬件需要可加：model:=rs channel:=can0
```

### 终端 B — 腕部相机 + TF（仅本包）

```bash
source /opt/ros/humble/setup.bash
source ~/rebotarm_ros2/install/setup.bash
source ~/ros2_ws/install/setup.bash   # orbbec_camera 所在工作空间

ros2 launch rebot_visual_grasp wrist_camera.launch.py
# 若只想发 TF、相机另开：
# ros2 launch rebot_visual_grasp wrist_camera.launch.py start_camera_driver:=false
# ros2 launch orbbec_camera gemini2.launch.py
```

### 终端 C — YOLO 识别（详细见 [YOLO_SETUP_zh.md](./YOLO_SETUP_zh.md)）

```bash
cd ~/rebotarm_ros2
source /opt/ros/humble/setup.bash && source install/setup.bash && source ~/ros2_ws/install/setup.bash

/home/ubuntu/miniconda3/envs/yolov8/bin/python -m rebot_visual_grasp.grasp_yolo --ros-args \
  -p yolo_model:=/path/to/yoloe-26s-seg.pt \
  -p yolo_device:=0 \
  -p target_class:=pen \
  -p place_class:=box \
  -p conf_threshold:=0.4 \
  -p move_to_observation_on_start:=true \
  -p auto_publish_on_detect:=true
```

或：

```bash
ros2 launch rebot_visual_grasp grasp_yolo.launch.py \
  yolo_device:=0 target_class:=pen place_class:=box
```

成功标志：

- 窗口框出 `pen` / `box`
- 日志周期性出现 `Seeed 6D` 与 `Place 6D`
- 话题有数据：`/grasp_pose` `/pregrasp_pose` `/place_pose`

快捷键：`G` 冻结 · `R` 恢复 · `Q` 退出

### 终端 D — 执行

```bash
ros2 launch rebot_visual_grasp grasp_execute.launch.py
# 确认 C 已在发 6D 后：
ros2 service call /grasp_execute std_srvs/srv/Trigger {}
```

一键（启动 execute 并等待位姿后触发）：

```bash
bash $(ros2 pkg prefix rebot_visual_grasp)/share/rebot_visual_grasp/scripts/grasp_go.sh
# 或仓库内：
bash src/rebot_visual_grasp/scripts/grasp_go.sh
```

---

## 7. 常用参数

| 参数 | 默认 | 含义 |
|------|------|------|
| `target_class` | `pen` | 抓取目标类名（YOLOE 文本类） |
| `place_class` | `box` | 放置容器类名 |
| `conf_threshold` | `0.4` | 置信度阈值 |
| `pregrasp_offset_m` | `0.08` | 预抓取后退/抬高距离 |
| `pregrasp_mode` | `tool_x` | `tool_x`=沿工具轴；`world_z`=正上方 |
| `grasp_approach_mode` | `camera` | `camera`=沿视线；`world_z`=爪尖垂直桌面 |
| `grasp_z_offset_m` | `0.03` | 抓取点整体抬高 |
| `place_z_offset_m` | `0.05` | 相对盒子表面再抬高 |
| `yolo_device` | `cpu` | `cpu` / `0`（GPU）；`gpu` 会映射为 `0` |

---

## 8. 故障排查

| 现象 | 处理 |
|------|------|
| 只有笔或只有盒 | 看日志 `YOLO dets:`；降 `conf_threshold`；确认两类都在 `custom_classes` |
| `missing /grasp_pose` | YOLO 未发布；先等 `Seeed 6D` |
| `move_to_pose pregrasp failed` / `trajectory planning failed` | 多为姿态不可达；先用倾斜 `camera`+`tool_x`；检查 z 是否过低；用 `/rebotarm/move_to_pose_ik` 单测 pose |
| 能手拖到附近，但自动失败 | 手拖只保证大致 XYZ；自动要满足完整四元数姿态 |
| 开爪/合爪 timeout | 先单独测 `/rebotarm/gripper/open`；确认已 enable |
| 臂像失能 | 常因过低撞桌保护；增大 `grasp_z_offset_m`，`enable` 恢复 |
| `No module named 'clip'` | 重装 Ultralytics CLIP（见 §4.2） |
| CUDA / `yolo_device:=gpu` 报错 | 用 `yolo_device:=0` |

单测预抓取 IK：

```bash
ros2 topic echo /pregrasp_pose --once
ros2 service call /rebotarm/move_to_pose_ik rebotarm_msgs/srv/MoveToPoseIK "{target_pose: ...}"
```

---

## 9. 给贡献者 / 复用者的建议拆分

向 [reBotArmController_ROS2](https://github.com/Seeed-Projects/reBotArmController_ROS2) 贡献时：

- **推荐**：只新增 `src/rebot_visual_grasp/`，**零改动**上游 bringup/controller  
- 相机外参走 `config/wrist_extrinsics.yaml` + `wrist_camera.launch.py`

`.gitignore` 建议补充：

```gitignore
__pycache__/
*.pt
*.onnx
*.ts
weights/
```

---

## 10. 最小验收标准

- [ ] 原 `bringup.launch.py` 可独立启动，未改上游文件
- [ ] `wrist_camera.launch.py` 后 TF 能连到 `camera_color_optical_frame`
- [ ] 相机彩色/深度有频率
- [ ] 观察位可稳定到达，画面能同时看到目标与盒子
- [ ] 日志同时出现有效 `Seeed 6D` 与 `Place 6D`
- [ ] `/grasp_execute` 完整跑通抓取与放盒，无撞桌、无长时间失能

---

## 11. 相关入口

| 入口 | 命令 |
|------|------|
| 臂（上游） | `ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true` |
| 臂+腕部相机一条指令 | `ros2 launch rebot_visual_grasp bringup_with_camera.launch.py use_rviz:=true`（默认 `model:=rs channel:=can0`，RViz 带相机模型 + Orbbec 驱动） |
| 腕部相机+TF | `ros2 launch rebot_visual_grasp wrist_camera.launch.py` |
| 识别 | `ros2 launch rebot_visual_grasp grasp_yolo.launch.py` |
| 执行 | `ros2 launch rebot_visual_grasp grasp_execute.launch.py` |
| 一键执行 | `bash .../scripts/grasp_go.sh` |
| 调支架 | `ros2 launch rebot_visual_grasp align_mount.launch.py` |
| 调相机 | `ros2 launch rebot_visual_grasp align_camera.launch.py` |

上游控制 API 仍以官方 README / `API_zh.md` 为准；本包只消费其 `enable`、夹爪与 `move_to_pose`。
