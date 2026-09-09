# YOLO / YOLOE 接入教程（rebot_visual_grasp）

YOLO **不是**改上游 `rebotarm_bringup` 加进去的，而是本包节点 `grasp_yolo` 在 **Python 里调用 Ultralytics**，订阅相机图，算出 6D 后再发 ROS 话题。

```text
Orbbec 彩色/深度话题
        │
        ▼
grasp_yolo（本包节点）
        │  内部: ultralytics YOLOE.predict()
        │  + 深度 → 相机系 6D → TF 到 base_link
        ▼
/grasp_pose  /pregrasp_pose  /place_pose
        │
        ▼
grasp_execute（只消费位姿，不跑 YOLO）
```

---

## 1. 你需要准备什么

| 项 | 说明 |
|----|------|
| 本包 | `src/rebot_visual_grasp` 已 `colcon build` |
| Python 环境 | 建议独立 conda（不要硬绑系统 `/usr/bin/python3`） |
| Ultralytics | ≥8.4，带 **YOLOE** |
| CLIP | **Ultralytics 版**（`set_classes` 开集类别需要） |
| 权重 | `yoloe-26s-seg.pt` + `mobileclip2_b.ts` |

权重、`.pt`、`.ts` **不要提交进 git**（本仓 `.gitignore` 已忽略）。

---

## 2. 安装 YOLO 环境（一次性）

```bash
# 若还没有环境
conda create -n yolov8 python=3.10 -y
conda activate yolov8

pip install -U ultralytics

# 关键：不要用 PyPI 里的 clip；YOLOE 要 Ultralytics CLIP
pip uninstall -y clip
pip install git+https://github.com/ultralytics/CLIP.git
```

验证：

```bash
python -c "from ultralytics import YOLOE; print('YOLOE OK')"
```

下载权重（示例，路径按你机器改）：

```bash
mkdir -p ~/ultralytics-main && cd ~/ultralytics-main
# yoloe-26s-seg.pt —— 可用 ultralytics 自动下载，或手动放到该目录
wget -c https://github.com/ultralytics/assets/releases/download/v8.4.0/mobileclip2_b.ts
```

`grasp_yolo` 调 `set_classes(["pen","box"])` 时会找 `mobileclip2_b.ts`（本包 `yolo_loader.py` 也会在常见目录里帮你定位）。

---

## 3. 本包里和 YOLO 相关的文件

| 文件 | 作用 |
|------|------|
| `rebot_visual_grasp/grasp_yolo.py` | ROS 节点：收图、推理、发 6D |
| `rebot_visual_grasp/yolo_loader.py` | 加载 YOLOE、`set_classes`、device 映射 |
| `rebot_visual_grasp/ordinary_grasp.py` | Seeed 风格 OBB/深度 → 抓取几何 |
| `launch/grasp_yolo.launch.py` | 启动参数封装 |

**不需要**改 `rebotarmcontroller` / `bringup` 才能「加 YOLO」。

---

## 4. 启动顺序（YOLO 接进整机）

先保证臂 + 相机 TF/图像正常，再开 YOLO。

### 终端 A — 臂（或一条指令带相机）

```bash
source /opt/ros/humble/setup.bash
source ~/rebotarm_ros2/install/setup.bash

# 原指令
ros2 launch rebotarm_bringup bringup.launch.py use_rviz:=true

# 或一条指令：臂 + 腕部相机
# ros2 launch rebot_visual_grasp bringup_with_camera.launch.py use_rviz:=true
```

若 A 没用 `bringup_with_camera`，再开相机：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rebot_visual_grasp wrist_camera.launch.py
```

### 终端 B — YOLO 识别（必须用装了 ultralytics 的 Python）

```bash
cd ~/rebotarm_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/ros2_ws/install/setup.bash

# 推荐：直接用 conda 里的解释器跑模块（避免 ros2 launch 用到系统 python）
/home/ubuntu/miniconda3/envs/yolov8/bin/python -m rebot_visual_grasp.grasp_yolo --ros-args \
  -p yolo_model:=/home/ubuntu/ultralytics-main/yoloe-26s-seg.pt \
  -p yolo_device:=0 \
  -p target_class:=pen \
  -p place_class:=box \
  -p conf_threshold:=0.4 \
  -p move_to_observation_on_start:=true \
  -p auto_publish_on_detect:=true
```

也可用 launch（注意：默认可能仍是系统 python，若缺 ultralytics 会失败）：

```bash
ros2 launch rebot_visual_grasp grasp_yolo.launch.py \
  yolo_model:=/home/ubuntu/ultralytics-main/yoloe-26s-seg.pt \
  yolo_device:=0 \
  target_class:=pen \
  place_class:=box
```

成功标志：

- 弹出 `grasp_yolo` 窗口，框出目标
- 日志：`YOLO custom classes: [...]`、`Seeed 6D ...`、`Place 6D ...`
- `ros2 topic echo /grasp_pose --once` 有数据

### 终端 C — 执行（不跑 YOLO，只订阅位姿）

```bash
ros2 launch rebot_visual_grasp grasp_execute.launch.py
ros2 service call /grasp_execute std_srvs/srv/Trigger {}
```

---

## 5. 关键参数（换检测目标）

| 参数 | 含义 | 示例 |
|------|------|------|
| `yolo_model` | 权重路径 | `.../yoloe-26s-seg.pt` |
| `yolo_device` | `cpu` 或 `0`（GPU）；不要写裸 `gpu` | `0` |
| `target_class` | 抓取类（开集文本） | `pen` / `banana` |
| `place_class` | 放置容器类 | `box` |
| `custom_classes` | YOLOE `set_classes` 列表 | launch 默认 `["pen","box"]` |
| `conf_threshold` | 置信度 | `0.4` |
| `use_yoloe` | 是否按 YOLOE 加载 | `true` |

换目标时：

```bash
-p target_class:="yellow banana" -p place_class:=box
```

`target_class` / `place_class` 若不在 `custom_classes` 里，节点启动时会自动补进列表。

---

## 6. 为什么常用「conda python -m」而不是纯 `ros2 run`

- `grasp_yolo` 依赖 **torch / ultralytics**，一般装在 conda  
- `ros2 run` / 部分 launch 用的是系统 Python，容易 `No module named ultralytics`  
- 用 conda 解释器执行 `-m rebot_visual_grasp.grasp_yolo`，同时 `source install/setup.bash`，即可同时用到 ROS 消息与 YOLO

---

## 7. 常见问题

| 现象 | 处理 |
|------|------|
| `No module named ultralytics` | 用 conda 的 python；或把该环境注册进 ROS 所用解释器 |
| `No module named 'clip'` | 重装 Ultralytics CLIP（§2） |
| `set_classes` / MobileCLIP 报错 | 下载 `mobileclip2_b.ts` 到权重旁或 CWD |
| `yolo_device:=gpu` 无效 | 改成 `0` 或 `cpu` |
| 只有 pen 或只有 box | 降 `conf_threshold`；看日志 `YOLO dets:` |
| 有框但没有 6D | 深度无效 / TF 未通；先查 `wrist_camera` 与 `tf2_echo` |

---

## 8. 和 Wiki Demo 的关系

Seeed Wiki 视觉抓取 Demo 常在 SDK 脚本里直接调 YOLO + `hand_eye.npz`。  
本方案把同一类能力拆成：

1. 相机 + TF → ROS  
2. `grasp_yolo` → ROS 位姿话题  
3. `grasp_execute` → 上游 `move_to_pose`

因此「加 YOLO」= **装 Ultralytics + 跑本包 `grasp_yolo` 节点**，不必改原仓库启动指令。
