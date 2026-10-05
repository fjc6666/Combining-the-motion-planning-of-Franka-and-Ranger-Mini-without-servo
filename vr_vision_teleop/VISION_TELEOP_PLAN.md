# 视觉遥操作改造方案

> 本文档供下一个 Claude 实例阅读，包含完整背景、架构决策和实现任务。

---

## 项目背景

原项目是 **ROS2 + MoveIt Servo** 的 VR 伺服抓取系统，底层控制链路已完整可用。
目标：将 VR 输入替换为 **摄像头 + MediaPipe**，实现视觉遥操作：
- 右手臂姿态 → Franka FR3 机械臂末端位置/姿态
- 右手手指捏合 → 夹爪开合
- 左手手势 → Ranger Mini v2 底盘前进/后退

---

## 工作空间结构（关键部分）

```
mobile_manipulation_moveit_servo_ws/
└── src/
    ├── vr_vision_teleop/          ← 主要改造包（ament_cmake 混合包）
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── scripts/
    │   │   └── vr_bridge.py       ← 原VR模拟节点（保留，新节点并列）
    │   ├── src/
    │   │   ├── vr_servo_bridge.cpp  ← 核心C++桥接节点（不改动！）
    │   │   └── robot_planner.cpp
    │   ├── launch/
    │   │   └── start_planner.launch.py
    │   └── config/
    │       └── fr3.srdf
    ├── my_composite_robot_config/
    │   └── config/
    │       └── servo_config.yaml   ← MoveIt Servo配置（不改动）
    └── composite_robot_description/
        └── urdf/mobile_manipulator.urdf.xacro
```

---

## 现有控制链路（不改动）

```
[输入源] ──→ /vr_target_pose (PoseStamped, frame: base_footprint)
                    ↓
         vr_servo_bridge (C++, P控制器, Kp=2.5)
                    ↓
         /servo_node/delta_twist_cmds (TwistStamped)
                    ↓
         MoveIt Servo Node
                    ↓
         /franka_arm_controller/joint_trajectory
                    ↓
         Franka FR3
```

**核心约束**：`/vr_target_pose` 必须用 `base_footprint` 坐标系，位置单位为米。

MoveIt Servo 配置关键参数（`servo_config.yaml`）：
- `move_group_name: franka_arm`
- `ee_frame_name: fr3_hand_tcp`
- `planning_frame: base_footprint`
- `linear_scale: 0.4`
- `incoming_command_timeout: 0.2`（200ms 无指令则急停）

---

## 需要新增的文件

### 1. `scripts/mediapipe_arm_node.py` — 核心感知节点

**功能**：读摄像头 → MediaPipe → 发布机械臂目标位姿 + 底盘速度指令

**依赖**：`mediapipe`, `opencv-python`（pip 安装，非 ROS 包）

**MediaPipe 模型选择**：使用 `MediaPipe Holistic`（同时检测 Pose + Hands，一次推理）

**地标使用**：
| 地标 | MediaPipe ID | 用途 |
|------|-------------|------|
| 右肩 | pose[12] | 人臂基准点 |
| 右肘 | pose[14] | 计算前臂方向 |
| 右腕 | pose[16] | 末端目标位置 |
| 右手拇指尖 | right_hand[4] | 夹爪控制 |
| 右手食指尖 | right_hand[8] | 夹爪控制 |
| 左手 | left_hand[0-20] | 底盘手势 |

**手臂映射算法**：
```
1. 提取右腕相对右肩的向量 v = wrist_pos - shoulder_pos（归一化坐标系）
2. 对 v 做低通滤波（指数移动平均，alpha=0.7）
3. 缩放映射到机器人工作空间：
   robot_pos = workspace_center + v * workspace_scale
   - workspace_center = [0.45, 0.0, 0.5]（base_footprint坐标系，单位m）
   - workspace_scale = 0.5（人臂运动范围→机器人工作空间）
4. 姿态估算：由 肩→肘 和 肘→腕 两向量叉积得到手掌法向量，转换为四元数
5. 发布 /vr_target_pose (PoseStamped, frame_id="base_footprint")
```

**坐标系转换说明**：
- MediaPipe 输出归一化像素坐标 (x∈[0,1], y∈[0,1])，z 为相对深度
- 人站在摄像头前，MediaPipe x轴对应机器人 y轴（左右），y轴对应机器人 z轴（上下），z轴对应机器人 x轴（前后）
- 需要在 `teleop_params.yaml` 中提供轴映射和翻转参数

**夹爪控制**：
```
pinch_distance = ||right_hand[4] - right_hand[8]||（归一化距离）
pinch_ratio = clamp((pinch_distance - 0.03) / 0.15, 0, 1)
→ 发布 /gripper_command (Float64, 0.0=全闭, 0.08=全开)
```

**底盘手势检测**（左手）：
```
检测5根手指是否伸展（指尖y坐标 < 对应掌根关节y坐标）
伸展指数 = 伸展手指数量
- 5根伸展 → forward，线速度 = +max_linear_vel
- 0根伸展（握拳） → backward，线速度 = -max_linear_vel
- 其他 → 停止
→ 发布 /gesture_cmd_vel (Twist)
```

**发布频率**：30Hz（摄像头帧率）

**置信度安全门**：
- pose_landmark confidence < 0.6 → 停止发布 /vr_target_pose（Servo 看门狗 200ms 后急停）
- hand landmark confidence < 0.5 → 发布零速度到 /gesture_cmd_vel

**发布的话题**：
- `/vr_target_pose` (geometry_msgs/PoseStamped)
- `/gesture_cmd_vel` (geometry_msgs/Twist)
- `/gripper_command` (std_msgs/Float64)

### 2. `scripts/cmd_vel_filter.py` — 底盘速度安全过滤

订阅 `/gesture_cmd_vel`，经过限幅后发布到 `/cmd_vel`。

参数：
- `max_linear_vel: 0.3` (m/s)
- `max_angular_vel: 0.0`（手势控制只做直线运动）

### 3. `config/teleop_params.yaml` — 可调参数

```yaml
mediapipe_arm_node:
  ros__parameters:
    camera_index: 0                    # 摄像头设备号
    workspace_center: [0.45, 0.0, 0.5] # 机器人工作空间中心(base_footprint坐标系，单位m)
    workspace_scale: 0.5               # 人臂运动→机器人工作空间缩放比
    smoothing_alpha: 0.7               # 低通滤波系数(越大越跟手，越小越平滑)
    pose_confidence_threshold: 0.6     # 低于此值停止发布
    hand_confidence_threshold: 0.5
    publish_rate: 30.0                 # Hz
    # 坐标轴映射（MediaPipe → base_footprint）
    axis_map: [2, 0, 1]               # MediaPipe [x,y,z] → robot [axis_map[0], axis_map[1], axis_map[2]]
    axis_flip: [1, -1, -1]            # 各轴翻转符号

cmd_vel_filter:
  ros__parameters:
    max_linear_vel: 0.3
```

### 4. `launch/vision_teleop.launch.py` — 一键启动

启动顺序：
1. `my_composite_robot_config` 的 `servo.launch.py`（启动 MoveIt Servo）
2. `vr_servo_bridge` 可执行文件（现有C++节点）
3. `mediapipe_arm_node.py`（新Python节点，加载 teleop_params.yaml）
4. `cmd_vel_filter.py`（新Python节点）

### 5. 修改 `CMakeLists.txt`

在现有 `install(PROGRAMS scripts/vr_bridge.py ...)` 后追加：
```cmake
install(PROGRAMS
  scripts/mediapipe_arm_node.py
  scripts/cmd_vel_filter.py
  DESTINATION lib/${PROJECT_NAME}
)
```

### 6. 修改 `package.xml`

追加依赖：
```xml
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
```
（`mediapipe` 和 `opencv` 是 pip 包，不在 package.xml 中声明）

---

## 夹爪控制接口说明

Franka 夹爪通过 Action Server 控制：
- Action：`/fr3_gripper/gripper_action` (control_msgs/GripperCommand)
- 或简单模式：直接发布 `/franka_gripper/move` goal

`mediapipe_arm_node.py` 发布 `/gripper_command` (Float64, 0~0.08m 表示开合宽度)，
需要一个轻量适配节点或在 `mediapipe_arm_node.py` 内直接创建 Action Client。

**推荐**：在 `mediapipe_arm_node.py` 内集成 GripperCommand Action Client，
捏合检测到变化超过阈值（delta > 0.01m）时才发送新 goal，避免频繁调用。

---

## 实现优先级

1. **先实现** `mediapipe_arm_node.py` 的手臂位姿部分 + `/vr_target_pose` 发布（可立即在 Gazebo 中测试）
2. **再实现** 底盘手势控制（`/gesture_cmd_vel` + `cmd_vel_filter.py`）
3. **最后实现** 夹爪控制（需要 Action Client，稍复杂）
4. 调参：`workspace_scale`、`smoothing_alpha`、`Kp` 根据实测调整

---

## 测试方法

```bash
# 终端1：启动 Gazebo + 底层控制
ros2 launch my_composite_robot_config bringup_gazebo.launch.py

# 终端2：启动视觉遥操作（包含 Servo + 新节点）
ros2 launch vr_vision_teleop vision_teleop.launch.py

# 调试：查看末端当前位置
ros2 topic echo /vr_target_pose

# 调试：查看 Servo 状态
ros2 topic echo /servo_node/status
```

---

## 注意事项

1. MediaPipe Holistic 在 CPU 上约 15-30ms/帧，30Hz 可达到。
2. `vr_servo_bridge.cpp` 的 Kp=2.5 可能需要调小（因为 VR 是绝对位置，摄像头映射可能有漂移）。
3. MediaPipe 坐标的 z 轴深度在单目摄像头下不准确，建议初期只用 x/y 做平面控制，z 轴固定或用肩-腕距离估算。
4. 注意 `incoming_command_timeout: 0.2s`，节点启动到 MediaPipe 初始化期间会有短暂急停，属正常现象。
