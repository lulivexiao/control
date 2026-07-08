# 仿真器设计原理与规控组工作流

## 一、整体仿真器设计原理

### 1.1 设计目标

在不依赖任何硬件（实车、LiDAR、INS、CAN 总线）的条件下，闭环验证完整的 FSD 算法链路。

### 1.2 核心思路：替换传感器输入层

FSD 管线的外部输入边界是 3 个传感器话题。仿真器只需按相同格式、相同频率、相同话题名发布这 3 个话题，FSD 管线内部无需任何修改即可运行：

| # | 话题 | 类型 | 频率 | 发布者 | 对应真实传感器 |
|---|------|------|------|--------|---------------|
| 1 | `/sim/ground_truth` | `nav_msgs/Odometry` | 50Hz | vehicle_model（规控组） | 无（评价基准） |
| 2 | `/hesai/pandar` | `sensor_msgs/PointCloud2` | 10Hz | lidar_simulator（感知组） | LiDAR |
| 3 | `/cg410/odometry` | `nav_msgs/Odometry` | 20Hz | ins_simulator（定位建图组） | INS/组合导航 |
| 4 | `/localization/velocity` | `geometry_msgs/TwistStamped` | 50Hz | can_simulator（规控组） | CAN 总线轮速 |
| 5 | `/localization/pose` | `geometry_msgs/PoseStamped` | 50Hz | sim_bridge / localization_manager | 定位系统 |
| 6 | `/control/command` | `autoware_msgs/Command` | 50Hz | FSD controller_node | —（控制指令，FSD→仿真器） |
| 7 | `/system/mission_state` | `wuta_msgs/MissionState` | — | sim_mission_manager | 赛事状态机 |
| 8 | `/mapping/cone_map` | `wuta_msgs/ConeMap` | 5Hz | FSD cone_map_builder | —（建图结果） |

### 1.3 话题依赖关系

```
FSD controller_node                              vehicle_model
    │                                                │
    │ ⑥ /control/command                             │
    ├───────────────────────────────────────────────►│
    │                                                │
    │                                                ├── 自行车模型
    │                                                ├── 限幅转向角
    │                                                │
    │ ① /sim/ground_truth (Odometry)                │
    │◄───────────────────────────────────────────────┤
    │                     │                          │
    │                     │ ① 订阅                    │
    │                     ▼                          │
    │              can_simulator                     │
    │                     │                          │
    │ ④ /localization/velocity                      │
    │◄────────────────────┤                          │
    │                                                │
    │ ⑤ /localization/pose                          │
    │◄────── sim_bridge (模式A) / FSD 定位 (模式B)   │
    │                                                │
    │ ③ /cg410/odometry                             │
    │◄────── ins_simulator (定位建图组)               │
    │                                                │
    │ ② /hesai/pandar                                │
    │◄────── lidar_simulator (感知组)                 │
```

### 1.4 规控组在仿真器中的职责

规控组负责仿真器中与车辆运动和控制直接相关的 2 个模块：

| 模块 | 输入 | 输出 | 功能 |
|------|------|------|------|
| vehicle_model | `/control/command` | `/sim/ground_truth` | 自行车运动学模型，生成真值位姿 |
| can_simulator | `/sim/ground_truth` | `/localization/velocity` | 模拟 CAN 总线车速信号 |

---

## 二、规控组模块详解

### 2.1 vehicle_model — 车辆运动学模型

**文件**：`vehicle_model.py`

**模型**：前轮转向自行车运动学模型（Bicycle Model）

**状态量**：

```python
(x, y)     # 后轴中心位置 (m)
yaw        # 车身朝向角 (rad)
speed      # 纵向速度 (m/s)
angle      # 前轮目标转向角 (度，来自控制指令)
```

**核心运动学方程**（前向欧拉离散，dt = 0.02s @ 50Hz）：

```
x    += speed * cos(yaw) * dt
y    += speed * sin(yaw) * dt
yaw  += speed * tan(steer) / wheel_base * dt
```

**当前代码中的处理流程**：

```
on_command(msg: Command)
    ├── self.speed = msg.speed       # 直接赋值，m/s
    └── self.angle = msg.angle       # 直接赋值，度

step()  ← timer 每 20ms 触发
    ├── steer = clamp(radians(angle), ±max_steer)  # 限幅转向角 ±25°
    ├── x += speed * cos(yaw) * dt                  # 运动学更新
    ├── y += speed * sin(yaw) * dt
    ├── yaw += speed * tan(steer) / wheel_base * dt
    ├── 打包 Odometry
    │     ├── header.frame_id = 'map'
    │     ├── child_frame_id = 'base_link'
    │     ├── pose.position = (x, y, 0)
    │     ├── pose.orientation = quaternion_from_yaw(yaw)
    │     ├── twist.linear.x = speed
    │     └── twist.angular.z = speed * tan(steer) / L
    └── publish /sim/ground_truth
```

**Odometry 输出格式**：

```
nav_msgs/msg/Odometry
    header
        stamp      # 当前仿真时间
        frame_id   # "map"
    child_frame_id   # "base_link"
    pose
        pose
            position
                x    # 自车位置 X (m)
                y    # 自车位置 Y (m)
                z    # 0.0
            orientation
                x    # 0.0
                y    # 0.0
                z    # sin(yaw/2)
                w    # cos(yaw/2)
        covariance     # 未使用
    twist
        twist
            linear
                x    # 纵向速度 (m/s)
                y    # 0.0
                z    # 0.0
            angular
                x    # 0.0
                y    # 0.0
                z    # speed * tan(steer) / L (横摆角速度, rad/s)
        covariance     # 未使用
```

**话题映射**：

```
Subscribe: /control/command (autoware_msgs/Command)     ← #6
Publish:   /sim/ground_truth   (nav_msgs/Odometry)       → #1
```

### 2.2 can_simulator — CAN 车速模拟器

**文件**：`can_simulator.py`

**功能**：模拟实车 CAN 总线上报的轮速/车速信号，供 Pure Pursuit 等控制算法计算动态前视距离。

真车上：轮速传感器 → CAN 总线 → CAN 驱动 → `/localization/velocity`

仿真中：直接采用 `vehicle_model` 发布的仿真实际车速，**不加噪声**（实车 CAN 信号质量高，且 controller_node 内部已有滤波处理）。

**当前代码中的处理流程**：

```
ground_truth_callback(msg: Odometry)  ← 收到 /sim/ground_truth
    ├── 打包 TwistStamped
    │     ├── header.stamp = msg.header.stamp    # 保留原始时间戳
    │     ├── header.frame_id = 'base_link'
    │     ├── twist.linear.x  = msg.twist.twist.linear.x
    │     ├── twist.linear.y  = msg.twist.twist.linear.y
    │     ├── twist.linear.z  = msg.twist.twist.linear.z
    │     ├── twist.angular.x = msg.twist.twist.angular.x
    │     ├── twist.angular.y = msg.twist.twist.angular.y
    │     └── twist.angular.z = msg.twist.twist.angular.z
    └── publish /localization/velocity
```

**话题映射**：

```
Subscribe: /sim/ground_truth   (nav_msgs/Odometry)       ← #1
Publish:   /localization/velocity (geometry_msgs/TwistStamped) → #4
```

---

## 三、规控组完整工作流

### 3.1 一个控制周期内的数据流（50Hz = 20ms）

```
时刻     事件
────     ────
t0       FSD controller_node 完成控制计算
         │  发布 /control/command (speed, angle)
         ▼
t1       vehicle_model.step() 触发
         ├── steer = clamp(angle, ±25°)
         ├── x += speed * cos(yaw) * dt
         ├── y += speed * sin(yaw) * dt
         ├── yaw += speed * tan(steer) / L * dt
         ├── 打包所有 12 个 Odometry 字段
         └── 发布 /sim/ground_truth
         │
         ▼
t2       can_simulator.ground_truth_callback() 触发
         ├── 提取全部 6 个 twist 字段
         ├── 打包 TwistStamped，保留原始时间戳
         └── 发布 /localization/velocity
         │
         ▼
t3       FSD controller_node 收到 /localization/velocity
         ├── Pure Pursuit: 前视距离 = f(speed)
         ├── 计算横向跟踪误差 → 目标转向角
         └── 进入下一周期，计算控制指令
```

### 3.2 信息流转图

```
┌──────────────────────────────────────────────────────────┐
│                  规控组模块（仿真器侧）                      │
│                                                          │
│  /control/command     vehicle_model     /sim/ground_truth │
│  ───────────────►  ┌──────────────┐  ─────────────────►  │
│   (speed, angle)    │ 自行车模型    │   (Odometry)        │
│                     │ 限幅转向角    │                      │
│                     │ 运动学更新    │     ▼               │
│                     └──────────────┘  can_simulator      │
│                                       ┌──────────────┐   │
│                                       │ 提取 speed    │   │
│                                       │ 转发全部字段   │   │
│                                       └──────────────┘   │
│                                             │             │
│                                /localization/velocity     │
│  ◄─────────────────────────────                          │
│   (TwistStamped)                → FSD controller_node     │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### 3.3 各话题在管线中的角色

```
话题                           路径                         作用
────                           ────                         ────
/control/command        FSD → vehicle_model                控制指令下发
/sim/ground_truth       vehicle_model → 所有仿真器模块      真值基准（评价用）
/localization/velocity  can_simulator → FSD controller     实际车速反馈
```

---

## 四、关键设计决策

| 决策 | 当前实现 | 理由 |
|------|---------|------|
| 模型选择 | 运动学自行车模型 | 低速场景（FSG ≤ 15m/s）无侧滑假设成立；计算开销低 |
| 更新频率 | 50Hz（dt = 0.02s） | 匹配控制指令频率，前向欧拉在此步长下足够精确 |
| CAN 车速是否加噪声 | 不加噪声 | 实车 CAN 信号由轮速编码器直接反馈，质量高；controller 自身有滤波 |
| CAN 转发字段 | 复制全部 6 个 twist 字段 | 下游可能使用 angular.z（横摆角速度），完整传递减少依赖遗漏 |
| Odometry 输出完整性 | 全部 12 个字段显式赋值 | 避免下游收到隐式默认值时的歧义 |
| 转向角处理 | 度 → 弧度转换 + ±25° 限幅 | `msg.angle` 单位为度，tan() 需要弧度，限幅模拟机械限位 |
| 线程模型 | 单线程（无锁） | on_command 和 step 在同一线程上下文执行，无竞争条件 |
