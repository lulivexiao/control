# 输出数据类型定义

## 1. `/sim/ground_truth`

| 项目 | 内容 |
|------|------|
| **发布者** | `vehicle_model` |
| **类型** | `nav_msgs/Odometry` |
| **频率** | 50Hz |

```
nav_msgs/Odometry
┣━ header
┃   ┣━ stamp          — 仿真时间
┃   ┗━ frame_id       — "map"
┣━ child_frame_id     — "base_link"
┣━ pose
┃   ┗━ pose
┃       ┣━ position
┃       ┃   ┣━ x     — 自车位置 X (m)
┃       ┃   ┣━ y     — 自车位置 Y (m)
┃       ┃   ┗━ z     — 0.0
┃       ┗━ orientation
┃           ┣━ x     — 0.0
┃           ┣━ y     — 0.0
┃           ┣━ z     — sin(yaw/2)
┃           ┗━ w     — cos(yaw/2)
┃   ┗━ covariance    — 未使用
┗━ twist
    ┗━ twist
        ┣━ linear
        ┃   ┣━ x     — 纵向速度 (m/s)
        ┃   ┣━ y     — 0.0
        ┃   ┗━ z     — 0.0
        ┗━ angular
            ┣━ x     — 0.0
            ┣━ y     — 0.0
            ┗━ z     — 横摆角速度 (rad/s)
    ┗━ covariance    — 未使用
```

### 字段对应关系

| Odometry 字段 | 来源 | 说明 |
|---------------|------|------|
| `pose.position.x` | `self.x` | 后轴中心 x 坐标（m） |
| `pose.position.y` | `self.y` | 后轴中心 y 坐标（m） |
| `pose.orientation.z` | `sin(yaw/2)` | 朝向四元数 z 分量 |
| `pose.orientation.w` | `cos(yaw/2)` | 朝向四元数 w 分量 |
| `twist.linear.x` | `self.speed` | 纵向速度（m/s） |
| `twist.angular.z` | `speed * tan(steer) / L` | 横摆角速度（rad/s） |

---

## 2. `/localization/velocity`

| 项目 | 内容 |
|------|------|
| **发布者** | `can_simulator` |
| **类型** | `geometry_msgs/TwistStamped` |
| **频率** | 50Hz |

```
geometry_msgs/TwistStamped
┣━ header
┃   ┣━ stamp          — 继承自 /sim/ground_truth 时间戳
┃   ┗━ frame_id       — "base_link"
┗━ twist
    ┣━ linear
    ┃   ┣━ x     — msg.twist.twist.linear.x   (纵向速度, m/s)
    ┃   ┣━ y     — msg.twist.twist.linear.y
    ┃   ┗━ z     — msg.twist.twist.linear.z
    ┗━ angular
        ┣━ x     — msg.twist.twist.angular.x
        ┣━ y     — msg.twist.twist.angular.y
        ┗━ z     — msg.twist.twist.angular.z   (横摆角速度, rad/s)
```

### 说明

- **数据来源**：完整复制 `Odometry.twist` 的 6 个字段
- **时间戳**：保留 `/sim/ground_truth` 的原始时间戳，不做改写
- **坐标系**：`base_link`
- **噪声**：不加噪声（实车 CAN 信号质量高，controller 自身有滤波）

---

## 3. 输入数据类型（参考）

| 话题 | 类型 | 接收者 | 字段 |
|------|------|--------|------|
| `/control/command` | `autoware_msgs/Command` | `vehicle_model` | `speed`（m/s）, `angle`（度） |

---

## 4. ROS 2 消息依赖

运行时需要安装的消息包：

```
nav_msgs            — Odometry
geometry_msgs       — TwistStamped
autoware_msgs       — Command
```
