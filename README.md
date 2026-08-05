# 点到点飞行工程（A → B）

本工程从以下两个接管后的仓库整理而来：

- `yeyulun16-dot/26-fly`：Orange Pi + ROS 2 上位机
- `yeyulun16-dot/26-flight-controller`：STM32F407 飞控

目标流程：人工完成上电、检查与解锁准备后，无人机以 A 点为起点，垂直爬升到巡航高度，水平飞向 B 点，并根据配置在 B 点持续悬停或下降到近地高度。默认采用 **到 B 后持续悬停**，不会自动锁桨。

> 实机飞行有伤人和损坏设备的风险。第一次调试必须拆桨验证指令，再进行限高、限速、系留测试；始终保留遥控器急停/接管能力。

准备交接或首次接手本工程时，请先阅读 [项目交接说明](HANDOVER.md)。

## 目录

```text
p_to_p_flight/
├── onboard_ws/                 # Orange Pi ROS 2 Humble 工作区
│   └── src/
│       ├── p_to_p_mission/     # 新整理的 A→B 状态机、配置和总启动文件
│       ├── pid_control_pkg/    # 地图位置误差 → 速度指令
│       ├── uart_to_stm32/      # 机体系转换、串口协议、上位机看门狗
│       ├── serial_comm/        # 0xAA 0xFF 协议收发
│       ├── my_carto_pkg/       # Cartographer 定位
│       ├── bluesea2/           # 平面激光雷达
│       └── laser_array_pkg/    # 可选面阵激光驱动（当前主启动不使用）
├── flight_controller/          # STM32F407 Keil 工程
├── scripts/                    # 构建、启动、任务操作脚本
├── docs/                       # 硬件适配、协议和试飞检查表
└── THIRD_PARTY_NOTICES.md
```

## 控制链路

```text
平面雷达 → Cartographer → map→laser_link 位姿
光流测距 → 飞控 → UART6 ───────→ /height
                                  ↓
p_to_p_mission → /target_position → position_pid_controller
                                           ↓ /target_velocity（map系）
                                   uart_to_stm32（转机体系）
                                           ↓ 0x31 串口帧
                                   STM32F407 速度控制
```

所有位置单位为 `cm`，速度为 `cm/s`，航向为 `deg`。飞控机体系按原工程约定：X 向前、Y 向左、Z 向上。

## 已做的关键整理

1. 新增可配置状态机：`IDLE → TAKEOFF → GOTO_B → HOLD_B/DESCEND_B`。
2. 默认在启动服务被人工调用时记录当前位置为 A，避免依赖固定地图原点。
3. B 点支持机体系相对坐标、地图相对坐标和地图绝对坐标。
4. PID 的地图系 XY 速度在串口发送前根据实时 yaw 转换为机体系。
5. 上位机 0.30 秒收不到新 PID 指令时发送三次零速度并关闭转发。
6. 飞控 0.30 秒收不到 `0x31` 时把 XYZ/Yaw 速度归零。
7. 飞控新增 `0x66 + 0x06` 任务结束帧处理，避免保留最后一条速度。
8. 到 B 默认持续闭环，而不是到点后立即切断速度输出。
9. 垂直 PID 的上升和下降速度统一受 `max_vertical_velocity` 对称限幅。
10. PID 直接检查测高和动态 TF 时间戳；超时先发布一次零速度，随后停止刷新指令，由串口看门狗关闭转发。
11. 新增 `0x67` 任务使能协议，替代原工程对触摸屏准备码和旧 `0xA2` 连接帧的隐式依赖；同时保留原机遥控兼容启动，未收到使能帧时必须先确认 CH6 低位，再拨到高位才会切模式并解锁。

## 默认硬件参数

| 项目 | 默认值 | 修改位置 |
|---|---:|---|
| Orange Pi 系统 | Ubuntu 22.04 + ROS 2 Humble | 部署环境 |
| 飞控 | STM32F407 | `flight_controller/ProjectSTM32F407` |
| 飞控串口 | `/dev/ttyS6`, 921600 8N1 | `p_to_p.yaml` |
| 飞控/光流高度 | UART6 的 `/height`（cm） | `p_to_p.yaml` |
| 平面雷达 | `/dev/ttyS4`, 921600 | `bluesea2/src/bluesea-ros2/params/uart_lidar.yaml` |
| 定位坐标 | `map → laser_link` | `p_to_p.yaml`、URDF |
| 默认 B 点 | 起飞机头前 50 cm、左 50 cm | `b_offset_x/y_cm` |
| 默认巡航高度 | 60 cm | `cruise_height_cm` |
| 默认末端行为 | `hold` | `terminal_behavior` |

硬件接口不同只需先修改 [硬件适配说明](docs/HARDWARE_ADAPTATION.md) 中列出的集中参数。

## 配置 A、B

编辑：

```text
onboard_ws/src/p_to_p_mission/config/p_to_p.yaml
```

推荐首次使用：

```yaml
target_mode: relative_body
b_offset_x_cm: 50.0
b_offset_y_cm: 50.0
cruise_height_cm: 60.0
terminal_behavior: hold
```

`relative_body` 在任务启动瞬间记录 A 点和航向：

- `b_offset_x_cm > 0`：向机头前方
- `b_offset_y_cm > 0`：向机体左侧

现场坐标已经标定时，可改为：

```yaml
target_mode: absolute
b_x_cm: 250.0
b_y_cm: -250.0
```

## Orange Pi 构建

```bash
cd ~/p_to_p_flight
chmod +x scripts/*.sh
./scripts/build_onboard.sh
```

脚本会先执行不依赖 ROS 的端到端仿真与安全契约检查，再使用 `/opt/ros/humble/setup.bash`、`rosdep` 和 `colcon`。也可以单独执行：

```bash
python3 scripts/verify_offline.py
```

Cartographer、Boost、Eigen 等缺失依赖需在联网时由 `rosdep` 安装。

## 飞控编译与下载

Keil 工程：

```text
flight_controller/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx
```

使用与原无人机一致的 Keil/ARMCC 环境编译并通过 J-Link/ST-Link 下载。首次编译前确认目标芯片、晶振、PWM 输出、遥控器通道和串口管脚没有因硬件版本变化而不同。

## 启动与操作

1. 无桨状态完成飞控、遥控器、雷达和串口检查。
2. 启动上位机：

```bash
cd ~/p_to_p_flight
./scripts/run_onboard.sh
```

需要临时使用另一份平面雷达参数时：

```bash
./scripts/run_onboard.sh lidar_params_file:=/绝对路径/uart_lidar.yaml
```

3. 确认 `/scan`、`/height`、TF 和串口正常；必须抬高机体确认 `/height` 随实际高度变化：

```bash
./scripts/preflight_check.sh
```

4. 保持遥控器 CH6 在低位，人工触发上位机任务准备：

```bash
./scripts/start_mission.sh
```

5. 确认日志显示 `0x67 enable` 已发送、A/B 目标正确、周围安全后，再把 CH6 从低位拨到高位。飞控随后按原流程切换模式、解锁，等待约 2 秒后接受速度目标。为兼容原机，上位机未发送使能帧时也可在确认过 CH6 低位后人工拨高启动，但正式点到点任务仍推荐先完成 `0x67 enable` 门控。
6. 观察状态：

```bash
ros2 topic echo /p_to_p/state
```

7. 需要中止路线但保持飞行时：

```bash
./scripts/abort_hold.sh
```

8. 仅在已经落地或高度低于配置阈值、且定位与高度数据仍新鲜时停止控制输出：

```bash
./scripts/stop_output_on_ground.sh
```

`stop_output` 不负责自动锁桨，落地后仍需按飞控/遥控器流程人工锁桨。

将 CH6 拨回低位会立即让飞控退出外部速度门控并把目标速度置零，但不会自动锁桨；仍须保留并验证原机的人工接管/锁桨流程。

若状态进入 `FAULT_HOLD`，先恢复定位和测高，再调用 `abort_hold.sh` 重新启用速度链路并保持当前位置；不要直接重新启动路线。确认悬停稳定后，人工处置或重启节点重新开始任务。

## 分阶段试飞

严格按 [试飞检查表](docs/FLIGHT_TEST_CHECKLIST.md) 执行。最少分为：

1. 拆桨通信测试：验证目标、PID、机体系转换、串口帧和 0.30 秒看门狗。
2. 单轴方向测试：低推力/受约束验证“X 前、Y 左、Z 上”和 yaw 正方向。
3. 低高度小位移：60 cm 高、前飞 50 cm、B 点悬停。
4. 增大距离：首次以默认 50/50 cm 验证方向后，再逐步放大到现场 A→B。
5. 最后才启用 `terminal_behavior: land`，且先验证下降方向和测高稳定性。

## 当前边界

- 不包含自动锁桨；这是有意保留的人工安全动作。
- `land` 是坐标/测高下降，不是视觉精准降落。
- Cartographer 原点与机体安装外参必须现场标定。
- 当前没有独立面阵激光；高度依赖光流模块测距经飞控回传的 `/height`。若该值为 0、冻结或方向错误，不得启动任务。
- 原工程中的机械臂、视觉识别、撒药、盘点等题目功能没有加入本精简工程。
