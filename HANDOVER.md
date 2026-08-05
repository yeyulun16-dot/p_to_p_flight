# 点到点飞行工程交接说明

交接版本：`handover-2026.08.03`  
交接目录：`p_to_p_flight/`  
接管账号：`yeyulun16-dot`

## 1. 交接结论

该目录是从原飞控仓库和上位机题目仓库中整理出的独立 A→B 点到点飞行工程，包含：

- Orange Pi / ROS 2 Humble 上位机工作区；
- STM32F407 / Keil 飞控工程；
- A→起飞→B→悬停/下降任务状态机；
- 构建、运行、任务操作和检查脚本；
- 上位机与飞控两级速度看门狗；
- 离线仿真、安全契约验证和试飞检查表。

当前版本已经完成源码整理、离线测试和交付目录哈希核对，但**尚未在实际 Orange Pi ROS 2 Humble、Keil/ARMCC 和无人机硬件上完成最终构建与飞行验收**。接收人不得把“离线验证通过”理解为“可以直接装桨飞行”。

## 2. 来源基线

| 内容 | 接管仓库 | 基线提交 |
|---|---|---|
| 上位机与题目代码 | `https://github.com/yeyulun16-dot/26-fly` | `56da812537678a95de4be293ae2ea8bcc7c56a9b` |
| STM32F407 飞控 | `https://github.com/yeyulun16-dot/26-flight-controller` | `fe2e78455aa48c45d190097838c67b1ae57e151b` |

更早的上游来源为 `GGbond-dot/26_fly` 和 `GGbond-dot/26--------`。详见 `THIRD_PARTY_NOTICES.md`。

## 3. 核心目录

```text
p_to_p_flight/
├── onboard_ws/src/                 ROS 2 Humble 工作区
│   ├── p_to_p_mission/             A→B 状态机、总启动、统一配置
│   ├── pid_control_pkg/            位置误差到速度指令
│   ├── uart_to_stm32/              坐标转换、任务门控、串口协议
│   ├── serial_comm/                AA FF 协议收发
│   ├── my_carto_pkg/               Cartographer 定位
│   ├── bluesea2/                   平面雷达驱动
│   └── laser_array_pkg/            可选面阵测高驱动（当前主启动不使用）
├── flight_controller/              STM32F407 Keil 工程
├── scripts/                        构建、运行、任务和验证脚本
├── docs/                           硬件适配与试飞检查
├── README.md                       使用说明
└── HANDOVER.md                     本交接说明
```

## 4. 默认硬件和坐标约定

| 接口 | 默认值 |
|---|---|
| Orange Pi → STM32 | `/dev/ttyS6`, 921600 8N1 |
| BlueSea 平面雷达 | `/dev/ttyS4`, 921600 |
| 飞控/光流高度 | UART6 回传 `/height`，单位 cm |
| 地图/机体 TF | `map → laser_link` |
| 机体系 | X 前、Y 左、Z 上 |
| yaw | 逆时针为正，单位 deg |
| 位置/速度 | cm、cm/s |

飞行参数集中在：

```text
onboard_ws/src/p_to_p_mission/config/p_to_p.yaml
```

本交接版本的 B 点已设置为起飞机头前方 50 cm、左侧 50 cm，巡航高度 60 cm，到 B 后持续悬停；这是首次实机验证参数。若需向右飞，必须将 `b_offset_y_cm` 改为负值。

## 5. 启动与人工授权顺序

```bash
cd ~/p_to_p_flight
chmod +x scripts/*.sh
./scripts/build_onboard.sh
./scripts/run_onboard.sh
./scripts/preflight_check.sh
```

任务启动顺序：

1. 拆桨或完成安全隔离，保持遥控器 CH6 低位。
2. 确认 `/scan`、测高、TF 和串口正常。
3. 执行 `./scripts/start_mission.sh`。
4. 检查 A/B 目标和日志中的 `0x67 enable`。
5. 周围安全后，人工把 CH6 拨到高位；飞控才会切模式、解锁并在约 2 秒后接受速度目标。
6. 中止路线并保持当前位置：`./scripts/abort_hold.sh`。
7. 仅在落地、数据新鲜且高度位于安全阈值内时执行 `./scripts/stop_output_on_ground.sh`，随后人工锁桨。

将 CH6 拨回低位会退出外部速度门控并把目标速度归零，不会自动锁桨。

## 6. ROS 接口

| 接口 | 作用 |
|---|---|
| `/p_to_p/start` | 记录当前 A，生成 B，开始起飞任务 |
| `/p_to_p/abort_hold` | 中止路线，锁定当前位姿悬停；也用于故障恢复后的人工重新使能 |
| `/p_to_p/stop_output` | 近地、数据新鲜时停止速度输出 |
| `/p_to_p/state` | 当前任务状态 |
| `/target_position` | `[x_cm, y_cm, z_cm, yaw_deg]` |
| `/target_velocity` | 地图系 `[vx, vy, vz, yaw_rate]` |

状态主流程：

```text
IDLE → TAKEOFF → GOTO_B → HOLD_B
                           └→ DESCEND_B → LANDED_HOLD
```

故障状态包括 `ABORT_HOLD`、`FAULT_HOLD` 和 `OBSTACLE_HOLD`。

## 7. 串口协议增量

```text
0x31 + 8B  机体系 vx/vy/vz/yaw_rate，四个 int16 小端
0x66 0x06  任务结束/速度归零
0x67 0x01  上位机任务准备；仍需 CH6 高位人工授权
0x67 0x00  撤销任务使能并归零，不自动锁桨
```

已删除上位机中的旧 `/velocity_map → 0x32` 旁路、飞控未处理的 `0x07` 测高转发，以及机械臂、磁铁、声光题目接口。运动指令只允许走受任务门控和看门狗保护的 `/target_velocity → 0x31` 链路。

## 8. 安全改动

- PID 检查测高和动态 TF 时间戳；超时先发布一次零速度，再停止刷新。
- 上位机 0.30 s 收不到新速度时，发送三次零速度、撤销 `0x67` 任务使能并关闭转发。
- STM32 300 ms 收不到 `0x31` 时执行 `Set_m_speed(0,0,0,0)`。
- 垂直速度正负方向统一受 `max_vertical_velocity` 对称限幅。
- `/p_to_p/stop_output` 会拒绝过期数据、负高度或高于安全阈值的情况。
- 到 B 默认继续闭环悬停，不自动切断控制，也不自动锁桨。

## 9. 已完成验证

```bash
python3 scripts/verify_offline.py
```

验证覆盖：

- ROS 包名唯一、Python/XML 可解析；
- 三种不同初始航向下的起飞、到 B、下降与 yaw 控制仿真；
- map↔body 速度旋转互逆；
- 水平、垂直和 yaw 速度限幅；
- 任务、PID、串口桥和 STM32 四层安全契约；
- Keil 工程引用已修改的 `AnoDTRasp.c`。

交接版本结果：3 个仿真场景通过，峰值为水平 30 cm/s、垂直 25 cm/s、yaw 25 deg/s。

## 10. 接收人必须完成的验证

- [ ] 在 Ubuntu 22.04 + ROS 2 Humble 上运行 `./scripts/build_onboard.sh` 并保存完整日志。
- [ ] 用 Keil/ARMCC Rebuild `flight_controller/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx`。
- [ ] 核对 `/dev/ttyS4`、`/dev/ttyS6` 与实机接线，并抬高机体验证 `/height` 实时变化。
- [ ] 核对电机序号、旋向、PWM、遥控通道、CH6 高低值和急停。
- [ ] 核对 URDF 中雷达安装方向和 `map → laser_link` yaw 正方向。
- [ ] 拆桨验证 `0x67`：CH6 低位不得解锁；CH6 高位才进入原解锁流程。
- [ ] 拆桨验证停止 PID/拔串口后两级看门狗均归零。
- [ ] 按 `docs/FLIGHT_TEST_CHECKLIST.md` 完成低高度、短距离、系留试飞。

## 11. 已知边界

- 当前没有实机编译和飞行结果，硬件参数仍需现场确认。
- `land` 是基于坐标与飞控/光流测高的下降，不是视觉精准降落。
- 不含自动锁桨；落地后必须人工执行原机锁桨流程。
- Cartographer 在无特征或动态环境中可能漂移，不能跳过现场定位评估。
- 当前没有独立面阵雷达；`/height` 为 0、冻结或方向错误时不得启动任务。
- `FAULT_HOLD` 恢复后应显式调用 `abort_hold`，不得假设任务会自动恢复路线。

## 12. 交接文件完整性

交接压缩包旁会提供同名 `.sha256` 文件。接收人解压前应计算 SHA-256 并核对；解压后先运行 `scripts/verify_offline.py`。
