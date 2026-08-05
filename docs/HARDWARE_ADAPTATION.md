# 硬件适配说明

## 上位机串口

默认配置位于 `onboard_ws/src/p_to_p_mission/config/p_to_p.yaml`：

```yaml
uart_to_stm32_node:
  ros__parameters:
    serial_port: /dev/ttyS6
    baud_rate: 921600
```

当前主启动不使用独立面阵激光。光流模块的测距先进入飞控，再由飞控通过
`/dev/ttyS6` 的 `0x05` 帧回传，`uart_to_stm32_node` 将其发布为 `/height`（cm）。
拆桨测试时必须抬高机体确认 `/height` 随实际高度变化，持续为 0 时不得试飞。

在 Orange Pi 上执行 `ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM*` 确认设备名。串口用户通常需要属于 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"
```

重新登录后生效。不要通过长期使用 `chmod 777` 绕过权限。

平面 BlueSea 雷达默认串口为 `/dev/ttyS4`、921600，配置文件为：

```text
onboard_ws/src/bluesea2/src/bluesea-ros2/params/uart_lidar.yaml
```

也可在启动时通过 `lidar_params_file:=/绝对路径/uart_lidar.yaml` 指定另一份配置。

## 坐标方向

协议约定：

- 地图系：由 Cartographer 的 `map` 决定。
- 机体系：X 前、Y 左、Z 上。
- yaw：逆时针为正，单位 deg。
- `uart_to_stm32` 默认把 map 系目标速度旋转到机体系。

拆桨状态发布小速度并观察飞控接收值，逐轴确认方向。若机体安装方向或 `laser_link` 外参不同，应修改 URDF，不要仅靠交换 B 点符号掩盖错误。

## TF 与雷达安装

`my_carto_pkg/urdf/fly.urdf` 描述传感器与机体关系。必须核对：

- 平面雷达的 X/Y 安装方向和偏移；
- `laser_link` 是否代表用于控制的机体坐标；
- 雷达扫描平面是否水平；
- Cartographer 静止时位置是否稳定、转动机体时 yaw 正负是否正确。

## 飞控

默认目标为 STM32F407。重点核对：

- Keil 工程目标器件和晶振；
- 遥控器 CH6 是否仍用于任务准备/解锁流程；
- 上位机串口对应的 MCU UART、波特率和 DMA/中断；
- 电机序号、旋向和 PWM 输出；
- 飞控高度/速度模式与原机一致；
- 急停和失控保护可用。

## 协议

Orange Pi → STM32：

```text
AA FF 31 08 vxL vxH vyL vyH vzL vzH yawL yawH SC AC
```

四个量均为有符号 `int16` 小端：`cm/s, cm/s, cm/s, deg/s`。

任务结束：`ID=0x66, data=0x06`。上位机在发送结束帧前会先连续发送三次零速度；飞控也会在 300 ms 未收到新 `0x31` 时自行归零。

点到点任务门控：

```text
ID=0x67, data=0x01  上位机任务已准备；仍需遥控器 CH6 高位才允许切模式/解锁
ID=0x67, data=0x00  撤销任务使能并将外部速度目标归零；不会自动锁桨
```

兼容原机遥控流程：若未收到 `0x67 enable`，上电后必须先让飞控确认一次 CH6 低位，再拨到高位，才允许进入原切模式/解锁流程。收到 `0x67 disable` 或速度超时后应先把 CH6 拨回低位，再重新准备任务。

上位机在任务启动时重复发送三次使能帧，在速度超时、任务结束或节点退出时发送撤销帧。该协议取代原题目工程依赖触摸屏密码帧和 `0xA2` 连接标志的启动条件。
