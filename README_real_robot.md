# 实车启动说明

## 概览

当前工作区为实车提供了两个辅助脚本：

- [scripts/start_real_robot.sh](/home/rcia/Desktop/engineer_whc/scripts/start_real_robot.sh)
- [scripts/start_commander.sh](/home/rcia/Desktop/engineer_whc/scripts/start_commander.sh)

`start_real_robot.sh` 用来配置 CAN 并启动实车主系统。

`start_commander.sh` 用来在第二个终端启动 commander 节点。

## 1. 主系统启动

打开终端 1：

```bash
cd ~/Desktop/engineer_whc
bash scripts/start_real_robot.sh --bitrate 1000000
```

常见用法：

```bash
bash scripts/start_real_robot.sh --bitrate 1000000 --build
bash scripts/start_real_robot.sh --bitrate 1000000 --gravity-mode assist
bash scripts/start_real_robot.sh --bitrate 1000000 --gravity-mode gravity_only
```

重力补偿模式：

- `off`：默认值，不启用重力补偿
- `assist`：期望/PID 叠加重力补偿
- `gravity_only`：只输出重力补偿

## 2. Commander 启动

打开终端 2：

```bash
cd ~/Desktop/engineer_whc
bash scripts/start_commander.sh
```

如果需要顺手编译：

```bash
bash scripts/start_commander.sh --build
```

## 3. 电机使能

两个节点都启动完成后：

- 按一次 `buttons[9]` 使能电机扭矩
- 再按一次 `buttons[9]` 失能电机扭矩

当前硬件接口在电机失能期间会持续把期望位置对齐到当前反馈位置，用来减小重新使能时的跳变。

## 4. CAN 映射

当前 [arm.ros2_control.xacro](/home/rcia/Desktop/engineer_whc/src/arm_description/urdf/arm.ros2_control.xacro) 中的配置为：

- `joint1` 到 `joint4` 使用 `can0`
- `joint5` 到 `joint7` 使用 `can1`

## 5. 说明

- 辅助脚本中的默认工作区路径是 `~/Desktop/engineer_whc`
- 默认 ROS 发行版是 `humble`
- 默认 CAN 驱动模块是 `mttcan`
- 默认 CAN 波特率是 `1000000`

如果你的硬件使用不同的波特率或驱动，可以显式传参：

```bash
bash scripts/start_real_robot.sh --bitrate 500000 --can-driver <your_driver>
```
