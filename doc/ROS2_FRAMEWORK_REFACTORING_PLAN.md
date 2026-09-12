# ROS 2 标准控制框架重构说明

本文基于当前项目代码整理，目标是把现有“手柄节点 + 硬件插件内置控制逻辑”的实现，重构为标准的 ROS 2 / `ros2_control` 分层架构：

```text
手柄 / 上层命令
        │
        ▼
teleop_command_node
  解析、归一化、生成三类期望值
        │
        ▼
control_mode_manager（有限状态机）
  仲裁模式、切换 controller_manager 中的 controller
        │
        ├── joint_trajectory_controller  ← 固定点位 / MoveIt
        ├── joint_velocity_controller    ← 关节空间遥操作
        └── cartesian_controller         ← 末端笛卡尔速度遥操作
        │
        ▼
ros2_control ControllerManager
        │  command interfaces
        ▼
RealArmSystemHardware::write()
        │
        ▼
CAN transport / 达妙电机
        ▲
        │
RealArmSystemHardware::read()
        │  state interfaces
        └────────────── 各 controller / joint_state_broadcaster
```

核心原则：硬件接口只负责“硬件生命周期、读反馈、写命令、硬件侧必要的单位/机构映射和安全保护”；它不再订阅手柄、不再运行 FSM、不再实现 DLS 或关节控制器，也不再创建自己的 ROS 节点和 spin 线程。

## 1. 当前项目的实际结构

当前代码并不是没有 ROS 2 控制基础，主要问题是职责边界混合。

### 已有的基础

- `src/arm_description/urdf/arm.ros2_control.xacro` 已声明 `ros2_control` system、关节 command/state interfaces、CAN 参数和电机参数。
- `src/arm_hardware_interface/src/real_arm_hardware_interface.cpp` 已实现 `SystemInterface` 的 `on_init`、`on_activate`、`read`、`write`。
- `src/arm_can` 已封装 CAN socket、达妙电机和多电机集合。
- `my_robot_bringup` 和 `my_robot_moveit_config` 已经使用 `controller_manager`、`joint_state_broadcaster` 和 `joint_trajectory_controller`。
- `my_robot_commander_cpp` 已经使用 `joy`、MoveIt，并提供固定点位、关节速度和笛卡尔速度的部分命令。
- `ControlFsm` 已有 `STOP / IDLE / DLS / JOINT / POSE` 五种状态。

### 当前主要耦合

`ArmHardwareBase` 当前同时持有并执行：

- ROS 订阅：`/arm_motor_enable`、`/arm_hold_position`、`/dls/twist_cmds`、`/joint_vel_cmds`、`/joint_pos_cmds`、`/pose_active`；
- 内部 ROS 节点及独立 executor 线程；
- `ControlFsm`；
- `DlsController`；
- `JointController`；
- 重力补偿计算及调试发布；
- J2/J3 耦合解耦；
- command buffer 的保持、积分和模式切换逻辑。

因此现在的真实数据流实际上是：

```text
手柄 → joy_to_servo_node → 硬件插件内部 topic 回调
                              │
                              ├─ 内部 FSM
                              ├─ DLS / JointController
                              └─ 直接改 hw_commands_*
                                      │
                                      ▼
                                  write() → CAN
```

这意味着 ros2_control 的 controller 并没有真正成为控制算法的唯一命令来源；硬件插件正在“绕过 controller 直接接管命令”。这会导致 controller 切换、资源声明、实时线程安全和 MoveIt 执行语义都变得不清晰。

## 2. 目标职责划分

### 2.1 `teleop_command_node`

建议新建包 `my_robot_teleop`，节点名为 `teleop_command_node`。它只做输入层和命令生成，不直接访问 CAN，也不直接修改硬件接口。

职责：

1. 订阅 `sensor_msgs/msg/Joy`；
2. 根据机器人版本加载手柄映射、方向、死区、最大速度和最大角速度；
3. 做死区、归一化、符号映射、限幅和按键去抖；
4. 输出三类命令：固定点位请求、关节空间命令、末端笛卡尔命令；
5. 输出电机使能、急停、保持等安全请求；
6. 只发布“意图”，不负责决定哪个 controller 最终获得资源。

建议接口：

| 用途 | 建议接口 | 类型 | 说明 |
|---|---|---|---|
| 固定点位 | `/arm/command/named_target` | `std_msgs/String` 或自定义 `NamedTarget` | `home/right/up/left` 等；也可转为 MoveIt action 请求 |
| 关节空间 | `/arm/command/joint_velocity` | 自定义 `JointVelocityCommand` | 关节名、速度、时间戳、deadman 状态 |
| 笛卡尔空间 | `/arm/command/cartesian_twist` | `geometry_msgs/TwistStamped` 或自定义命令 | 明确 `frame_id`、速度单位和有效期 |
| 使能 | `/arm/command/enable` | `std_srvs/SetBool` 或自定义状态命令 | 不建议用无有效期的 Bool topic 做安全控制 |
| 急停/保持 | `/arm/command/stop`、`/arm/command/hold` | service/action | 需要明确一次性事件语义 |

不要继续使用 `Float64MultiArray` 作为正式控制接口。它没有关节名、时间戳、单位和版本信息，调试时很容易发生 7 个数顺序错位。可以在过渡阶段保留兼容桥接，但新 controller 应只使用有明确语义的消息。

### 2.2 `control_mode_manager`

建议新建包 `my_robot_control_manager`，节点名为 `control_mode_manager`。它是应用层 FSM 和 `controller_manager` 的客户端。

职责：

- 接收三类命令和安全事件；
- 对命令做优先级仲裁；
- 维护当前模式和模式转换原因；
- 调用 `/controller_manager/switch_controller`；
- 在 controller 切换前后完成命令清零、当前状态同步和超时保护；
- 发布当前模式、切换结果和故障原因。

建议状态：

```text
DISABLED
  └─ enable → HOLD
HOLD
  ├─ named_target → TRAJECTORY
  ├─ joint_velocity → JOINT_VELOCITY
  ├─ cartesian_twist → CARTESIAN
  └─ disable / timeout / fault → DISABLED
TRAJECTORY
  └─ action done / cancel / fault → HOLD
JOINT_VELOCITY
  └─ deadman release / timeout / stop → HOLD
CARTESIAN
  └─ deadman release / timeout / stop → HOLD
FAULT
  └─ 只有人工复位或明确 reset 才能离开
```

状态机不应位于硬件插件内。`ControlFsm` 可以作为迁移起点，但应移动到 manager 包，并删除对 `DlsController`、硬件 buffer 和 CAN 对象的依赖。

controller 切换应使用标准服务：

```text
deactivate: 当前运动 controller
activate:   目标运动 controller
strictness: STRICT
start_asap: false
```

同一组关节的运动 controller 原则上只允许一个 active，避免 position、velocity 和 trajectory controller 同时争用 command interface。`joint_state_broadcaster` 应一直保持 active，不参与运动模式切换。

### 2.3 三个运动 controller

#### 固定点位 / 轨迹 controller

使用 `joint_trajectory_controller`，命名建议为 `arm_trajectory_controller`。MoveIt 的规划结果、固定点位和上层轨迹统一通过 `FollowJointTrajectory` action 或 trajectory topic 进入。

- `home/right/up/left` 应是 MoveIt named target 或 manager 发起的轨迹 action；
- 轨迹 controller 负责时间参数化轨迹的插值和执行；
- 不再用 `/pose_active` 通知硬件插件进入 POSE；
- 轨迹 controller active 时，不能让 DLS 或关节速度 controller 同时占用同一组关节。

#### 关节空间 controller

建议新增 `arm_joint_velocity_controller`，优先采用现成的 forward command controller 或自定义 `controller_interface::ControllerInterface`：

- 输入带关节名、有效期和速度数组的命令；
- 根据 state interface 做 deadman、超时、关节限位和速度/加速度限制；
- 输出 joint velocity command；
- 如果底层电机只有 MIT 的位置形式，速度积分成位置目标应放在该 controller 内，而不是 hardware 的 `write()` 中。

当前 `JointController` 的位置累积、速度限制和关节限位逻辑，可以迁移到该 controller；`ArmHardwareBase::write()` 中的 `vel_mode_active_`、`integrated_pos_` 应随后删除。

#### 末端笛卡尔 controller

建议新增 `arm_cartesian_controller`，或明确采用 MoveIt Servo 作为该层实现。当前项目已经存在两条互相重叠的路线：

- `DlsController`：硬件插件内部基于 Pinocchio Jacobian 的 DLS；
- MoveIt Servo：`my_robot_commander_cpp` 中可选的 `JointJog` / `TwistStamped` 输出。

必须二选一作为正式控制链路：

1. 若需要固定的 500 Hz、低延迟和可控的实时行为，迁移 `DlsController` 为独立 `arm_cartesian_controller`；
2. 若优先使用 MoveIt 生态，则由 MoveIt Servo 负责笛卡尔到关节命令转换，项目只保留标准 Servo 输出对应的 controller。

不应同时让 Servo 和硬件内部 DLS 对同一机械臂输出。

当前 DLS 算法中的以下功能应保留，但位置移动到 controller：Pinocchio 模型初始化、LOCAL Jacobian、阻尼最小二乘、奇异性保护、加速度限制、关节限位预测、目标位置积分和诊断数据。

### 2.4 `RealArmSystemHardware`

建议最终把 `RealArmHardwareInterface` 更名为 `RealArmSystemHardware`，但名称不是硬性要求。它只保留：

`on_init()`：

- 读取 `hardware_interface::HardwareInfo`；
- 创建 CAN transport 和电机对象；
- 分配 state/command 缓冲；
- 读取关节/电机参数；
- 初始化机构映射和硬件安全参数。

`on_activate()` / `on_deactivate()`：

- 打开/关闭 CAN；
- 电机清错、使能、失能；
- 建立或释放硬件资源；
- 不创建 ROS 节点，不创建 executor，不启动 spin 线程。

`read()`：

- 从 CAN 读取原始反馈；
- 做电机角度/速度/力矩到 ROS 关节状态的单位、方向和 J2/J3 机构映射；
- 填充 state interfaces；
- 检测超时、错误码和通信故障；
- 返回 `ERROR` 或设置硬件故障状态。

`write()`：

- 读取 controller 已写入的 command interfaces；
- 做必要的关节到电机映射、限幅、力矩前馈叠加；
- 生成 CAN 帧并发送；
- 不调用控制器，不消费 topic，不运行 FSM。

重力补偿的归属需要明确：如果它是 controller 的模型前馈，应放入 controller；如果是电机驱动必须叠加的底层保护/补偿，可以保留在 hardware，但必须与 command interface 的语义清楚区分，并不能覆盖 controller 写入的 command。建议第一阶段将重力补偿迁移到控制器，hardware 只做映射和最终安全限幅。

## 3. 现有文件到目标文件的迁移表

| 当前文件/对象 | 当前职责 | 目标处理 |
|---|---|---|
| `arm_hardware_interface/arm_hardware_base.*` | ROS 订阅、FSM、DLS、关节控制、调试、映射 | 拆成 hardware 公共映射工具；控制逻辑迁出 |
| `control_fsm.*` | 硬件内部模式切换 | 移到 `my_robot_control_manager` |
| `dls_controller.*` | 笛卡尔速度到关节目标 | 迁移为 `arm_cartesian_controller` 库/插件 |
| `joint_controller.*` | 关节速度积分和限位 | 迁移为 `arm_joint_velocity_controller` |
| `gravity_compensator.*` | Pinocchio 重力模型 | 由对应 controller 使用，或单独 `arm_dynamics` 库 |
| `real_arm_hardware_interface.*` | CAN + 控制策略 + ROS | 仅保留 `SystemInterface` 和 CAN I/O |
| `remote.*` | 手柄映射/输入转换 | 放入 teleop 包；保留为纯输入映射类 |
| `commander_template.cpp` | 手柄、MoveIt、三类输出、夹爪 | 拆为 teleop 节点、named target client、gripper command |
| `PoseCommand.msg` | 末端命令字段 | 扩展为带 header/frame/有效期的正式命令消息，或改用标准消息 |
| `arm_controllers.yaml` | controller 定义 | 统一为唯一的 ros2_control controller 配置来源 |
| `my_robot_moveit_config/config/ros2_controllers.yaml` | 另一份 controller 定义 | 与 bringup 合并，避免两套配置漂移 |
| `arm.ros2_control.xacro` | 关节和 hardware 参数 | 保留硬件参数，删除控制算法参数和运行时 ROS 语义 |

## 4. 接口和数据语义需要先固定

在写新代码前，建议先形成一份接口契约，至少固定以下内容：

### 关节命名

当前代码的机械臂关节是 `joint1` 到 `joint7`，夹爪是 `joint_right_finger`。所有消息必须使用相同名称，不能只依赖数组位置。现有代码中 `commander_template.cpp`、URDF、controller YAML 已经使用这些名称，但 `JointJog` 的 J5 方向存在额外 `-joint_cmds[4]`，需要统一到一处，不能同时在 teleop 和 hardware 各做一次方向翻转。

### 坐标系

笛卡尔命令必须明确是 `tool_link` 的 LOCAL frame、base frame 还是命令消息中的 `frame_id`。当前 `DlsController` 固定用 Pinocchio `LOCAL` Jacobian，而 commander 发布了 `command_frame_id_`，两者语义可能不一致。重构时应：

- 消息携带 `header.frame_id`；
- controller 明确支持的 frame；
- 不支持的 frame 由 TF 转换，不能静默当作 LOCAL；
- 在诊断中发布实际使用的 frame。

### 命令有效期和死区

速度/笛卡尔命令必须有 timeout。建议 controller 以单调时钟判断：超过 100–200 ms 没有新命令就输出保持或零速度，并通知 manager 回到 `HOLD`。当前硬件插件使用 `kDlsTimeout = 50` 和固定周期假设，属于控制器内部逻辑，不应继续放在 hardware。

### command interface 选择

建议先采用以下最小集合：

```yaml
arm_trajectory_controller:
  command_interfaces: [position]

arm_joint_velocity_controller:
  command_interfaces: [position]

arm_cartesian_controller:
  command_interfaces: [position]
```

原因是当前达妙 MIT CAN 控制最终发送的是位置、速度、力矩组合，但硬件接口已经将三者都声明出来。若自定义 controller 确实需要 effort，应明确 controller 输出的 effort 是否是关节力矩、是否包含重力补偿，以及 hardware 是否再做一次映射。第一阶段不建议让多个 controller 争用三种 command interface。

## 5. 需要优先修正的现有问题

### 5.1 硬件插件内创建 ROS 节点

`setup_internal_node()` 在硬件插件中创建独立节点和线程。这会绕过 controller_manager 的 executor 和生命周期管理，也让硬件插件承担非实时 ROS 工作。迁移完 topic 后，删除 `internal_node_`、`spin_executor_`、`spin_thread_` 以及所有内部 subscription/publisher。

### 5.2 非实时线程与实时循环共享数据

当前 ROS 回调直接写入 `dls_twist_`、`joint_vel_target_`、`joint_pos_target_`，控制循环同时读取；其中部分变量是 atomic，部分 vector/array 不是，存在数据竞争。新结构中：

- ROS callback 只写 controller 的实时安全命令缓存；
- 使用 realtime buffer、双缓冲或锁-free snapshot；
- controller 的 `update()` 只使用当前周期快照；
- hardware `read/write` 不再接触 ROS callback 数据。

### 5.3 controller 配置重复

当前至少有：

- `src/my_robot_bringup/config/arm_controllers.yaml`；
- `src/my_robot_moveit_config/config/ros2_controllers.yaml`；
- `config/variants/V1.0/ros2_controllers.yaml`；
- `config/variants/V1.1/ros2_controllers.yaml`。

它们的 controller 名称、接口和参数并不完全一致，例如 bringup 中有 `forward_velocity_controller`，MoveIt 配置中没有。应建立一份按机器人版本组织的 controller 配置，bringup 和 MoveIt 都引用同一份，或者明确分出“硬件启动配置”和“MoveIt trajectory execution 配置”但由测试检查两者一致。

### 5.4 `POSE` 通过 Bool 与硬件隐式握手

当前 `pose_sub_` 通过 `/pose_active` 驱动硬件 FSM，且 `process_control()` 还等待 trajectory command 接近当前状态。这是一个隐式协议。标准做法是由 manager 先切换 controller，再让 trajectory controller 通过 action 接收目标；切换失败、取消和超时都由 action/manager 显式反馈。

### 5.5 硬件层的速度积分

当前 `write()` 检测 `hw_commands_vel_` 后在硬件层积分 `integrated_pos_`。这会使“velocity command interface”实际变成“位置目标生成器”，并且与 controller 的职责冲突。应把积分器移到关节速度 controller，并最终只让 hardware 看到它声明的目标接口。

### 5.6 J2/J3 映射参数需要统一

`RealArmHardwareInterface` 读取了 `j2j3_coupling`、`j2j3_j3_scale`、`j2j3_j3_offset` 等参数；`arm.ros2_control.xacro` 当前主要声明的是 `j2j3_up_poly_*`、`j2j3_down_poly_*` 等参数。需要检查最终展开 URDF 中参数是否真的存在，并将“关节侧 ↔ 电机侧”的映射写成独立、可单测的 `JointMotorMapper`，而不是散落在 read、write 和重力补偿中。

### 5.7 固定 7 关节常量

`kJointCount = 7`、`std::array<double, 7>`、固定的 `joint1` 到 `joint7` 在多个地方出现。建议从 `HardwareInfo` 和 controller 参数中得到关节列表，只有机器人特定的 J2/J3 索引在配置层声明。夹爪等额外关节不能靠“没有 can_id 就 Mock”来表达最终硬件语义。

## 6. 推荐的目标目录

建议逐步形成如下目录：

```text
src/
├── arm_can/
│   └── ...                         # CAN socket、达妙协议、transport
├── arm_description/
│   └── ...                         # URDF/Xacro、ros2_control 声明
├── arm_hardware_interface/
│   ├── include/arm_hardware_interface/
│   │   ├── real_arm_system.hpp
│   │   ├── mock_arm_system.hpp
│   │   ├── joint_motor_mapper.hpp
│   │   └── hardware_safety.hpp
│   └── src/
│       └── real_arm_system.cpp
├── arm_dynamics/
│   ├── dls_solver.hpp
│   ├── gravity_model.hpp
│   └── pinocchio_model_loader.cpp
├── arm_joint_velocity_controller/
│   ├── include/...
│   └── src/...
├── arm_cartesian_controller/
│   ├── include/...
│   └── src/...
├── my_robot_control_manager/
│   ├── include/.../control_mode_manager.hpp
│   ├── src/control_mode_manager.cpp
│   └── config/control_modes.yaml
├── my_robot_teleop/
│   ├── include/.../joy_mapper.hpp
│   ├── src/teleop_command_node.cpp
│   └── config/joy_mapping.yaml
├── my_robot_interfaces/
│   ├── msg/JointVelocityCommand.msg
│   ├── msg/CartesianTwistCommand.msg
│   ├── msg/ControlMode.msg
│   └── srv/SetControlMode.srv
├── my_robot_moveit_config/
└── my_robot_bringup/
    ├── launch/robot.launch.py
    └── config/controllers.yaml
```

`arm_dynamics` 不应依赖 `rclcpp`，这样 DLS、重力和机构映射可以在普通 C++ 单元测试、仿真和 controller 中复用。

## 7. 分阶段迁移方案

### 阶段 0：建立基线

- 保存当前 real/mock 启动命令和控制器列表；
- 记录当前 CAN 反馈频率、控制周期、关节方向、J2/J3 映射结果；
- 在 mock hardware 下确认 MoveIt 固定点位可执行；
- 为现有 DLS、JointController、J2/J3 mapper 补最小单元测试；
- 不在此阶段改变电机增益和机械参数。

验收：旧系统在 `use_mock_hardware:=true` 和实机低速模式下行为可复现。

### 阶段 1：先拆出命令协议

- 新建自定义命令消息，替代 `Float64MultiArray`；
- 将 `remote.cpp` 改为无 ROS 依赖的纯映射类；
- 新建 `teleop_command_node`，只发布三类正式命令；
- 暂时增加一个 compatibility adapter，把新消息转换为当前 `/dls/twist_cmds`、`/joint_vel_cmds`，确保旧系统仍能运行。

验收：手柄归一化结果、方向和 deadman 行为与基线一致。

### 阶段 2：先把 FSM 移出 hardware

- 新建 `control_mode_manager`；
- 把 `ControlFsm` 迁移过去；
- manager 先只发布当前模式和调用 controller switch 服务；
- 旧 hardware 内部 FSM 暂时保留为兼容层，但不再接收新 teleop topic；
- 逐步删除 `/pose_active`、`/arm_motor_enable` 等隐式协议。

验收：每种模式只有一个运动 controller active；切换时不跳变、不回拉旧轨迹。

### 阶段 3：迁移关节 controller

- 将 `JointController::Update()` 迁移到标准 controller 的 `update()`；
- 由 controller 读取 state interface，产生 command interface；
- 增加命令超时、限速、限加速度和当前状态同步；
- 删除 hardware 中 `process_control()` 的 `JOINT` 分支和 `integrated_pos_`。

验收：关节速度命令只通过 controller 产生，hardware 只负责 `write()`。

### 阶段 4：迁移笛卡尔 controller

- 明确选择自研 DLS 或 MoveIt Servo；
- 如果选择自研，将 `DlsController` 和 Pinocchio 模型放入独立 controller/library；
- 明确 `frame_id`、LOCAL/WORLD Jacobian、单位、超时和奇异性故障策略；
- 删除 hardware 中 DLS topic、DLS FSM 分支和 debug publisher。

验收：笛卡尔命令在 mock hardware 中可重复测试，接近奇异位形时不会产生异常关节速度。

### 阶段 5：清理硬件接口

- 删除 `ArmHardwareBase` 中所有 ROS node/subscription/publisher；
- 删除硬件层 FSM、DLS、JointController；
- 保留 CAN、反馈、command/state interface、J2/J3 映射、底层安全限幅；
- 将重力补偿移动到明确的 controller 或独立 dynamics component；
- 将真实硬件和 mock hardware 的接口定义统一。

验收：不启动 teleop 和 MoveIt 时，`ros2_control_node` 仍可独立启动；hardware plugin 不创建额外 ROS 节点。

### 阶段 6：统一 launch 和配置

- `robot.launch.py` 负责 description、`ros2_control_node`、broadcaster、manager、teleop；
- `move_group` 和 RViz 作为可选组件；
- 删除重复 controller YAML，统一 variant 选择逻辑；
- 用 event handler 或 lifecycle/服务等待 controller_manager 就绪后再启动 manager；
- 把 real/mock、robot version、CAN interface、控制频率作为 launch 参数；
- 加入启动时检查：controller 是否存在、关节名是否一致、command interface 是否匹配。

## 8. 测试和验收标准

### 单元测试

- 手柄死区、归一化、方向和按键去抖；
- 命令消息合法性、时间戳和超时；
- FSM 所有状态转换和非法转换；
- J2/J3 位置、速度、力矩正逆映射；
- DLS 奇异性保护、速度/加速度/关节限位；
- CAN 电机参数解析和 command 到电机帧的映射。

### 集成测试

- mock hardware + controller_manager：三种 controller 能被严格切换；
- 固定点位执行时没有速度/DLS controller 抢占资源；
- 关节和笛卡尔命令超时后回到 HOLD；
- manager 重启或 teleop 断开时 hardware 不会继续执行旧速度命令；
- CAN 断线、单电机错误和反馈超时能进入 FAULT/安全输出；
- J2/J3 映射后的 state 与电机原始反馈方向一致。

### 实机验收顺序

1. CAN 只读反馈；
2. 单关节低增益 HOLD；
3. 单关节速度 controller；
4. 全关节速度 controller；
5. 笛卡尔 controller 小速度；
6. 固定点位短轨迹；
7. 完整 MoveIt 轨迹；
8. 使能、失能、急停、断线恢复。

每一步都应有最大速度、最大位置误差和停机条件，不能直接从当前内部 DLS 版本切到完整实机轨迹。

## 9. 建议的第一批代码修改顺序

第一批不建议直接重写硬件插件，而是按以下顺序提交小改动：

1. 新增本文中的正式命令消息和 `my_robot_teleop` 包；
2. 将 commander 的归一化逻辑迁入 teleop，保留 compatibility adapter；
3. 新增 `my_robot_control_manager`，先只实现状态发布和 controller switch；
4. 新增统一 `controllers.yaml`，确保 mock hardware 下能启动三种模式；
5. 把 `JointController` 迁移成 controller，并用 mock 做闭环测试；
6. 把 `DlsController` 迁移成 controller；
7. 最后删除硬件插件内部 ROS/FSM/算法代码；
8. 统一 launch、MoveIt trajectory execution 和 variant 配置。

这样可以每一步都保持一个可运行系统，且能明确判断问题来自输入层、模式管理、controller 还是 CAN/hardware 层。

## 10. 重构完成的判定条件

满足以下条件时，才可以认为已经达到标准 ROS 2 框架：

- hardware plugin 不创建 ROS node、executor 或 subscription；
- 所有运动命令都先进入 controller，再由 controller 写 command interface；
- `read()` 只产生 state，`write()` 只消费 command；
- 每个运动模式对应一个明确的 controller，切换由 controller_manager 完成；
- FSM 不依赖 CAN、硬件 buffer 或具体控制算法；
- MoveIt 固定点位走 `FollowJointTrajectory`，而不是自定义 Bool 握手；
- 关节/笛卡尔命令具有明确关节名、坐标系、单位、时间戳和有效期；
- mock hardware、仿真和真实 CAN 使用同一套 controller 接口；
- CAN 断线、命令超时、反馈超时和急停均有可验证的安全行为；
- controller YAML、URDF/Xacro 参数和 MoveIt 配置不再存在互相矛盾的重复定义。

