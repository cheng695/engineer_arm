# ROS2 工作空间重构排查记录

## 当前结论

当前工作空间已完成基础编译、Mock/J2/J3 测试，并已收敛主要配置和控制安全链路。真实 CAN 电机仍需在安全条件下单独验证。

## 优先处理的问题

### 1. J2/J3 标定参数没有进入当前算法（已处理）

`control_gains.yaml` 和 xacro 使用的是：

```text
j2j3_up_poly_*
j2j3_down_poly_*
```

真实硬件接口读取的却是：

```text
j2j3_poly_*
```

因此当前标定系数没有完整进入算法，实际运行时可能使用默认系数。

当前已统一为以下链路：

```text
control_gains.yaml
    -> launch 参数
    -> xacro hardware 参数
    -> RealArmHardwareInterface
    -> ArmHardwareBase J2/J3 解耦算法
```

### 2. 使能流程没有等待硬件就绪（已处理）

已将 `ControlModeManager::enableCallback()` 修改为先设置 `enable_pending_`，等待硬件发布 `hardware_ready=true` 后再触发状态切换。

硬件发布 `hardware_ready=false` 时，管理器会清除 pending，并将当前控制状态切换到 `DISABLED`。重复使能不会改变已经生效的运动模式。

可能导致：

- 电机尚未完成使能和反馈确认时，控制器已经激活；
- 控制器过早读取状态并锁定目标位置；
- 暖启动阶段和控制器切换阶段发生竞态；
- 硬件 ready 变为 false 后，管理器没有自动退出当前运动状态。

应形成以下顺序：

```text
收到使能请求
    -> 等待 hardware_ready=true
    -> 切换到 HOLD 或重力测试控制器
    -> 允许接收运动命令
```

### 3. JOINT/CARTESIAN 命令缺少超时保护（已处理）

`JointCommandController` 和 `DlsCartesianController` 已增加命令接收时间和超时判断，默认超时时间为 0.1 秒。

当手柄断连、发布节点退出或通信中断时，控制器仍可能继续积分并输出旧命令。

当前行为为：

- 超时后输出零速度或零 Twist 并保持当前位置；
- 控制器重新激活时清空旧命令有效标志；
- 重新收到有效命令后才恢复运动。

### 4. 运动过程中缺少电机反馈掉线检测（已处理）

真实硬件接口现在会在正常 `read()` 过程中持续检查每个真实电机的反馈。发现异常后会立即清零控制目标、发送失能帧并发布 `hardware_ready=false`，控制管理器随后切换到 `DISABLED`。

运行过程中检查：

- 每个真实电机的反馈计数是否持续增加；
- 最近一次反馈时间是否超过 100 ms；
- 电机是否仍处于 enabled 状态以及错误码是否正常；
- 任一电机掉线时是否立即清除 `hardware_ready_`；
- 是否停止控制输出并通知控制管理器切换到 `DISABLED`。

### 5. 控制增益 YAML 覆盖链路存在风险（已处理）

已移除真实接口通过内部节点二次读取增益的路径。启动时将以下参数显式传入 xacro，硬件初始化直接从 `HardwareInfo::joints[].parameters` 读取并设置到电机对象：

```text
arm_control_gains.<joint>.kp
arm_control_gains.<joint>.kd
```

已验证生成的 ros2_control XML 能反映修改后的 kp/kd。

### 6. gravity_always_on 启动存在控制器加载竞态（已处理）

开启 `gravity_always_on:=true` 时，控制管理器可能在 `arm_gravity_controller` 完成加载和配置前请求激活它。

已经观察到类似日志：

```text
Could not 'activate' controller ... 'arm_gravity_controller'
Aborting, no controller is switched! ('STRICT' switch)
```

管理器现在通过 `/controller_manager/list_controllers` 确认目标处于 `inactive/active` 后才切换；未就绪请求会保存在 pending 中，由定时器重试，并按当前状态避免重复激活/停用重力控制器。

处理内容：

- 等待所需控制器处于 configured/inactive 状态；
- 保存尚未完成的切换请求；
- 控制器准备完成后重试；
- 只在重力控制器当前未激活时请求激活；
- 只在失能时请求停用重力控制器。

### 7. 纯重力测试模式约束不完整（已处理）

`gravity_test_mode` 现在拒绝 JOINT、CARTESIAN 和命名轨迹请求，HOLD/PAUSED 统一映射到重力控制器；使能和失能仍经过正常硬件 ready 流程。

纯重力测试应该明确限制为：

```text
DISABLED -> GravityCompensationController
```

在该模式下：

- 不接受 JOINT、CARTESIAN、TRAJECTORY 运动请求；
- 只允许重力控制器写 effort；
- 电机使能和失能仍走正常硬件流程；
- 失能后关闭重力控制器。

### 8. Real/Mock 节点线程缺少析构兜底（已处理）

Real 和 Mock 硬件接口析构函数现在都会执行内部 executor 的 cancel/join 清理，并对已清理资源安全复入。

需要在析构函数中增加统一清理，确保：

- executor 被 cancel；
- spin 线程完成 join；
- timer、publisher、subscription 和 node 按顺序释放；
- 不重复释放已经清理的资源。

Mock 失活时发布 `hardware_ready=false` 的通知链路也需要进一步确认，避免管理器保留过期的 ready 状态。

## 其他需要清理的地方

### 失效配置（已处理）

launch 文件已只宣传 `off` 和 `external_gravity_only`，删除旧 J3 重力比例参数，且 `gravity_effort_scale` 默认值改为 `1.0`。

### 控制器关节索引缺少完整校验（已处理）

JOINT、DLS 和 GravitySolver 使用 `getJointId()` 后，需要同时检查：

- joint id 是否有效；
- id 是否小于 `model.joints.size()`；
- 关节是否为一自由度；
- q/v 索引是否在对应向量范围内。

配置中出现错误关节名时，应返回配置错误，不能继续访问潜在的非法索引。

### 手柄暂停命令没有真正发布

Commander 创建了 `pause_pub_`，但当前手柄回调没有调用它。

启动提示中显示“RT→暂停”，实际链路却没有闭合。需要补充按钮边沿检测和 `/arm/command/pause` 发布逻辑，或者删除这条提示。

### 依赖仍有重构残留（部分处理）

需要重新检查以下依赖关系：

- `arm_hardware_interface` 是否仍然强制依赖 Pinocchio 和 MoveIt；
- `my_robot_controllers` 是否必须依赖 `arm_hardware_interface`；
- `remote` 手柄解析仍暂留在硬件接口包中，后续可作为独立的小范围包迁移；本轮未改手柄暂停链路。
- 已删除的硬件层控制器、DLS 和重力文件是否仍出现在安装、导出或文档中。

目标是让硬件接口只负责硬件通信、状态管理和 J2/J3 解耦，运动学、重力计算和运动控制由控制器包负责。

### raw 数据语义需要明确

当前 `/arm_debug/raw_motor_states` 发布的是 CAN 协议解析后的电机反馈，未应用电机方向符号，因此：

```text
方向符号和 J2/J3 解耦之前的 raw 电机反馈
```

协议帧解析本身仍已完成；供控制器使用的关节状态在硬件接口的后续阶段单独应用方向和解耦。

## 已完成的验证

加载完整 ROS 环境后：

- 13 个工作空间包编译通过；
- `hardware_contract_test` 通过；
- J2/J3 基础解耦测试通过；
- 普通 Mock 控制链路测试通过，覆盖 HOLD、JOINT、轨迹、PAUSED 和 DISABLED；
- 开启 `gravity_always_on` 的测试复现了重力控制器启动竞态。

尚未进行真实 CAN 电机测试。

## 建议处理顺序

1. 统一 J2/J3 标定参数命名和传递链路；
2. 修复使能请求与 hardware ready 的握手；
3. 增加 JOINT/CARTESIAN 命令超时保护；
4. 增加运行中的电机反馈掉线检测；
5. 修复重力控制器加载和并行激活竞态；
6. 明确纯重力测试模式的状态约束；
7. 清理失效 launch、参数和包依赖；
8. 将 remote 手柄解析独立出硬件接口包（暂停命令按当前需求暂不改）。
