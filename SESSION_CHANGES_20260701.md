# Session Changes — 2026-07-01

## 概览

9 个文件被修改，涉及 7 个功能模块。以下按功能分组描述每次改动及其影响范围。

---

## 一、使能/失能修复（实机关键）

### 1.1 Home 键不能使能/失能

**根因**：三条不同路径互相冲突：
- `remote.cpp` 读 `buttons[9]`，但实际 Home 键在 `buttons[10]`
- `remote.hpp` 的 `enable()/disable()` 条件反了（`motors_on_` 在 `update()` 中先翻转，后检查）
- `write()` 中 `process_fdcc()` 在 `process_motor_requests()` 前面，FDCC 有 twist 数据时永远不处理使能/失能请求

**改动**：

| 文件 | 改动 | 说明 |
|------|------|------|
| `remote.cpp` | `buttons[9]` → `buttons[10]` | Home 键映射修正 |
| `remote.hpp` | `enable()`: `!motors_on_` → `motors_on_` | 条件交换（翻转后检查） |
| `remote.hpp` | `disable()`: `motors_on_` → `!motors_on_` | 同上 |
| `arm_hardware_interface.cpp` | `process_motor_requests()` 移到 `process_fdcc()` 前面 | 使能/失能不被 FDCC 阻塞 |
| `arm_hardware_interface.cpp` | 失能后 `write()` 直接 return，不调用 `send_can_commands()` | MIT 帧会隐式重新使能电机 |
| `arm_hardware_interface.cpp` + `hpp` | 添加 `/joy` 直接订阅 | 硬件接口内处理 Home 键，不依赖外部节点 |
| `commander_template.cpp` | 添加 `/arm_motor_enable` 发布（后移除） | joy_to_servo 节点也发使能命令，但和硬件接口的 /joy 订阅冲突，最终移除 |

### 1.2 失能帧发出后电机不关

**根因**：DaMiao 电机 MIT 模式下，MIT 控制帧被固件当作隐式使能。失能帧 `FF*7 FD` 发出后，同一 `write()` 周期内的 `send_can_commands()` 又发了 MIT 帧，把电机重新唤醒。

**改动**：
- `arm_hardware_interface.cpp`：失能分支移除 `send_can_commands()` 调用

---

## 二、安全使能——防止机械臂飞起

**根因**：使能瞬间控制器指令位置和实际位置差距大，MIT 协议 `torque = Kp × 误差` 产生巨大阶跃力矩（J8009 Kp=120，0.1rad 误差 = 12Nm）。

### 2.1 安全使能两阶段

**改动** (`arm_hardware_interface.cpp`)：
- Phase 1（Hold，30 帧）：锁定使能瞬间位置，零速零力矩
- Phase 2（Blend，20 帧）：从锁定位置线性过渡到控制器指令
- 总计 50 帧 = 100ms（500Hz）
- `enable_motors()` 中同步 `safe_zero_held_pos_` 和 `last_sent_cmd_pos_`

### 2.2 持续安全限幅

**改动** (`arm_hardware_interface.cpp` + `hpp`)：
- **Layer 1 位置误差保护**：`|cmd_pos - actual_pos| ≤ T_max / Kp / 1.5`，动态按电机型号计算
- **Layer 2 步进限制器**：每周期 `|Δpos| ≤ 0.03 rad`（等效最大速度 15 rad/s）
- 新增常量：`kMaxPosStep=0.03`, `kPosErrorSafetyFactor=1.5`
- 新增状态：`last_sent_cmd_pos_`, `safe_zero_held_pos_`
- 初始化：`on_activate()` 和 `enable_motors()` 中同步 `last_sent_cmd_pos_`

---

## 三、CAN 反馈 Echo 过滤

**根因**：SocketCAN 开启 ECHO 模式，发出去的命令帧被回显，和电机反馈帧共用同一个 CAN ID。`readFeedback()` 把两类帧都当反馈解析，echo 帧覆盖了真实反馈数据，导致 `hw_states_pos_` 读到垃圾值（J2=0, J3=0）。

**改动** (`dm_device_collection.hpp`)：
- 在 `parse_feedback()` 前加校验：`data[0] & 0x0F == motor->get_can_id()`
- DM 反馈帧 byte[0] 低 4 位 = 电机 ID，命令帧无此标志

**影响**：`arm_can` 包，rebuild 后生效。

---

## 四、重力补偿 Preload

### 4.1 URDF 文件生成

**根因**：xacro 硬件参数 `robot_description` = `/tmp/robot_description.urdf`（27字符路径），但 launch 文件从没把 URDF 写到这个文件。Pinocchio 加载的是过期旧文件，导致：
- `nq=9`（多一个 DOF，关节索引错位）
- 质量/质心参数不对 → 重力补偿算出来 J2=-1.5Nm（实际需要 -8.4Nm）

**改动** (`arm_bringup.launch.py`)：
- 添加 `_write_urdf_to_file()` 函数，启动时运行 xacro 并写入 `/tmp/robot_description.urdf`

### 4.2 FDCC 力矩字段为空

**发现**（`fdcc_controller.cpp:156`）：`out.push_back({pos, vel, 0.0})` — FDCC 只输出位置和速度，力矩恒为 0。前馈全靠重力补偿。

---

## 五、控制增益

### 5.1 J3 Kp=0.1 被 MIT 编码截断为 0

**根因**：MIT 协议 `float_to_uint(0.1, 0, 500, 12) = 0.8 → trunc → 0`。J3 没有位置刚度，纯自由模式。

**改动** (`arm.ros2_control.xacro`)：J3 `kp=0.1→120.0, kd=0.1→3.0`

### 5.2 sync_control_gains() 不生效

**发现**：`sync_control_gains()` 从 `internal_node_`（`arm_hw_internal`）读参数，但 YAML 参数在父节点 `ros2_control_node` 上。`get_parameter_or()` 永远返回默认值 → 条件 `kp != motor->get_kp()` 永假 → YAML 增益永远不生效。

**影响**：YAML 里调的增益不会在运行时覆盖 xacro。目前靠 xacro 直接设置。

---

## 六、控制增益 YAML 调整

**改动** (`control_gains.yaml`)：
- J2: kp=120→240, kd=3.0→6.0
- J3: kp=120→240, kd=3.0→6.0

**注意**：由于 5.2 的问题，这些值目前不生效，实际用的是 xacro 里的值。

---

## 七、Rviz 初始姿态

**改动** (`my_robot.srdf`)：`home` 和 `up` group_state 顺序交换（后回退）

**改动** (`arm_hardware_interface.cpp`)：`read()` 中失能时用 `hw_commands_pos_` 替代 CAN 反馈（后回退）

**结论**：xacro 中已有正确的 `initial_value`（home 值），启动时 `/joint_states` 应显示 home。问题是 echo 污染导致位置被清零。

---

## 八、J2/J3 耦合与方向

### 8.1 方向分析

- J2 和 J3 镜像安装，对抗重力时力矩应异号
- 实测两个都输出负力矩 → J3 可能方向反了
- 但用户验证后认为方向没错，撤销了 `direction=-1.0` 的改动

### 8.2 力矩分配

- J2 扛主要重力（-8.4Nm），J3 接力矩较小（-4.8Nm）
- 重力补偿（-1.5Nm）严重不足，PD 在硬扛

---

## 改动文件清单

| # | 文件 | 改动类型 |
|---|------|---------|
| 1 | `arm_can/.../dm_device_collection.hpp` | echo 过滤 |
| 2 | `arm_description/.../arm.ros2_control.xacro` | J3 kp 修改、Simulation 切换（已回退） |
| 3 | `arm_hardware_interface/.../arm_hardware_interface.hpp` | 安全常量、状态变量、joy 订阅 |
| 4 | `arm_hardware_interface/.../arm_hardware_interface.cpp` | 使能/失能、安全层、FDCC 顺序、调试日志 |
| 5 | `arm_hardware_interface/.../remote.hpp` | enable/disable 条件修复 |
| 6 | `arm_hardware_interface/.../remote.cpp` | Home 键索引修正 |
| 7 | `my_robot_bringup/.../control_gains.yaml` | J2/J3 Kp 翻倍 |
| 8 | `my_robot_bringup/.../arm_bringup.launch.py` | URDF 文件写入 |
| 9 | `my_robot_commander_cpp/.../commander_template.cpp` | /arm_motor_enable 发布（后清理） |

---

## 重新应用时的建议顺序

1. **先修 CAN 反馈** — echo 过滤（#1），确保位置读数正确
2. **再修使能/失能** — FDCC 顺序（#4）、失能不发送 MIT 帧（#4）、Home 键（#5, #6）
3. **安全层**（#3, #4） — 防止飞臂
4. **重力补偿**（#8） — URDF 文件生成
5. **增益**（#2, #7） — J3 Kp、YAML
6. **仿真/真机兼容** — 不要再动 xacro 的 plugin 选择，改 `ArmHardwareInterface` 内部处理

---

## 九、调试日志（临时，建议移除或降级）

**添加位置**：`arm_hardware_interface.cpp`

1. `enable_sub_` 回调中加了 `[DEBUG] enable_sub 收到 true/false` 日志
2. `process_motor_requests()` 中加了 `[DEBUG] 消费 enable/disable_requested` 日志
3. `send_can_commands()` 中加了 `[SAFETY] 位置误差超限` 警告（每 100 帧一次）

**用途**：跟踪使能/失能请求从订阅到执行的完整链路。

---

## 十、未解决的问题

| 问题 | 状态 | 分析 |
|------|------|------|
| 重力补偿值偏小（J2 需要 -8.4Nm，只给 -1.5Nm） | 未解决 | URDF 文件更新后未验证是否改善；FDCC 力矩字段为 0 是设计如此 |
| sync_control_gains() 不生效 | 未解决 | 需要修改为从父节点读参数，或改为 xacro 传参 |
| J3 和 J2 力矩输出对抗 | 部分解决 | J3 Kp=0 是主要原因（已修复），但 J3 方向是否反了未最终确认 |
| 仿真模式下无法走 FDCC | 未解决 | 需要给 ArmHardwareInterface 加仿真模式参数 |
| DM 电机 `data[0]` 低 4 位校验 | 待验证 | 确认所有 DM 型号（J4310/J4340/J8009）反馈帧都遵循此格式 |

---

## 十一、重应用时的关键代码片段

### Echo 过滤（dm_device_collection.hpp readFeedback）

```cpp
// 在 match 之后、parse_feedback 之前：
uint8_t feedback_id = frame.data[0] & 0x0F;
if (feedback_id != static_cast<uint8_t>(motor->get_can_id()))
    continue;  // 跳过命令回显帧
```

### FDCC 顺序修复（arm_hardware_interface.cpp write）

```cpp
// 原顺序：process_fdcc() → process_motor_requests()
// 修复后：process_motor_requests() → motors_enabled_ 检查 → process_fdcc()
// 确保使能/失能请求每周期都被处理，不被 FDCC 阻塞
```

### 安全限幅（send_can_commands 中，发 CAN 前）

```cpp
// Layer 1: 位置误差保护（限制 MIT 协议可产生的力矩）
double max_pos_error = motor->parameters().T_MAX / motor->get_kp() / 1.5;
pos_cmd = actual_pos + clamp(pos_cmd - actual_pos, -max_pos_error, +max_pos_error);

// Layer 2: 步进限制器（限制每周期位置变化）
pos_cmd = last_sent_cmd_pos + clamp(pos_cmd - last_sent_cmd_pos, -0.03, +0.03);
last_sent_cmd_pos = pos_cmd;
```

### 安全使能两阶段（write 中 safe_zero 块）

```cpp
// Phase 1 (frames_remaining > 20): hw_commands_pos_ = safe_zero_held_pos_
// Phase 2 (frames_remaining ≤ 20): blend held → controller command
// alpha = (blend_frames - remaining + 1) / (blend_frames + 1)
// hw_commands_pos_ = held + alpha * (controller_cmd - held)
```

### URDF 文件生成（arm_bringup.launch.py）

```python
def _write_urdf_to_file(context, *args, **kwargs):
    # 1. 读取所有 LaunchConfiguration
    # 2. subprocess.run xacro → capture stdout
    # 3. 写入 /tmp/robot_description.urdf
```
