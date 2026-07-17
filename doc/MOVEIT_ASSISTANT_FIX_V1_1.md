# MoveIt Setup Assistant V1.1 加载问题处理记录

## 问题现象

在 MoveIt Setup Assistant 中选择或手动输入 V1.1 URDF 后，界面出现以下问题：

- 输入框看起来为空或没有正常加载。
- 点击加载后 GUI 无响应。
- 终端进程卡住，需要手动 kill。

最初尝试的 URDF 包括：

```text
/home/whc/engineer_whc/my_robot_v1_1_lightweight.urdf
/home/whc/engineer_whc/src/V1.1urdf/robot.urdf
```

这些文件本身并不一定是坏的，`check_urdf` 可以解析通过，但 MoveIt Setup Assistant 仍然可能在 GUI/RobotModel/self-collision 构建阶段卡住。

## 原因总结

主要有两个原因。

第一，MoveIt Setup Assistant 的 `--urdf_path` 更适合加载 ROS package 内的 URDF。直接传 workspace 根目录或普通目录下的 URDF，Assistant 在推断 package、相对路径和资源路径时可能行为异常。

第二，完整 V1.1 模型包含大量 STL mesh、底盘、轮子、夹爪、mimic 关节和复杂固定分支。即使 URDF 语法正确，Assistant 在加载可视模型或生成 self-collision matrix 时也可能卡死。

## 排查过程

先用命令确认 URDF 语法正确：

```bash
source /opt/ros/humble/setup.bash
check_urdf /home/whc/engineer_whc/my_robot_v1_1_lightweight.urdf
check_urdf /home/whc/engineer_whc/src/V1.1urdf/robot.urdf
```

然后确认 Assistant 支持命令行传 URDF：

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant --help
```

其中 `--urdf_path` 的说明是：

```text
Optional, path to URDF file in ROS package
```

所以最终采用 package 内安装路径，而不是直接使用包外路径。

## 解决方案

### 1. 创建 Assistant 专用简化 URDF

为了让 Assistant 稳定打开，创建了一个只包含机械臂主体的简化 URDF：

```text
/home/whc/engineer_whc/src/arm_description/urdf/my_robot_v1_1_moveit_assistant.urdf
```

它只保留：

```text
base_link -> Link_1 -> Link_2 -> Link3 -> Link4 -> Link5 -> Link6 -> Link7 -> tool_link
```

去掉了底盘、轮子、夹爪 mimic 和多余固定分支。

为了在 Assistant/RViz 中能看到简化外形，又把每个 link 的 collision box 复制成 visual box，没有引入 STL mesh。

### 2. 安装到 ROS package 中

`arm_description` 的 `CMakeLists.txt` 已经安装 `urdf` 目录：

```cmake
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
```

重新编译：

```bash
cd /home/whc/engineer_whc
source /opt/ros/humble/setup.bash
colcon build --packages-select arm_description
source install/setup.bash
```

然后用安装路径打开 Assistant：

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant \
  --debug \
  --urdf_path /home/whc/engineer_whc/install/arm_description/share/arm_description/urdf/my_robot_v1_1_moveit_assistant.urdf
```

这个方案可以稳定加载。

### 3. 完整 V1.1 URDF 的准确性修正

一开始临时创建过一个完整模型：

```text
/home/whc/engineer_whc/install/arm_description/share/arm_description/urdf/v1_1_full_robot.urdf
```

后来确认它不能直接从 `src/V1.1urdf/robot.urdf` 复制，因为那不是工程当前的权威模型。

权威 V1.1 模型应该由这些 xacro 组合生成：

```text
src/arm_description/urdf/variants/V1.1/arm_links_v1_1.xacro
src/arm_description/urdf/variants/V1.1/arm_joints_v1_1.xacro
src/chassis_description/urdf/variants/V1.1/chassis_v1_1.xacro
src/gripper_description/urdf/variants/V1.1/gripper_v1_1.xacro
src/my_robot_description/urdf/my_robot.urdf.xacro
```

因此重新生成：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

xacro src/my_robot_description/urdf/my_robot.urdf.xacro \
  arm_version:=v1_1 \
  chassis_version:=v1_1 \
  gripper_version:=v1_1 \
  use_mock_hardware:=true \
  robot_description_file:=/tmp/v1_1_full_robot.urdf \
  -o src/arm_description/urdf/v1_1_full_robot.urdf

colcon build --packages-select arm_description
source install/setup.bash
```

最终确认 install 版和 src 版完全一致。

## 生成 MoveIt V1.1 配置

Assistant 生成的临时包路径：

```text
/home/whc/engineer_whc/src/my_robot_moveit_config_v1_1_generated
```

不要直接覆盖现有：

```text
/home/whc/engineer_whc/src/my_robot_moveit_config
```

而是把有用内容合并到：

```text
/home/whc/engineer_whc/src/my_robot_moveit_config/config/variants/V1.1
```

合并策略：

- `my_robot.srdf` 使用 Assistant 新生成的 V1.1 版本。
- `joint_limits.yaml` 按 V1.1 URDF 手写完整 position/velocity limits。
- `kinematics.yaml` 只保留 `arm` 的 KDL IK。
- controller、Servo、RViz、Pilz 配置沿用现有工程配置。
- `sensors_3d.yaml` 设置为 `sensors: []`，因为没有配置 3D perception。

## Assistant 页面填写要点

End Effector 页面推荐：

```text
End Effector Name: gripper_ee
End Effector Group: gripper
Parent Link: Link7
Parent Group: arm
```

Passive Joints 页面：

```text
保持为空
```

Perception 页面：

```text
None
```

Author Information：

```text
Name: whc
Email: whc@qq.com
```

Configuration Files 页面：

先生成到临时包：

```text
/home/whc/engineer_whc/src/my_robot_moveit_config_v1_1_generated
```

再手动合并到 variant 目录。

## 如果 Assistant 再次卡住

先查看进程：

```bash
pgrep -af moveit_setup_assistant
```

正常终止：

```bash
kill <pid>
```

如果无响应再强制终止：

```bash
kill -9 <pid>
```

如果怀疑是 Qt 问题，可以尝试：

```bash
QT_QPA_PLATFORM=xcb ros2 run moveit_setup_assistant moveit_setup_assistant \
  --debug \
  --urdf_path /home/whc/engineer_whc/install/arm_description/share/arm_description/urdf/my_robot_v1_1_moveit_assistant.urdf
```

## 当前推荐做法

生成/维护 MoveIt 配置时，优先使用：

```text
/home/whc/engineer_whc/install/arm_description/share/arm_description/urdf/my_robot_v1_1_moveit_assistant.urdf
```

检查完整机器人外观或完整模型结构时，使用：

```text
/home/whc/engineer_whc/install/arm_description/share/arm_description/urdf/v1_1_full_robot.urdf
```

不要把 `src/V1.1urdf/robot.urdf` 当作当前工程的权威 MoveIt 输入；当前工程权威模型是由 `my_robot.urdf.xacro` 组合 V1.1 arm/chassis/gripper 生成的。

