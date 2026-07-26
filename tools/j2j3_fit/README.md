# J2/J3 耦合拟合工具

这个文件夹专门用来做 J2/J3 耦合关系的采样和拟合。

## 使用前提

先启动机械臂 bringup，并确认硬件接口正在发布调试关节状态：

```text
/arm_debug/raw_motor_states
```

这个 topic 是电机侧原始角度，比较适合你现在“让 Link3 与 Link2 保持相对静止，然后采样 J2/J3 角度”的拟合流程。

## 运行命令

在工作区根目录运行：

```bash
python3 tools/j2j3_fit/j2j3_sampler_fit.py
```

默认拟合模型是：

```text
joint3 = poly(joint2)
```

默认是一阶拟合，也就是：

```text
joint3 = a * joint2 + b
```

如果想做二阶多项式拟合：

```bash
python3 tools/j2j3_fit/j2j3_sampler_fit.py --degree 2
```

如果想改用其他 JointState topic：

```bash
python3 tools/j2j3_fit/j2j3_sampler_fit.py --topic /joint_states
```

如果想用处理后的关节状态做对比：

```bash
python3 tools/j2j3_fit/j2j3_sampler_fit.py --topic /arm_debug/processed_joint_states
```

如果想导入已有 CSV 再拟合，脚本会自动剔除超出 J2/J3 活动范围的点：

```bash
python3 tools/j2j3_fit/j2j3_sampler_fit.py tools/j2j3_fit/data/j2j3_samples.csv --fit-and-exit
```

也可以用命名参数：

```bash
python3 tools/j2j3_fit/j2j3_sampler_fit.py --input-csv tools/j2j3_fit/data/j2j3_samples.csv --fit-and-exit
```

默认活动范围按 V1.1：

```text
joint2: -0.262  ~  1.204 rad
joint3: -0.9599 ~  1.221 rad
```

如果要手动指定范围：

```bash
python3 tools/j2j3_fit/j2j3_sampler_fit.py your.csv --fit-and-exit --x-min -0.262 --x-max 1.204 --y-min -0.9599 --y-max 1.221
```

## 按键操作

脚本运行后，用键盘操作：

```text
空格键    采集一次当前 joint2/joint3 角度
回车键    对当前已采集数据做拟合，并保存 CSV/JSON
s         只保存 CSV，不拟合
q         退出
```

推荐流程：

```text
1. 手动调整机械臂到一个采样姿态
2. 确认 Link3 与 Link2 保持相对静止
3. 按空格键采一次
4. 换下一个姿态，继续按空格采样
5. 采够后按回车键拟合
```

## 输出文件

拟合结果会保存到：

```text
tools/j2j3_fit/data/j2j3_samples.csv
tools/j2j3_fit/data/j2j3_fit.json
```

其中：

```text
j2j3_samples.csv   原始采样数据
j2j3_fit.json      拟合系数、RMSE、最大误差
```

## 常用命令

```bash
# 一阶拟合，使用默认 raw motor topic
python3 tools/j2j3_fit/j2j3_sampler_fit.py --degree 1

# 二阶拟合
python3 tools/j2j3_fit/j2j3_sampler_fit.py --degree 2

# 使用 /joint_states
python3 tools/j2j3_fit/j2j3_sampler_fit.py --topic /joint_states

# 使用处理后的关节状态
python3 tools/j2j3_fit/j2j3_sampler_fit.py --topic /arm_debug/processed_joint_states
```

## IMU CSV 拟合

`j2j3_imu_fk_fit.py` 支持这种 7 列 CSV：

```text
I0,I1,I2,I3,I4,I5,I6
```

其中默认按下面的含义读取：

```text
I1: IMU pitch angle, deg
I2: joint2 / motor2 angle, rad
I3: joint3 / motor3 angle, rad
```

运行：

```bash
python3 tools/j2j3_fit/j2j3_imu_fk_fit.py your.csv
```

默认拟合 J2/J3 解耦多项式，流程是：

```text
joint3_decoupled = raw_joint3 + a3*raw_joint2^3 + a2*raw_joint2^2 + a1*raw_joint2 + a0
fk_pitch_deg = rad2deg(joint3_decoupled - raw_joint2)
loss = fk_pitch_deg - imu_pitch_deg
```

默认优化器是 Levenberg-Marquardt，也可以切换：

```bash
python3 tools/j2j3_fit/j2j3_imu_fk_fit.py your.csv --optimizer lbfgs
python3 tools/j2j3_fit/j2j3_imu_fk_fit.py your.csv --optimizer adam
```

运行后会在终端打印 `r_squared`，结果 JSON 里也会保存 `r_squared`，并默认保存拟合效果图：

```text
tools/j2j3_fit/data/j2j3_imu_fk_fit.png
```

如果想改图的输出路径：

```bash
python3 tools/j2j3_fit/j2j3_imu_fk_fit.py your.csv --plot-png /tmp/j2j3_fit.png
```

如果 IMU 角度列是弧度：

```bash
python3 tools/j2j3_fit/j2j3_imu_fk_fit.py your.csv --imu-pitch-unit rad
```

## Foxglove 预览解耦效果

如果想先用当前拟合系数在线预览解耦效果，不改控制链路，启动 bringup 后运行：

```bash
python3 tools/j2j3_fit/j2j3_motor_pitch_preview.py
```

脚本默认订阅：

```text
/arm_debug/raw_motor_states
```

默认发布：

```text
/arm_debug/j2j3_fit_pitch_deg
/arm_debug/j2j3_fit_preview
/arm_debug/j2j3_decoupled_preview
```

在 Foxglove 的 3D 面板里，把 JointState 数据源切到：

```text
/arm_debug/j2j3_decoupled_preview
```

这个话题会复制 `/arm_debug/raw_motor_states`，并把 `joint3` 替换为：

```text
joint3_decoupled = raw_joint3 + a3*raw_joint2^3 + a2*raw_joint2^2 + a1*raw_joint2 + a0
```

当前预览脚本默认系数：

```text
joint3_decoupled = raw_joint3 + 0.986 * raw_joint2
```

如果要使用拟合出的系数临时预览：

```bash
python3 tools/j2j3_fit/j2j3_motor_pitch_preview.py --coeff A3 A2 A1 A0
```

## 尝试当前公式

当前控制参数已经可以表达这个公式：

```text
J3 = (M3 + 0.986 * J2 - offset) / 0.82
```

对应配置在：

```text
src/my_robot_bringup/config/control_gains.yaml
```

参数是：

```yaml
j2j3_coupling: 0.986
j2j3_j3_scale: 0.82
j2j3_j3_offset: 0.0
```

修改参数后，重新启动 bringup，再观察 `/joint_states` 或 `/arm_debug/processed_joint_states` 里的 joint3 效果。

## 注意

如果只是校准 J2 对 J3 的耦合系数，建议先用一阶拟合。只有当一阶拟合误差明显随角度变化时，再尝试二阶或更高阶。
