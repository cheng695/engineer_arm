sudo ip link set down can0
sudo ip link set down can1
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set up can1
ip -details link show can0
ip -details link show can1

V1.0
ros2 launch my_robot_bringup robot.launch.py use_mock_hardware:=true robot:=v1.0
ros2 launch my_robot_bringup robot.launch.py use_mock_hardware:=false robot:=v1.0


ros2 launch my_robot_commander_cpp commander.launch.py robot:=v1.0
ros2 launch my_robot_commander_cpp commander.launch.py use_servo:=false robot:=v1.0

V1.1
ros2 launch my_robot_bringup robot.launch.py use_mock_hardware:=true robot:=v1.1
ros2 launch my_robot_bringup robot.launch.py use_mock_hardware:=false robot:=v1.1

ros2 launch my_robot_commander_cpp commander.launch.py robot:=v1.1
ros2 launch my_robot_commander_cpp commander.launch.py use_servo:=false robot:=v1.1

MuJoCo V1.1 仿真
# MuJoCo 安装位置：/home/whc/mujoco-3.13.0
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --packages-select arm_hardware_interface --symlink-install \
  --cmake-clean-cache --cmake-args \
  -DMUJOCO_ROOT=/home/whc/mujoco-3.13.0

# 完整启动：MuJoCo、ros2_control、RViz、MoveIt 和手柄
ros2 launch my_robot_bringup robot.launch.py \
  robot:=v1.1 \
  use_mujoco_hardware:=true \
  use_mock_hardware:=false \
  mujoco_model_path:=$PWD/sim/mujoco/my_robot_v1_1.xml

# 不启动手柄时，可加：start_commander:=false start_joy:=false

pkill -9 -f commander; pkill -9 -f joy_node; pkill -9 -f ros2_comtrol; pkill -9 -f robot_state; pkill -9 -f rviz; pkill -9 -f move_group; pkill -9 -f spawner; pkill -9 -f controller_manager; sleep 2


source install/setup.bash
ros2 launch my_robot_description display.launch.py robot:=v1.1

ros2 launch my_robot_bringup robot.launch.py   use_mock_hardware:=false   robot:=v1.1   active_real_joints:=joint1,joint2,joint3,joint4

ros2 launch foxglove_bridge foxglove_bridge_launch.xml

/joint_states.position[1]                       # J2 解耦后反馈
/joint_states.position[2]                       # J3 解耦后反馈
/arm_debug/raw_motor_states.position[1]         # J2 原始电机反馈
/arm_debug/raw_motor_states.position[2]         # J3 原始电机反馈
/arm_debug/joint_target_states.position[1]      # J2 JOINT期望
/arm_debug/joint_target_states.position[2]      # J3 JOINT期望
/arm_debug/dls_target_states.position[1]        # J2 DLS期望
/arm_debug/dls_target_states.position[2]        # J3 DLS期望
/arm_controller/controller_state.reference.positions[2] # POS/JTC的J3期望


ros2 launch my_robot_bringup robot.launch.py   use_mock_hardware:=false   robot:=v1.1   gravity_compensation_mode:=external_gravity_only   gravity_effort_scale:=1.1
