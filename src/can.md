sudo ip link set down can0
sudo ip link set down can1
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set up can1
ip -details link show can0
ip -details link show can1


ros2 launch my_robot_bringup robot.launch.py use_mock_hardware:=true

ros2 launch my_robot_bringup robot.launch.py use_mock_hardware:=false


ros2 launch my_robot_commander_cpp commander.launch.py

ros2 launch my_robot_commander_cpp commander.launch.py use_servo:=false


pkill -9 -f commander; pkill -9 -f joy_node; pkill -9 -f ros2_comtrol; pkill -9 -f robot_state; pkill -9 -f rviz; pkill -9 -f move_group; pkill -9 -f spawner; pkill -9 -f controller_manager; sleep 2
