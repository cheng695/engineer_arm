#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sensor_msgs/msg/joy.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_msgs/msg/bool.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using StringMsg = example_interfaces::msg::String;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using Joy = sensor_msgs::msg::Joy;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using JointJog = control_msgs::msg::JointJog;
using BoolMsg = std_msgs::msg::Bool;
using Int8 = std_msgs::msg::Int8;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using namespace std::placeholders;

// Commander: 手柄输入 -> MoveIt规划 / Servo实时控制 的统一控制器
class Commander
{   
    public:
        Commander(std::shared_ptr<rclcpp::Node> node)
        {   
            node_ = node;
            RCLCPP_INFO(node_->get_logger(), "[BOOT] V5.2 Commander 启动完成");

            // 两个规划组：机械臂 + 夹爪
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

            // 规划场景监视器（用于Servo获取状态/场景）
            psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
            
            // 读取 moveit_servo 参数，先尝试命名空间 "moveit_servo"，失败则回退默认
            servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_, "moveit_servo");
            if (!servo_parameters_) 
            {
                servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_);
            }

            // 等待机器人模型就绪，避免后续节点初始化失败
            int retry = 0;
            while (rclcpp::ok() && !psm_->getRobotModel() && retry < 20) 
            {
                RCLCPP_INFO(node_->get_logger(), "[INIT] 等待机器人模型就绪... (%d)", retry+1);
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                retry++;
            }

            if (psm_->getRobotModel() && servo_parameters_) 
            {
                // 启动状态、场景、世界几何监控
                psm_->startStateMonitor(servo_parameters_->joint_topic);
                psm_->startSceneMonitor(servo_parameters_->monitored_planning_scene_topic);
                psm_->startWorldGeometryMonitor();
                psm_->getStateMonitor()->enableCopyDynamics(true);

                // 启动Servo主循环
                startServo();

                // 实时控制输出：笛卡尔速度 / 关节速度
                twist_pub_ = node_->create_publisher<TwistStamped>(servo_parameters_->cartesian_command_in_topic, 10);
                joint_pub_ = node_->create_publisher<JointJog>(servo_parameters_->joint_command_in_topic, 10);

                // 调试订阅：可用于观察Servo输出轨迹
                debug_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                    servo_parameters_->command_out_topic, 10,
                    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) 
                    {
                        // 这里只做“是否有有效输出”的观察点，不参与控制闭环。
                        // 如需排查抖动/限速问题，可在此打印每个关节的 velocity / position。
                        if (!msg->points.empty() && !msg->points[0].velocities.empty()) 
                        {
                            // double v0 = msg->points[0].velocities[0];
                        }
                    });

            } 
            else 
            {
                RCLCPP_ERROR(node_->get_logger(), "[INIT][ERROR] 初始化失败");
            }

            // 电机使能控制 + 手柄订阅
            motor_enable_pub_ = node_->create_publisher<BoolMsg>("/arm_motor_enable", 10);
            joy_sub_ = node_->create_subscription<Joy>("joy", 10, std::bind(&Commander::joyCallback, this, _1));

        }

    private:
        // 电机扭矩总使能状态（由手柄Guide键翻转）
        bool motor_enabled_{false};
        bool prev_guide_pressed_{false};
        rclcpp::Publisher<BoolMsg>::SharedPtr motor_enable_pub_;

        // 创建并启动Servo实例
        void startServo() 
        {
            if (servo_) return;
            try 
            {
                servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters_, psm_);
                servo_->start();
                RCLCPP_INFO(node_->get_logger(), "[SERVOMAN] Servo实例已启动");
            } 
            catch (const std::exception& e) 
            {
                RCLCPP_ERROR(node_->get_logger(), "[SERVOMAN][ERROR] 启动失败: %s", e.what());
            }
        }

        // 关闭Servo实例（用于规划轨迹执行期间避免回弹/冲突）
        void stopServo() 
        {
            if (servo_) 
            {
                servo_->setPaused(true);
                servo_.reset();
                RCLCPP_INFO(node_->get_logger(), "[SERVOMAN] Servo实例已停止");
            }
        }

        // 规划并异步执行：执行期间置busy，完成后恢复Servo
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
        {
            MoveGroupInterface::Plan plan;
            if(interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) 
            {
                is_busy_ = true;
                
                // ONLY reset Servo if the Arm is moving (to prevent snap-back)
                // Gripper is independent and doesn't need to kill the arm engine.
                bool is_arm = (interface == arm_);
                if (is_arm) stopServo(); 
                
                RCLCPP_INFO(node_->get_logger(), "[ASYNC] 开始执行轨迹 (%s)...", is_arm ? "ARM" : "GRIPPER");
                
                // 放到独立线程执行，避免阻塞joy回调和主执行器
                std::thread([this, interface, plan, is_arm]() 
                {
                    auto result = interface->execute(plan); 
                    if (result == moveit::core::MoveItErrorCode::SUCCESS) 
                    {
                        RCLCPP_INFO(node_->get_logger(), "[SETTLE] 运动执行成功");
                    } 
                    else 
                    {
                        RCLCPP_ERROR(node_->get_logger(), "[ABORT][ERROR] 运动中断或失败, code=%d", result.val);
                    }
                    
                    if (is_arm) 
                    {
                        // 给控制器/状态同步留一点余量，再恢复Servo
                        rclcpp::sleep_for(std::chrono::milliseconds(1200)); 
                        psm_->requestPlanningSceneState();
                        startServo();
                    }
                    
                    is_busy_ = false;
                    RCLCPP_INFO(node_->get_logger(), "[READY] 系统恢复可控状态");
                }).detach();
            } 
            else 
            {
                RCLCPP_WARN(node_->get_logger(), "[PLAN][WARN] 规划失败");
            }
        }

        // 手柄回调：集中处理按键事件 + 实时速度控制
        void joyCallback(const Joy &msg)
        {
            auto now = node_->now();
            auto msg_time = rclcpp::Time(msg.header.stamp);

            // 丢弃过旧输入，减少延迟/卡顿导致的误动作
            if ((now - msg_time).seconds() > 0.5) return; 

            // Xbox Guide键(Button 8)：切换电机扭矩使能
            bool guide_pressed = (msg.buttons.size() > 8 && msg.buttons[8]);
            if (guide_pressed && !prev_guide_pressed_) 
            {
                // 采用“上升沿触发”：避免长按时每帧重复翻转。
                motor_enabled_ = !motor_enabled_;
                auto enable_msg = std::make_unique<BoolMsg>();
                enable_msg->data = motor_enabled_;
                motor_enable_pub_->publish(std::move(enable_msg));
                RCLCPP_WARN(node_->get_logger(), "[POWER] 电机扭矩状态: %s", motor_enabled_ ? "ENABLE(使能)" : "DISABLE(失能)");
            }
            prev_guide_pressed_ = guide_pressed;

            // RT 触发检测（兼容轴值/按键两种手柄映射）
            bool rt_pressed = false;
            if (msg.axes.size() > 5 && msg.axes[5] < 0.7) rt_pressed = true;
            if (msg.buttons.size() > 7 && msg.buttons[7]) rt_pressed = true;

            // RT上升沿：急停当前轨迹 + 切换手动暂停
            if (rt_pressed && !prev_rt_pressed_) // 判断RT的上升沿
            {
                RCLCPP_WARN(node_->get_logger(), "[STOP][WARN] 检测到中断触发");
                if (is_busy_) // 如果在前往固定位置
                {
                    RCLCPP_ERROR(node_->get_logger(), "[STOP][ERROR] 强制中止所有运动");
                    arm_->stop();   
                    gripper_->stop();
                }
                manual_paused_ = !manual_paused_;
                if (manual_paused_)
                {
                    RCLCPP_WARN(node_->get_logger(), "[LOCK] 输入已锁定(暂停)");
                    if (servo_) servo_->setPaused(true);    // 暂停servo
                } 
                else 
                {
                    RCLCPP_INFO(node_->get_logger(), "[UNLOCK] 输入已解锁(恢复控制)");
                    if (servo_) servo_->setPaused(false);   // 恢复servo
                }
            }
            prev_rt_pressed_ = rt_pressed;

            // 模式切换：笛卡尔模式 <-> 全关节空间模式 这里只写日志和状态切换判断，具体控制在下面
            bool r3_toggle = (msg.buttons.size() > 10 && msg.buttons[12]);
            if (r3_toggle && !prev_r3_pressed_) // R3上升沿触发模式切换
            {
                global_joint_mode_ = !global_joint_mode_;
                if (global_joint_mode_) 
                {
                    RCLCPP_WARN(node_->get_logger(), "[MODE] 已切换到关节空间模式");
                } 
                else 
                {
                    RCLCPP_INFO(node_->get_logger(), "[MODE] 已切换到笛卡尔空间模式");
                }
            }
            prev_r3_pressed_ = r3_toggle;

            // 忙碌或手动暂停时，不接受新的控制输入
            // is_busy_: 正在执行规划轨迹；manual_paused_: 用户主动锁定输入。
            if(is_busy_ || manual_paused_) return; 

            // 离散按键动作（规划到命名位姿）
            if(msg.buttons.size() > 5) 
            {
                // 这些动作属于“规划执行路径”，不是Servo连续速度控制路径。
                if(msg.buttons[0]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("home"); planAndExecute(arm_); return; }
                if(msg.buttons[1]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("right"); planAndExecute(arm_); return; }
                if(msg.buttons[3]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("left"); planAndExecute(arm_); return; }
                if(msg.buttons[2]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("up"); planAndExecute(arm_); return; }
                if(msg.buttons[10]) { arm_->setStartStateToCurrentState(); arm_->setNamedTarget("through"); planAndExecute(arm_); return; }
                if(msg.buttons[4]) { gripper_->setStartStateToCurrentState(); gripper_->setNamedTarget("open"); planAndExecute(gripper_); return; }
                if(msg.buttons[5]) { gripper_->setStartStateToCurrentState(); gripper_->setNamedTarget("close"); planAndExecute(gripper_); return; }
            }

            // 摇杆输入预处理：后面的读取都是调用这个函数
            // 1) 设置 deadzone=0.12，消除中位抖动（小幅输入直接归零）
            // 2) 对超出死区的输入重新线性归一化到 [-1, 1]，保证满量程手感不变
            auto stick_val = [](double val) 
            { 
                const double deadzone = 0.12; 
                if (std::abs(val) < deadzone) return 0.0;
                return (val - (val > 0 ? deadzone : -deadzone)) / (1.0 - deadzone);
            };

            // Servo异常时直接退出，避免空指针发布
            if (!servo_) return;

            // 工具坐标系模式：LT/按钮触发后，twist使用末端坐标系
            // 直观效果：同样的手柄方向，在末端坐标系下会“跟工具朝向走”。
            bool tool_mode = (msg.axes.size() > 2 && msg.axes[2] < 0.7) ||
                             (msg.buttons.size() > 6 && msg.buttons[6]);
            
            // 关节空间Jog模式
            if (global_joint_mode_) 
            {
                auto joint_msg = std::make_unique<JointJog>();
                joint_msg->header.stamp = node_->now();
                joint_msg->header.frame_id = "base_link";
                joint_msg->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
                
                // L3按住作为上下文修饰：RX在J4/J7间切换
                bool l3_hold = (msg.buttons.size() > 9 && msg.buttons[11]);

                // J1, J2, J3 (Inverted)
                joint_msg->velocities.push_back(-stick_val(msg.axes[0])); // J1 (LX)
                joint_msg->velocities.push_back(-stick_val(msg.axes[1])); // J2 (LY)
                joint_msg->velocities.push_back(-stick_val(msg.axes[4])); // J3 (RY)
                
                // J4 (RX) - Restricted if L3 is held (Inverted)
                joint_msg->velocities.push_back(l3_hold ? 0.0 : -stick_val(msg.axes[3]));
                
                // J5, J6 改为读取 D-Pad buttons（顺序：上、下、左、右）
                // 注意：不同手柄/驱动索引可能不同，换手柄时优先检查 /joy。
                // 当前映射：up=13, down=14, left=15, right=16
                const double dpad_up = (msg.buttons.size() > 13 && msg.buttons[13]) ? 1.0 : 0.0;
                const double dpad_down = (msg.buttons.size() > 14 && msg.buttons[14]) ? 1.0 : 0.0;
                const double dpad_left = (msg.buttons.size() > 15 && msg.buttons[15]) ? 1.0 : 0.0;
                const double dpad_right = (msg.buttons.size() > 16 && msg.buttons[16]) ? 1.0 : 0.0;
                joint_msg->velocities.push_back(stick_val(dpad_up - dpad_down));     // J5
                joint_msg->velocities.push_back(stick_val(dpad_right - dpad_left));  // J6
                
                // J7 (RX if L3 held) (Inverted)
                joint_msg->velocities.push_back(l3_hold ? -stick_val(msg.axes[3]) : 0.0);

                if (joint_pub_) // 判断指针是否有效
                {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[TELEMETRY] 关节空间Jog控制中");
                    joint_pub_->publish(std::move(joint_msg)); // 发布关节空间Jog控制
                }
            } 
            else // 笛卡尔模式
            {
                // 笛卡尔Twist模式（base_link或末端坐标系）
                auto twist_msg = std::make_unique<TwistStamped>();
                twist_msg->header.stamp = node_->now();
                twist_msg->header.frame_id = tool_mode ? "gripper_center" : "base_link";

                // 线速度：xyz；
                double lx = stick_val(msg.axes[1]); 
                double ly = stick_val(msg.axes[0]);
                double lz = (msg.axes.size() > 4) ? stick_val(msg.axes[4]) : 0.0;

                // 笛卡尔旋转改为D-Pad输入：上下->pitch(ay), 左右->yaw(az)
                // 当前映射：up=13, down=14, left=15, right=16
                const double dpad_up = (msg.buttons.size() > 13 && msg.buttons[13]) ? 1.0 : 0.0;
                const double dpad_down = (msg.buttons.size() > 14 && msg.buttons[14]) ? 1.0 : 0.0;
                const double dpad_left = (msg.buttons.size() > 15 && msg.buttons[15]) ? 1.0 : 0.0;
                const double dpad_right = (msg.buttons.size() > 16 && msg.buttons[16]) ? 1.0 : 0.0;
                double ax = (msg.axes.size() > 3) ? -stick_val(msg.axes[3]) : 0.0; // roll
                double ay = stick_val(dpad_up - dpad_down);     // pitch
                double az = stick_val(dpad_right - dpad_left);  // yaw

                twist_msg->twist.linear.x = lx; 
                twist_msg->twist.linear.y = ly; 
                twist_msg->twist.linear.z = lz; 
                twist_msg->twist.angular.x = ax; 
                twist_msg->twist.angular.y = ay; 
                twist_msg->twist.angular.z = az; 

                if (std::abs(lx)>0.01 || std::abs(lz)>0.01) 
                {
                    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "[TELEMETRY] 笛卡尔轴向控制中");
                }
                if(twist_pub_) twist_pub_->publish(std::move(twist_msg)); // 发布笛卡尔速度控制
            }
        }

        // ---------- 成员变量 ----------
        std::shared_ptr<rclcpp::Node> node_;            // ROS2 节点句柄：用于日志、时间、创建通信实体
        std::shared_ptr<MoveGroupInterface> arm_;       // MoveIt 机械臂规划接口（arm 组）
        std::shared_ptr<MoveGroupInterface> gripper_;   // MoveIt 夹爪规划接口（gripper 组）
        std::atomic<bool> is_busy_{false};  // 轨迹执行忙碌标志（异步线程与回调线程共享）
        bool manual_paused_{false};         // 手动暂停标志：RT 切换，true 时屏蔽控制输入
        bool prev_rt_pressed_{false};       // RT 上一帧状态：用于上升沿检测
        bool global_joint_mode_{false};     // 模式标志：false=笛卡尔，true=关节空间
        bool prev_r3_pressed_{false};       // R3 上一帧状态：用于模式切换上升沿检测
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;  // 规划场景监视器：提供状态/场景/世界几何
        std::unique_ptr<moveit_servo::Servo> servo_;  // Servo 实例：实时速度控制核心
        std::shared_ptr<const moveit_servo::ServoParameters> servo_parameters_;  // Servo 参数快照
        rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;  // 笛卡尔命令发布器（TwistStamped）
        rclcpp::Publisher<JointJog>::SharedPtr joint_pub_;      // 关节命令发布器（JointJog）
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr debug_sub_;  // 调试订阅器：观察 Servo 输出轨迹
        rclcpp::Subscription<Joy>::SharedPtr joy_sub_;  // 手柄订阅器：接收 /joy 并触发 joyCallback
};

int main(int argc, char** argv)
{
    // 多线程执行器：确保回调、规划执行、Servo相关任务能并行调度
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    rclcpp::executors::MultiThreadedExecutor executor;
    auto commander = Commander(node);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
