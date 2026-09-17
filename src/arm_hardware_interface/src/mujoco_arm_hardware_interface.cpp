#include "arm_hardware_interface/mujoco_arm_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <mujoco/mujoco.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware_interface
{

namespace
{
constexpr int kNotFound = -1;
}

MujocoArmHardwareInterface::~MujocoArmHardwareInterface()
{
    teardown_internal_node();
    hardware_ready_ = false;
    if (data_)
        mj_deleteData(data_);
    if (model_)
        mj_deleteModel(model_);
}

bool MujocoArmHardwareInterface::load_model(const std::string& model_path)
{
    char error[1024] = {};
    model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!model_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MujocoArmHW"),
            "无法加载 MuJoCo 模型 '%s': %s", model_path.c_str(), error);
        return false;
    }

    data_ = mj_makeData(model_);
    if (!data_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MujocoArmHW"), "无法创建 MuJoCo 仿真数据");
        return false;
    }
    return true;
}

bool MujocoArmHardwareInterface::configure_joints()
{
    mujoco_joint_ids_.resize(info_.joints.size(), kNotFound);
    mujoco_actuator_ids_.resize(info_.joints.size(), kNotFound);
    mujoco_qpos_addresses_.resize(info_.joints.size(), kNotFound);
    mujoco_dof_addresses_.resize(info_.joints.size(), kNotFound);

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const auto& name = info_.joints[i].name;
        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
        if (joint_id == kNotFound)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MujocoArmHW"),
                "MuJoCo 模型中找不到关节 '%s'", name.c_str());
            return false;
        }

        int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
        if (actuator_id == kNotFound)
            actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, (name + "_actuator").c_str());
        if (actuator_id == kNotFound)
            actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, ("actuator_" + name).c_str());
        if (actuator_id == kNotFound)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MujocoArmHW"),
                "MuJoCo 模型中找不到关节 '%s' 对应的 actuator", name.c_str());
            return false;
        }

        mujoco_joint_ids_[i] = joint_id;
        mujoco_actuator_ids_[i] = actuator_id;
        mujoco_qpos_addresses_[i] = model_->jnt_qposadr[joint_id];
        mujoco_dof_addresses_[i] = model_->jnt_dofadr[joint_id];
    }
    return true;
}

hardware_interface::CallbackReturn MujocoArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    init_joint_buffers(info);
    init_joint_limits(info);

    const auto model_it = info.hardware_parameters.find("mujoco_model_path");
    if (model_it == info.hardware_parameters.end() || model_it->second.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("MujocoArmHW"),
            "必须设置 hardware 参数 mujoco_model_path；它应指向 MJCF/XML 文件");
        return CallbackReturn::ERROR;
    }
    model_path_ = model_it->second;

    if (const auto it = info.hardware_parameters.find("mujoco_command_mode");
        it != info.hardware_parameters.end() && !it->second.empty())
        command_mode_ = it->second;
    if (const auto it = info.hardware_parameters.find("mujoco_simulation_steps");
        it != info.hardware_parameters.end() && !it->second.empty())
        simulation_steps_ = std::max(1, std::stoi(it->second));
    if (command_mode_ != "position" && command_mode_ != "velocity" && command_mode_ != "effort")
    {
        RCLCPP_ERROR(rclcpp::get_logger("MujocoArmHW"),
            "不支持的 mujoco_command_mode='%s'", command_mode_.c_str());
        return CallbackReturn::ERROR;
    }

    if (!load_model(model_path_) || !configure_joints())
        return CallbackReturn::ERROR;

    RCLCPP_INFO(rclcpp::get_logger("MujocoArmHW"),
        "MuJoCo 硬件接口初始化完成 (%zu 关节, command_mode=%s, simulation_steps=%d)",
        info_.joints.size(), command_mode_.c_str(), simulation_steps_);
    return CallbackReturn::SUCCESS;
}

void MujocoArmHardwareInterface::setup_internal_node()
{
    internal_node_ = rclcpp::Node::make_shared("mujoco_arm_hw_internal");
    enable_pub_ = internal_node_->create_publisher<std_msgs::msg::Bool>(
        "/arm/command/motor_enable", 10);
    ready_pub_ = internal_node_->create_publisher<std_msgs::msg::Bool>(
        "/arm/state/hardware_ready", 10);
    status_timer_ = internal_node_->create_wall_timer(std::chrono::milliseconds(20), [this] {
        std_msgs::msg::Bool enable;
        enable.data = true;
        enable_pub_->publish(enable);

        std_msgs::msg::Bool ready;
        ready.data = hardware_ready_.load();
        ready_pub_->publish(ready);
    });
    spin_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    spin_executor_->add_node(internal_node_);
    spin_thread_ = std::make_unique<std::thread>([this] { spin_executor_->spin(); });
}

void MujocoArmHardwareInterface::teardown_internal_node()
{
    if (spin_executor_)
        spin_executor_->cancel();
    if (spin_thread_)
    {
        spin_thread_->join();
        spin_thread_.reset();
    }
    spin_executor_.reset();
    status_timer_.reset();
    enable_pub_.reset();
    ready_pub_.reset();
    internal_node_.reset();
}

hardware_interface::CallbackReturn MujocoArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    if (!data_)
        return CallbackReturn::ERROR;
    mj_resetData(model_, data_);
    setup_internal_node();
    update_state_from_mujoco();
    hardware_ready_ = true;
    RCLCPP_INFO(rclcpp::get_logger("MujocoArmHW"), "MuJoCo 仿真已激活，自动使能");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*prev*/)
{
    hardware_ready_ = false;
    teardown_internal_node();
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MujocoArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_eff_[i]);
    }
    return ifaces;
}

std::vector<hardware_interface::CommandInterface>
MujocoArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_vel_[i]);
        ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_eff_[i]);
    }
    return ifaces;
}

void MujocoArmHardwareInterface::update_state_from_mujoco()
{
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        const int qpos_adr = mujoco_qpos_addresses_[i];
        const int dof_adr = mujoco_dof_addresses_[i];
        hw_states_pos_[i] = data_->qpos[qpos_adr];
        hw_states_vel_[i] = data_->qvel[dof_adr];
        hw_states_eff_[i] = data_->qfrc_actuator[dof_adr];
    }
}

hardware_interface::return_type MujocoArmHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!data_ || !hardware_ready_.load())
        return hardware_interface::return_type::ERROR;
    update_state_from_mujoco();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoArmHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!data_ || !hardware_ready_.load())
        return hardware_interface::return_type::ERROR;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        double command = 0.0;
        if (command_mode_ == "position") command = hw_commands_pos_[i];
        if (command_mode_ == "velocity") command = hw_commands_vel_[i];
        if (command_mode_ == "effort") command = hw_commands_eff_[i];
        if (!std::isfinite(command))
            command = command_mode_ == "position" ? hw_states_pos_[i] : 0.0;
        data_->ctrl[mujoco_actuator_ids_[i]] = command;
    }

    for (int step = 0; step < simulation_steps_; ++step)
        mj_step(model_, data_);
    return hardware_interface::return_type::OK;
}

}  // namespace arm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    arm_hardware_interface::MujocoArmHardwareInterface,
    hardware_interface::SystemInterface)
