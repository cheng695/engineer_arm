#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP

#include <cstdint>
#include <vector>
#include <string>

namespace arm_hardware_interface::motor_drivers
{
    /**
     * @brief Runtime state snapshot of a motor.
     */
    struct MotorState
    {
        double angle_Deg = 0.0;
        double angle_Rad = 0.0;
        double velocity_Rad = 0.0;
        double velocity_Rpm = 0.0;
        double current_A = 0.0;
        double torque_Nm = 0.0;
        double temperature_C = 0.0;

        double last_angle = 0.0;
        double add_angle = 0.0;
        uint8_t error_code = 0;
    };

    /**
     * @brief Hardware configuration and tuning parameters for a motor.
     */
    struct MotorConfig
    {
        uint32_t can_id = 0;
        std::string bus_name = "can1";
        float kp = 0.0f;
        float kd = 0.3f;
    };

    /**
     * @brief Base class for motors, representing a single hardware resource.
     */
    class MotorBase
    {
    protected:
        MotorState state_;
        MotorConfig config_;
        bool is_enabled_ = false;

    public:
        MotorBase() = default;
        virtual ~MotorBase() = default;

        // Abstract methods for motor-specific protocol logic
        virtual void get_enable_command(uint8_t data[8]) = 0;
        virtual void get_disable_command(uint8_t data[8]) = 0;
        virtual void get_clear_errors_command(uint8_t data[8]) = 0;
        virtual void pack_mit_command(float pos, float vel, float kp, float kd, float torque, uint8_t data[8]) = 0;
        virtual void parse_feedback(const uint8_t data[8]) = 0;

        // Accessors
        MotorConfig& config() { return config_; }
        const MotorConfig& config() const { return config_; }
        
        MotorState& state() { return state_; }
        const MotorState& state() const { return state_; }

        bool isEnabled() const { return is_enabled_; }
        void setEnabled(bool enabled) { is_enabled_ = enabled; }

        // Helper accessors for convenience in the interface loop
        uint32_t getCanId() const { return config_.can_id; }
        void setCanId(uint32_t id) { config_.can_id = id; }

        std::string getBusName() const { return config_.bus_name; }
        void setBusName(const std::string& name) { config_.bus_name = name; }

        float getKp() const { return config_.kp; }
        void setKp(float kp) { config_.kp = kp; }

        float getKd() const { return config_.kd; }
        void setKd(float kd) { config_.kd = kd; }
        
        double getAngleRad() const { return state_.angle_Rad; }
        double getVelocityRad() const { return state_.velocity_Rad; }
        double getTorqueNm() const { return state_.torque_Nm; }
        uint8_t getErrorCode() const { return state_.error_code; }
    };
}

#endif
