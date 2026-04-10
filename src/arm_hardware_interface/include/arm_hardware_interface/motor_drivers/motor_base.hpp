#ifndef MOTOR_BASE_HPP
#define MOTOR_BASE_HPP

#include <cstdint>
#include <vector>
#include <string>

namespace arm_hardware_interface::motor_drivers
{
    /**
     * @brief Base class for motors, containing common data structures.
     * Refactored from embedded version to be hardware-agnostic.
     */
    class MotorBase
    {
    protected:
        struct UnitData
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
        };

        UnitData unit_data_;
        bool is_Enabled = false;
        uint32_t can_id_ = 0;
        std::string bus_name_ = "can1";
        float kp_ = 50.0f; // Default Kp
        float kd_ = 1.0f;  // Default Kd

    public:
        MotorBase() = default;
        virtual ~MotorBase() = default;

        // Abstract methods for motor-specific protocol logic
        virtual void get_enable_command(uint8_t data[8]) = 0;
        virtual void get_disable_command(uint8_t data[8]) = 0;
        virtual void pack_mit_command(float pos, float vel, float kp, float kd, float torque, uint8_t data[8]) = 0;
        virtual void parse_feedback(const uint8_t data[8]) = 0;

        // Common getters
        uint32_t getCanId() const { return can_id_; }
        void setCanId(uint32_t id) { can_id_ = id; }

        std::string getBusName() const { return bus_name_; }
        void setBusName(const std::string& name) { bus_name_ = name; }

        float getKp() const { return kp_; }
        void setKp(float kp) { kp_ = kp; }

        float getKd() const { return kd_; }
        void setKd(float kd) { kd_ = kd; }
        double getAngleDeg() const { return unit_data_.angle_Deg; }
        double getAngleRad() const { return unit_data_.angle_Rad; }
        double getVelocityRad() const { return unit_data_.velocity_Rad; }
        double getVelocityRpm() const { return unit_data_.velocity_Rpm; }
        double getCurrent() const { return unit_data_.current_A; }
        double getTorque() const { return unit_data_.torque_Nm; }
        double getTemperature() const { return unit_data_.temperature_C; }

        bool isEnabled() const { return is_Enabled; }
        void setEnabled(bool enabled) { is_Enabled = enabled; }
    };
}

#endif
