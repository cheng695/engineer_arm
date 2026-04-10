#ifndef DJI_MOTOR_HPP
#define DJI_MOTOR_HPP

#include "motor_base.hpp"
#include <cmath>
#include <cstring>
#include <endian.h>

namespace arm_hardware_interface::motor_drivers::Dji
{
    struct Parameters
    {
        double reduction_ratio;
        double torque_constant;
        double feedback_current_max;
        double current_max;
        double encoder_resolution;

        // Coefficients
        double encoder_to_rad;
        double rpm_to_radps;
        double current_to_torque;

        Parameters(double rr, double tc, double fmc, double mc, double er)
            : reduction_ratio(rr), torque_constant(tc), feedback_current_max(fmc), current_max(mc), encoder_resolution(er)
        {
            encoder_to_rad = (2.0 * M_PI) / encoder_resolution;
            rpm_to_radps = (2.0 * M_PI) / (60.0 * reduction_ratio);
            current_to_torque = reduction_ratio * torque_constant * (current_max / feedback_current_max);
        }
    };

    class DjiMotor : public MotorBase
    {
    public:
        DjiMotor(const Parameters& params) : params_(params) {}

        /**
         * @brief Parse feedback from 8-byte DJI CAN frame
         */
        void parse_feedback(const uint8_t data[8])
        {
            // DJI feedback is big-endian
            int16_t angle_raw = (data[0] << 8) | data[1];
            int16_t velocity_raw = (data[2] << 8) | data[3];
            int16_t current_raw = (data[4] << 8) | data[5];
            uint8_t temp = data[6];

            unit_data_.angle_Rad = angle_raw * params_.encoder_to_rad;
            unit_data_.angle_Deg = unit_data_.angle_Rad * 57.29577951308232;
            unit_data_.velocity_Rad = velocity_raw * params_.rpm_to_radps;
            unit_data_.velocity_Rpm = velocity_raw / params_.reduction_ratio;
            unit_data_.torque_Nm = current_raw * params_.current_to_torque;
            unit_data_.temperature_C = (double)temp;

            // Simple multi-turn accumulation could be added here if needed
        }

        // Static factories
        static DjiMotor M3508() {
            return DjiMotor(Parameters(19.0, 0.3, 16384.0, 20.0, 8192.0));
        }
        static DjiMotor M2006() {
            return DjiMotor(Parameters(36.0, 0.18, 16384.0, 10.0, 8192.0));
        }

    private:
        Parameters params_;
    };
}

#endif
