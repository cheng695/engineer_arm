#ifndef DM_MOTOR_HPP
#define DM_MOTOR_HPP

#include "motor_base.hpp"
#include <cmath>
#include <cstring>
#include <array>

namespace arm_hardware_interface::motor_drivers::DM
{
    struct Parameters
    {
        float P_MIN, P_MAX;
        float V_MIN, V_MAX;
        float T_MIN, T_MAX;
        float KP_MIN, KP_MAX;
        float KD_MIN, KD_MAX;

        static constexpr double rad_to_deg = 57.29577951308232;

        Parameters(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax,
                   float kpmin, float kpmax, float kdmin, float kdmax)
            : P_MIN(pmin), P_MAX(pmax), V_MIN(vmin), V_MAX(vmax),
              T_MIN(tmin), T_MAX(tmax), KP_MIN(kpmin), KP_MAX(kpmax), KD_MIN(kdmin), KD_MAX(kdmax) {}
    };

    class DmMotor : public MotorBase
    {
    public:
        DmMotor(const Parameters& params) : params_(params) {}
        virtual ~DmMotor() = default;

        /**
         * @brief Pack MIT mode control command into 8-byte buffer
         */
        void pack_mit_command(float pos, float vel, float kp, float kd, float torque, uint8_t data[8])
        {
            uint16_t pos_tmp = float_to_uint(pos, params_.P_MIN, params_.P_MAX, 16);
            uint16_t vel_tmp = float_to_uint(vel, params_.V_MIN, params_.V_MAX, 12);
            uint16_t kp_tmp = float_to_uint(kp, params_.KP_MIN, params_.KP_MAX, 12);
            uint16_t kd_tmp = float_to_uint(kd, params_.KD_MIN, params_.KD_MAX, 12);
            uint16_t tor_tmp = float_to_uint(torque, params_.T_MIN, params_.T_MAX, 12);

            data[0] = (pos_tmp >> 8);
            data[1] = (pos_tmp & 0xFF);
            data[2] = (vel_tmp >> 4);
            data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
            data[4] = (kp_tmp & 0xFF);
            data[5] = (kd_tmp >> 4);
            data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
            data[7] = (tor_tmp & 0xFF);
        }

        /**
         * @brief Parse feedback from 8-byte CAN frame
         */
        void parse_feedback(const uint8_t data[8])
        {
            // Byte 0: [ID: 0-3 bit] [Status/Error: 4-7 bit]
            state_.error_code = (data[0] >> 4);
            
            // Extract raw bits
            uint16_t p_int = (data[1] << 8) | data[2];
            uint16_t v_int = (data[3] << 4) | (data[4] >> 4);
            uint16_t t_int = ((data[4] & 0x0F) << 8) | data[5];

            // Convert to floating point
            state_.angle_Rad = uint_to_float(p_int, params_.P_MIN, params_.P_MAX, 16);
            state_.angle_Deg = state_.angle_Rad * params_.rad_to_deg;
            state_.velocity_Rad = uint_to_float(v_int, params_.V_MIN, params_.V_MAX, 12);
            state_.torque_Nm = uint_to_float(t_int, params_.T_MIN, params_.T_MAX, 12);
            state_.temperature_C = (double)data[6];
        }

        /**
         * @brief Get Enable command frame (DAMIAO MIT Mode)
         */
        void get_enable_command(uint8_t data[8])
        {
            for (int i = 0; i < 7; i++) data[i] = 0xFF;
            data[7] = 0xFC;
        }

        /**
         * @brief Get Disable command frame (DAMIAO MIT Mode)
         */
        void get_disable_command(uint8_t data[8])
        {
            for (int i = 0; i < 7; i++) data[i] = 0xFF;
            data[7] = 0xFD;
        }

        /**
         * @brief Get Clear Errors command frame
         */
        void get_clear_errors_command(uint8_t data[8])
        {
            for (int i = 0; i < 7; i++) data[i] = 0xFF;
            data[7] = 0xFB;
        }

        /**
         * @brief Get Save Zero Pos command frame
         */
        void get_save_zero_command(uint8_t data[8])
        {
            for (int i = 0; i < 7; i++) data[i] = 0xFF;
            data[7] = 0xFE;
        }

    private:
        float uint_to_float(int x_int, float x_min, float x_max, int bits)
        {
            float span = x_max - x_min;
            return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
        }

        int float_to_uint(float x, float x_min, float x_max, int bits)
        {
            float span = x_max - x_min;
            if (x < x_min) x = x_min;
            if (x > x_max) x = x_max;
            return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
        }

    protected:
        Parameters params_;
    };

    /**
     * @brief J4310 Motor class
     */
    class J4310 : public DmMotor
    {
    public:
        J4310() : DmMotor(Parameters(-12.56f, 12.56f, -30.0f, 30.0f, -3.0f, 3.0f, 0.0f, 500.0f, 0.0f, 5.0f)) {}
    };
    
    /**
     * @brief J4340 Motor class
     */
    class J4340 : public DmMotor
    {
    public:
        J4340() : DmMotor(Parameters(-12.56f, 12.56f, -50.0f, 50.0f, -9.0f, 9.0f, 0.0f, 500.0f, 0.0f, 5.0f)) {}
    };

    /**
     * @brief J8009 Motor class
     */
    class J8009 : public DmMotor
    {
    public:
        J8009() : DmMotor(Parameters(-12.56f, 12.56f, -50.0f, 50.0f, -20.0f, 20.0f, 0.0f, 500.0f, 0.0f, 5.0f)) {}
    };
}

#endif
