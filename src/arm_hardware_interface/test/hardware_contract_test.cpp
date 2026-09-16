#include "arm_hardware_interface/arm_hardware_base.hpp"
#include <cmath>
#include <stdexcept>

class Probe : public arm_hardware_interface::ArmHardwareBase {
public:
    void check() {
        hardware_interface::HardwareInfo info;
        info.joints.resize(3);
        init_joint_buffers(info);
        hw_commands_pos_ = {0.2, 0.3, 0.4};
        use_real_joint_io_ = {false, true, true};
        echo_mock_joints(info);
        require(hw_states_pos_[0], 0.2);
        for (bool multiply : {false, true}) {
            j2j3_scale_mode_ = multiply ? "multiply" : "divide";
            j2j3_j3_scale_ = 1.7;
            j2j3_poly_a2_ = 0.2;
            hw_states_pos_ = {0, 0.3, 0.4};
            hw_states_vel_ = {0, 0.5, 0.6};
            hw_states_eff_ = {0, 0.7, 0.8};
            const double input_power = 0.5 * 0.7 + 0.6 * 0.8;
            apply_j2j3_coupling();
            require(hw_states_vel_[1] * hw_states_eff_[1] +
                    hw_states_vel_[2] * hw_states_eff_[2], input_power);
            const double correction = j2j3_poly_correction(0.3);
            const double recovered = multiply
                ? (hw_states_pos_[2] - correction) / 1.7
                : 1.7 * hw_states_pos_[2] - correction;
            require(recovered, 0.4);
        }
        // 重力力矩现在由 controller 写入 effort command，硬件层不再计算重力。
    }
private:
    static void require(double actual, double expected) {
        if (std::abs(actual - expected) > 1e-10)
            throw std::runtime_error("hardware contract mismatch");
    }
};
int main() { Probe().check(); }
