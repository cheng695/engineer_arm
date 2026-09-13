#pragma once

#include "my_robot_control_manager/control_types.hpp"

namespace my_robot_control_manager
{

struct FsmTransition
{
    bool accepted{false};   // 状态是否被接受
    bool changed{false};    // 状态是否被改变
    ArmControlState previous{ArmControlState::DISABLED};  // 原状态
    ArmControlState current{ArmControlState::DISABLED};   // 新状态
};

// 纯状态机：只处理事件和状态
class ControlFsm
{
public:
    void init();
    FsmTransition dispatch(ArmControlEvent event);
    FsmTransition forceStop();
    ArmControlState state() const { return state_; }
    static const char* stateName(ArmControlState state);

private:
    FsmTransition transitionTo(ArmControlState next);
    bool canAcceptCommand() const;

    ArmControlState state_{ArmControlState::DISABLED};
};

}  // namespace my_robot_control_manager
