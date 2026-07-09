#pragma once

#include <string>

namespace arm_hardware_interface
{

/**
 * @brief 控制状态机
 *
 *  STOP  ── enable ──→ IDLE
 *  IDLE  ── disable ─→ STOP
 *  IDLE  ── twist   ─→ DLS
 *  IDLE  ── joint   ─→ JOINT
 *  IDLE  ── pose    ─→ POSE
 *  DLS   ── timeout ─→ IDLE
 *  JOINT ── timeout ─→ IDLE
 *  POSE  ── done    ─→ IDLE
 *  任意  ── disable ─→ STOP
 */
class ControlFsm
{
public:
  enum class State
  {
    STOP,   // 停机（电机失能，不输出任何指令）
    IDLE,   // 待命（电机使能，等待指令，保持位置）
    DLS,    // 笛卡尔控制（DlsController）
    JOINT,  // 关节控制（JointController）
    POSE,   // 固定位姿（arm_controller 轨迹执行）
  };

  ControlFsm() = default;

  /// 当前状态
  State state() const { return state_; }

  /// 状态名称（日志用）
  const char* state_name() const;

  // ---- 事件（由外部调用） ----

  /// 电机使能
  void onEnable()  { requested_ = Request::ENABLE; }
  /// 电机失能
  void onDisable() { requested_ = Request::DISABLE; }
  /// 收到 DLS twist
  void onTwist()   { twist_pending_ = true; }
  /// 收到关节速度
  void onJoint()   { joint_pending_ = true; }
  /// 固定位姿开始
  void onPoseStart() { requested_ = Request::POSE_START; }
  /// 固定位姿结束
  void onPoseDone()  { requested_ = Request::POSE_DONE; }

  /**
   * @brief 每周期调用，处理待处理的事件。
   * @param twist_active   DLS countdown > 0
   * @param joint_active   joint countdown > 0
   * @return 当前状态是否允许输出控制指令
   */
  bool update(bool twist_active, bool joint_active);

  /// 是否为 STOP 状态
  bool isStopped() const { return state_ == State::STOP; }

  /// 是否刚切换到 DLS（本周期首次激活）
  bool justEnteredDls() const { return just_entered_dls_; }
  /// 是否刚切换到 JOINT
  bool justEnteredJoint() const { return just_entered_joint_; }
  /// 是否刚从 STOP 切出（需要同步位置）
  bool justExitedStop() const { return just_exited_stop_; }

private:
  enum class Request
  {
    NONE,
    ENABLE,
    DISABLE,
    POSE_START,
    POSE_DONE,
  };

  void setState(State s);

  State state_ = State::STOP;
  Request requested_ = Request::NONE;
  bool twist_pending_ = false;
  bool joint_pending_ = false;

  bool just_entered_dls_ = false;
  bool just_entered_joint_ = false;
  bool just_exited_stop_ = false;
};

}  // namespace arm_hardware_interface
