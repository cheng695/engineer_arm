"""Step 1: describe the Reach 6D Pose RL task.

This is intentionally plain Python for learning. Later we will convert it to an
Isaac Lab environment config.
"""

from dataclasses import dataclass


@dataclass
class Reach6DPoseEnvConfig:
    # Simulation scale
    num_envs: int = 1024
    episode_length_s: float = 6.0
    control_dt: float = 1.0 / 60.0

    # RL action: end-effector Twist [vx, vy, vz, wx, wy, wz]
    action_dim: int = 6
    max_linear_twist: float = 0.25
    max_angular_twist: float = 1.0

    # Success condition
    position_success_tolerance: float = 0.025
    orientation_success_tolerance_rad: float = 0.15

    # Random target range: x, y, z in robot base frame
    target_position_range_m: tuple = ((0.25, 0.55), (-0.25, 0.25), (0.15, 0.55))


OBSERVATION_TERMS = [
    "arm_joint_positions",
    "arm_joint_velocities",
    "end_effector_position",
    "end_effector_orientation",
    "target_position",
    "target_orientation",
    "pose_error",
]


ACTION_TERMS = [
    "end_effector_twist_6d",
]


REWARD_TERMS = [
    "small_position_error",
    "small_orientation_error",
    "smooth_action",
    "avoid_joint_limits",
]


TERMINATION_TERMS = [
    "success",
    "timeout",
    "unsafe_state",
]
