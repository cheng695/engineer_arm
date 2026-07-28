# Reach 6D Pose

Goal: first describe the RL task clearly.

Final idea:

```text
robot state + target 6D pose
  -> RL policy
  -> end-effector Twist
  -> DLS-SVD
  -> arm motion
```

Step 1 only keeps one config file:

```text
config/reach_6d_pose_env_cfg.py
```

For now, this file only answers:

- What can RL observe?
- What does RL output?
- What is the reward?
- When does one episode end?
