# MoveIt Config Variant: V1.1

This variant combines the MoveIt Setup Assistant output for the V1.1 robot with
the project's existing controller and launch assumptions.

- `my_robot.srdf` comes from the regenerated V1.1 Setup Assistant package.
- `joint_limits.yaml` mirrors the V1.1 URDF limits and keeps full velocity scaling.
- `kinematics.yaml` keeps IK only for the `arm` planning group.
- Controller, Servo, RViz, and Pilz config files are inherited from the root
  `my_robot_moveit_config/config` files to preserve project-specific behavior.
- `sensors_3d.yaml` is intentionally disabled because no 3D perception sensor
  was configured in Setup Assistant.

