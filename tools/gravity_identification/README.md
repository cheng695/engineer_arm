# Gravity Identification Data Collection

This folder is for the first route: Pinocchio + SciPy traditional parameter identification.

The first dataset should contain static robot postures and the real effort needed to hold each posture:

```text
q_real -> tau_real
```

Run the sampler after the real robot bringup is publishing joint states:

```bash
python3 tools/gravity_identification/gravity_static_sampler.py
```

Default input topic:

```text
/arm_debug/processed_joint_states
```

Default output:

```text
tools/gravity_identification/data/gravity_static_samples.csv
```

Controls:

```text
SPACE  collect one static posture sample
s      save CSV
q      save CSV and quit
```

Recommended procedure:

```text
1. Move the arm to a stable posture.
2. Wait until all selected joints are still.
3. Press SPACE and let the script average a 1 second window.
4. Repeat across the workspace.
5. Press q to save.
```

The CSV contains:

```text
stamp,sample_frame_count,
joint1..joint7,
joint1_velocity..joint7_velocity,
joint1_effort..joint7_effort,
joint1_effort_stddev..joint7_effort_stddev
```

If the chosen topic does not contain real effort feedback, switch topic:

```bash
python3 tools/gravity_identification/gravity_static_sampler.py --topic /arm_debug/raw_motor_states
```

For gravity identification on the real arm, prefer decoupled joint positions with real motor feedback efforts:

```bash
python3 tools/gravity_identification/gravity_static_sampler.py \
  --topic /arm_debug/processed_joint_states \
  --effort-topic /arm_debug/raw_motor_states \
  --csv-name gravity_static_decoupled_effort_samples.csv
```

In this mode, the saved `joint*_effort` columns mean:

```text
joint1, joint4, joint5, joint6, joint7: raw motor feedback effort
joint2, joint3: raw motor feedback effort mapped to decoupled joint-side effort
```

The J2/J3 effort mapping follows the hardware interface formula:

```text
joint2_effort = raw_joint2_effort - d(joint3_correction)/d(joint2) * raw_joint3_effort
joint3_effort = j2j3_j3_scale * raw_joint3_effort
```

Default parameters match the current C++ defaults:

```text
j2j3_coupling = 0.986
j2j3_j3_scale = 1.0
j2j3_scale_mode = divide
```

Override them if your bringup uses different values:

```bash
python3 tools/gravity_identification/gravity_static_sampler.py \
  --topic /arm_debug/processed_joint_states \
  --effort-topic /arm_debug/raw_motor_states \
  --j2j3-j3-scale 0.82
```

For early debugging you can relax effort validation:

```bash
python3 tools/gravity_identification/gravity_static_sampler.py --no-require-effort
```

## Pinocchio Baseline Comparison

After collecting a first CSV, compare measured static effort against URDF gravity torque:

```bash
python3 tools/gravity_identification/compare_pinocchio_gravity.py
```

Or pass a CSV path directly:

```bash
python3 tools/gravity_identification/compare_pinocchio_gravity.py tools/gravity_identification/data/gravity_static_raw_motor_samples.csv
```

Default inputs:

```text
CSV:  tools/gravity_identification/data/gravity_static_samples.csv
URDF: src/arm_description/urdf/v1_1_full_robot.urdf
```

Outputs:

```text
tools/gravity_identification/data/pinocchio_gravity_compare.json
tools/gravity_identification/data/pinocchio_gravity_predictions.csv
tools/gravity_identification/data/pinocchio_gravity_timeseries.png
tools/gravity_identification/data/pinocchio_gravity_scatter.png
tools/gravity_identification/data/pinocchio_gravity_rmse.png
```

The script reports two baselines:

```text
raw Pinocchio tau_g
per-joint scale+bias fit: tau_real[j] ~= scale[j] * tau_pinocchio[j] + bias[j]
```

If the measured effort sign is opposite to Pinocchio, rerun:

```bash
python3 tools/gravity_identification/compare_pinocchio_gravity.py --effort-sign -1
```

Useful metrics:

```text
RMSE / MAE / max_abs_error: lower is better
NRMSE by measured torque span: useful for comparing joints with different torque ranges
R squared: closer to 1 means the model explains more measured variation
```
