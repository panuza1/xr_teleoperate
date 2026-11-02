# G1_23 controller-pose IK: restore official best-effort behavior

## Why this change

The G1_23 IK path in this fork rejected a solved arm whenever the wrist position
residual exceeded `0.05` m, including after smoothing. The official Unitree
`G1_23_ArmIK` does not have this acceptance threshold; it uses a bounded,
best-effort IK solution. An otherwise valid Quest controller target could
therefore leave one or both arms holding the measured position in this fork.

Official reference:
https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_arm_ik.py

## Changes

1. `teleop/robot_control/robot_arm_ik.py`
   - Restore official `pinocchio.casadi.log3` wrist-rotation error for `G1_23_ArmIK`.
   - Remove the `position_error_tolerance=0.05` constructor argument and both
     position-residual rejection paths (raw and smoothed).
   - Keep wrist residuals in `last_ik_status` for diagnosis. Retain finite-pose
     checks, arm joint limits, solver-failure fallback, measured-position holds
     on tracking loss, and the smoothing filter.
2. `teleop/robot_control/test_g1_23_controller_ik.py`
   - Test that a bounded solver result with >5 cm positional residual is
     reported rather than rejected solely for that residual.
   - Continue checking independent controllers, joint limits, invalid poses,
     single-controller loss and recovery.
3. `teleop/robot_control/test_g1_23_dry_run.py`
   - Update the unreachable-target preview expectation: a bounded best-effort
     result may be accepted even though the physical target remains unreachable.

Unchanged: `--arm G1_23` / `--arm G1_29` explicit model selection,
`--input-mode controller`, Quest pose transformation, locomotion handling,
motor gains, joint limits and DDS publishing. No G1-version-detection loop
was added. No automatic ready/home pose was added.

## Software-only verification

```bash
python -m unittest discover -s teleop/televuer/tests -p 'test_hybrid_input.py' -v
python -m unittest teleop.robot_control.test_g1_23_controller_ik \
    teleop.robot_control.test_g1_23_dry_run -v
```

The controller-pose tests can run independently of robot hardware. The IK tests
need the project's documented **conda** environment with a Pinocchio build
that provides `pinocchio.casadi`, the Unitree SDK and the project's submodules;
the generic pip Pinocchio build does not supply that binding.

## Important limits

An accepted IK result does **not** guarantee the requested wrist position was
reached. Inspect `last_ik_status['left_residual_m']` and
`['right_residual_m']` while diagnosing motion. Out-of-reach targets may drive
the solver toward joint limits, although the existing joint limit and velocity
checks remain. Validate in a controlled, supervised hardware test only after
confirming the robot is G1_23, the correct DDS interface, and stable tracking.
This change does not prove the physical tracking issue is fixed.
