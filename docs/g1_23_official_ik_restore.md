# G1_23 controller-pose IK: restore official best-effort behavior

## Why this change

The G1_23 IK path in this fork rejected a solved arm whenever the wrist position
residual exceeded `0.05` m, including after smoothing. The official Unitree
`G1_23_ArmIK` does not have this acceptance threshold; it uses a bounded,
best-effort IK solution. An otherwise valid Quest controller target could
therefore leave one or both arms holding the measured position in this fork.

Official reference, compared at Unitree commit
[`817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b`](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm_ik.py).

## Rotation-gradient regression and fix

Restoring official `cpin.log3(R_current @ R_target.T)` caused
`test_rotation_with_one_stationary_wrist_keeps_ik_solver_valid` to fail with
`nlp_grad_f failed: NaN detected for output grad_f_x` and IPOPT
`Invalid_Number_Detected` at iteration zero. The original suite reproduced
this as 9 passes and 1 failure with CasADi 3.6.7 and Pinocchio 3.1.0.

The failure is in the differentiated symbolic rotation expression: at the
zero measured joint configuration, a 0.8 rad left-wrist X rotation with the
right wrist stationary produces a non-finite rotation-cost gradient for the
left arm. Direct FK targets reproduce it before invoking IPOPT; the Quest
coordinate roundtrip also reproduces it (maximum pose difference about
`2.8e-17`). Thus neither an invalid Quest transform nor failed optimizer
convergence is needed to trigger the bug. This is a numerical differentiation
problem in this model/expression and dependency combination, not a claim that
the mathematical SO(3) logarithm is singular at 0.8 rad.

G1_23 now uses the previous chordal residual for each wrist:

```python
casadi.reshape(
    (R_current @ R_target.T - np.eye(3)) / np.sqrt(2.0),
    9, 1,
)
```

Its squared norm is `4 * sin(theta / 2)**2`, which approximates the official
`theta**2` cost for small angles while avoiding the symbolic logarithm.
The two wrists contribute 18 residual entries instead of 6. The objective
still uses the official rotation weight `0.5`; position, regularization and
smoothing weights remain `50`, `0.02` and `0.1`. The 5 cm positional rejection
remains removed, both before and after smoothing.

## Changes

1. `teleop/robot_control/robot_arm_ik.py`
   - Use the chordal wrist-rotation residual for `G1_23_ArmIK` to resolve the
     NaN-gradient regression introduced by restoring official `cpin.log3`.
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
   - Check finite rotation-cost gradients and Hessians through the real
     FK/controller transform path with either wrist stationary, at zero,
     0.8 rad and near-pi rotation errors. This added test failed on the
     restored `cpin.log3` expression before the fix.
3. `teleop/robot_control/test_g1_23_dry_run.py`
   - Update the unreachable-target preview expectation: a bounded best-effort
     result may be accepted even though the physical target remains unreachable.

Unchanged: `--arm G1_23` / `--arm G1_29` explicit model selection,
`--input-mode controller`, Quest pose transformation, locomotion handling,
motor gains, joint limits and DDS publishing. No G1-version-detection loop
was added. No automatic ready/home pose was added.

## Intentional differences from official Unitree G1_23 IK

| Area | This fork and reason |
| --- | --- |
| Rotation residual | Chordal matrix residual instead of `cpin.log3`, to keep derivatives finite for the reproduced controller-pose case. Large-angle costs differ; the gradient becomes weak near pi and is zero at exactly pi, so finite derivatives do not guarantee convergence there. |
| Asset/cache paths | Resolve from `__file__` instead of the process working directory, so offline tests and alternate launch directories load the same G1_23 model. |
| Input validation | Require finite, in-range measured joints and valid rigid wrist poses; validate optional joint velocities. A measured state is required to form a safe hold. |
| Partial tracking loss | Substitute measured FK for an invalid wrist target and hold that arm's measured joints with zero feedforward torque, allowing the tracked arm to continue. Freshness checks also remain in the existing caller/wrapper. |
| Solution validation | Check solver success, finite joint/torque outputs and existing model/controller limits. These checks prevent invalid outputs without rejecting a valid solution solely for positional residual. |
| Failure/filter state | Return measured joints and zero torque on failure without changing the saved seed/filter. Official code reads debug iterates and updates the filter/seed before returning measured joints on exceptions. |
| Diagnostics | Keep per-arm acceptance, failure reasons and final position residuals in `last_ik_status` to expose best-effort accuracy and partial holds. |

The 10-joint reduced model, locked joints, wrist-roll end frames and their
`[0.20, 0, 0]` offsets, model bounds, objective weights, IPOPT options,
`[0.4, 0.3, 0.2, 0.1]` filter weights and zero-velocity RNEA feedforward
calculation match the official implementation. `save_cache`, `load_cache`
and `scale_arms` are unchanged in executable structure; arm scaling remains
unused in G1_23 solving. The regression fix itself only changes G1_23's
rotation residual; G1_29 and G1_23's existing solve/acceptance path are unchanged.

## Software-only verification

```bash
python -m unittest discover -s teleop/televuer/tests -p 'test_hybrid_input.py' -v
python -m unittest teleop.robot_control.test_g1_23_controller_ik \
    teleop.robot_control.test_g1_23_dry_run -v
python -m unittest teleop.utils.test_xr_tracking_fallback \
    teleop.robot_control.test_arm_only_message_equivalence \
    teleop.robot_control.test_g1_23_readonly_state -v
python -m unittest teleop.robot_control.test_g1_29_reach_scaling \
    teleop.test_controller_locomotion -v
```

Results after the fix: **64 passed, 0 failed, 0 skipped**:

- G1_23 controller IK and dry run: 11 passed, including the original failing
  solver case and the new gradient/Hessian regression test. No CasADi NaN
  gradient warning or `Invalid_Number_Detected` occurred.
- Controller-pose/Quest 6DoF and hybrid input: 13 passed.
- Tracking fallback, stubbed command-message equivalence and read-only-state
  unit tests: 14 passed.
- G1_29 reach scaling (including offline MuJoCo validation) and controller
  locomotion: 26 passed.

Tests ran from an isolated checkout of the PR branch with its committed
submodules, excluding unrelated local work. Robot command publishers were
not initialized and no physical motors were commanded. Expected error logs
from deliberately mocked solver/RPC failures are part of the passing tests.

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
