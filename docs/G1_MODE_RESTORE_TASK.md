# G1 Teleop Mode Restore Task

## Problem

When running `teleop/teleop_hand_and_arm.py` on a real Unitree G1 without `--motion`, the program enters debug / low-level control mode.

After exiting teleoperation with `Ctrl+C` or `q`, the robot may remain in this low-level/debug state. As a result, the remote controller or app may no longer switch the G1 back to its normal operational modes until the robot is restarted.

## Confirmed Root Cause (Phase 1)

Relevant files:

- `teleop/teleop_hand_and_arm.py`
- `teleop/robot_control/robot_arm.py`
- `teleop/utils/motion_switcher.py`

Confirmed behavior:

1. `teleop_hand_and_arm.py` calls `MotionSwitcher.Enter_Debug_Mode()`.
2. `Enter_Debug_Mode()` repeatedly calls `ReleaseMode()` until no active high-level mode remains.
3. The arm controller starts publishing low-level commands.
4. On shutdown, `ctrl_dual_arm_go_home()` is called.
5. The code that should call `Exit_Debug_Mode()` is currently disabled/commented.
6. Arm publishing threads use continuous loops and do not currently have a proper explicit shutdown mechanism.

Because of this, simply enabling `Exit_Debug_Mode()` is not sufficient. Mode restoration must happen only after low-level command publishing has stopped.

### Affected Control Flow

- Real robot without `--motion`: DDS domain 0 is initialized, `Enter_Debug_Mode()` releases the active high-level mode, then the arm controller starts publishing `rt/lowcmd` at 250 Hz.
- Real robot with `--motion`: debug mode is not entered; G1 arm commands use `rt/arm_sdk`, and controller locomotion may create `LocoClientWrapper`.
- `--sim`: DDS domain 1 is used, physical mode switching is skipped, and the arm publisher still runs against the simulated command topic.
- `q` sets `STOP`; both the pre-start wait and teleop loop terminate and execution reaches `finally`.
- `Ctrl+C` is caught and reaches the same `finally` block. Other startup/runtime exceptions are logged and also reach it.
- Cleanup currently homes the arms while the publisher is active, then closes keyboard/IPC, image, XR, simulation, and recording resources. It never stops the arm publisher or subscriber.
- Every arm controller in `robot_arm.py` has an infinite daemon publisher loop. Real-robot controllers also have an infinite daemon low-state subscriber loop. Neither loop has a stop event or bounded join.
- End-effector controllers also start daemon threads/processes, but they publish separate hand topics; the G1 high-level-mode race is caused by the arm publisher on `rt/lowcmd`.
- Partial startup is tolerated only by separate broad cleanup `try` blocks: uninitialized local variables raise `NameError`, are logged, and cleanup continues.

`MotionSwitcher.Exit_Debug_Mode()` currently only calls `SelectMode('ai')`; it does not call `CheckMode()` afterward, so it cannot verify restoration. Both switcher methods also collapse SDK exceptions to `(None, None)`.

## Target Shutdown Flow

```text
Ctrl+C / q
    ↓
Stop accepting teleop commands
    ↓
Move arms to safe/home target
    ↓
Stop low-level arm publisher thread
    ↓
Stop related controller resources
    ↓
Restore G1 high-level mode
    ↓
Verify restored mode
    ↓
Close remaining resources
    ↓
Exit
```

For partial startup, each step applies only if that resource was successfully created. Arm home must run before arm `stop()`, and verified mode restoration must run only after `stop()` confirms the low-level publisher is no longer alive.

## Files to Change

- Phase 2: `teleop/robot_control/robot_arm.py`, `teleop/teleop_hand_and_arm.py`, and one hardware-free lifecycle test.
- Phase 3: `teleop/utils/motion_switcher.py`, `teleop/teleop_hand_and_arm.py`, and mocked mode-restore tests.
- Phase 4: only files required by issues found during the final review.

## Implementation Checklist

- [x] Confirm debug-mode entry, DDS publisher startup, exit paths, resources, and mode differences.
- [x] Add idempotent, bounded arm controller shutdown and prove publisher termination.
- [x] Route `q`, `Ctrl+C`, startup failure, and runtime failure through initialized-resource-safe cleanup.
- [x] Restore only real-robot, non-`--motion` sessions that successfully entered debug mode.
- [x] Verify the selected high-level mode before logging success.
- [x] Run the full hardware-free shutdown/restore regression gate.

## Safety Rules

- Do not send real robot commands during automated tests.
- Use mocks for MotionSwitcher and DDS tests.
- Do not restore robot modes in `--sim`.
- Keep `--motion` behavior separate from debug-mode behavior.
- Never report mode restoration as successful unless it is verified.
- Shutdown must remain safe if startup fails halfway through.
- Avoid race conditions between low-level publishers and mode switching.

---

# Implementation Plan

## Phase 1 — Analyze and Document

Inspect:

- `teleop/teleop_hand_and_arm.py`
- `teleop/robot_control/robot_arm.py`
- `teleop/utils/motion_switcher.py`

Confirm:

- where debug mode is entered
- where low-level DDS publishing begins
- how Ctrl+C and `q` reach cleanup
- which controllers start background threads/processes
- current cleanup order
- behavior differences between:
  - real robot debug mode
  - `--motion`
  - `--sim`

Do not modify functional code during this phase.

### Codex Prompt

```text
Investigate the G1 shutdown/mode-restore bug in this repo.

Problem: after running teleop without --motion and exiting with Ctrl+C/q, the G1 may remain in debug/low-level mode and the remote cannot return to normal modes.

Inspect:
- teleop/teleop_hand_and_arm.py
- teleop/robot_control/robot_arm.py
- teleop/utils/motion_switcher.py

Do not modify functional code yet.

Update docs/G1_MODE_RESTORE_TASK.md with:
- confirmed root cause
- affected control flow
- safe shutdown order
- files to change
- test plan
- implementation checklist

Keep it concise.
```

---

## Phase 2 — Fix Controller Shutdown

Status: complete (2026-08-30).

Add a proper lifecycle to the arm controller.

Expected changes:

- add explicit running/shutdown state
- make low-level publisher loops stoppable
- provide `stop()` / `close()` or equivalent
- ensure shutdown waits for publishing to stop
- make shutdown idempotent
- avoid hanging joins
- handle partially initialized controllers safely

Ctrl+C and `q` should eventually use the same cleanup path.

Do not automatically restore high-level robot mode yet.

### Codex Prompt

```text
Read docs/G1_MODE_RESTORE_TASK.md.

Implement Phase 2: safe controller shutdown.

Requirements:
- arm publishing threads must be stoppable
- no infinite lowcmd publishing after cleanup begins
- Ctrl+C and q must share the same cleanup path
- shutdown must be idempotent
- preserve existing teleop behavior
- do not implement automatic robot mode restoration yet

Add/update hardware-free tests.

Run relevant tests and update docs/G1_MODE_RESTORE_TASK.md.
```

---

## Phase 3 — Restore G1 Mode

Status: complete (2026-08-30).

After low-level publishing is fully stopped, restore the robot high-level mode.

Expected logic:

```text
if real_robot and not args.motion and debug_mode_was_entered:
    stop low-level publishers
    restore high-level mode
    verify mode
```

Use the existing MotionSwitcher abstraction.

Potential target mode:

```text
ai
```

But implementation should verify actual mode switching behavior instead of assuming success.

### Codex Prompt

```text
Read docs/G1_MODE_RESTORE_TASK.md.

Implement Phase 3: safe G1 mode restoration.

Requirements:
- restore only after low-level command publishing has stopped
- apply only to real robot non---motion debug-mode operation
- do not restore modes in --sim
- use MotionSwitcher
- verify the resulting mode
- handle failure safely with clear logs
- never claim success unless verification passes
- keep --motion behavior unchanged

Use mocks only in tests. Do not send commands to real hardware.

Update docs/G1_MODE_RESTORE_TASK.md when finished.
```

---

## Phase 4 — Final Review

Status: complete (2026-08-30).

Review the complete implementation.

Check:

- Ctrl+C
- `q`
- exception during startup
- exception during teleop
- controller created partially
- image client unavailable
- XR wrapper unavailable
- end-effector initialization failure
- repeated cleanup calls
- real robot debug mode
- `--motion`
- `--sim`
- low-level publisher shutdown
- mode restoration verification
- logging correctness

### Codex Prompt

```text
Read docs/G1_MODE_RESTORE_TASK.md.

Perform the final review.

Check:
- shutdown race conditions
- Ctrl+C/q behavior
- debug vs --motion behavior
- simulation safety
- partial-startup exceptions
- background thread/process cleanup
- idempotent cleanup
- mode restoration verification
- regression risks

Fix issues found.

Run all relevant hardware-free tests.

Finally update docs/G1_MODE_RESTORE_TASK.md with:
- completed checklist
- tests and results
- remaining real-hardware validation steps

Do not perform real-robot motion tests.
```

---

# Test Plan

Hardware-free tests should cover:

- [x] publisher thread can start and stop
- [x] `stop()` can be called more than once
- [x] shutdown does not leave a publishing thread alive
- [x] Ctrl+C enters cleanup
- [x] `q` enters the same cleanup
- [x] `--sim` does not invoke MotionSwitcher restore
- [x] `--motion` does not use debug-mode restore flow
- [x] non-motion real robot path attempts restore
- [x] restore occurs after publisher shutdown
- [x] failed restore is logged as failure
- [x] successful restore is verified
- [x] partial initialization does not crash cleanup
- [x] existing arm-only tests still pass
- [x] Inspire/Dex hand behavior is not regressed

## Real Hardware Validation

Do only after hardware-free tests pass.

Suggested manual validation:

1. Power on G1 normally.
2. Confirm remote/controller mode switching works.
3. Start teleop without `--motion`.
4. Do not begin aggressive motion.
5. Exit using `q`.
6. Confirm cleanup logs.
7. Confirm remote/controller mode switching works again.
8. Repeat using `Ctrl+C`.
9. Test failure handling separately.

Keep an emergency stop / damping procedure available during hardware tests.

---

# Status

- Root cause: confirmed in local code
- Phase 1: complete — control flow and shutdown race verified in local code (2026-08-30)
- Phase 2: complete — arm publisher/subscriber lifecycle and shared cleanup implemented (2026-08-30)
- Phase 3: complete — real G1 debug sessions restore and verify `ai` after arm shutdown (2026-08-30)
- Phase 4: complete — final shutdown, partial-startup, mode-gating, and regression review passed (2026-08-30)
- Real hardware validation: pending

## Phase 1 Test Results

- Python 3.10 compile check for the three inspected modules: passed.
- Hardware-free arm message equivalence and XR fallback tests: 9 passed.
- Hardware-free controller-locomotion tests with SDK boundary stubs: 7 passed.
- Hardware-free Inspire worker-selection tests with SDK/retargeting boundary stubs: 2 passed.
- No robot, DDS network, camera, or motion command was used.

## Phase 2 Test Results

- Python 3.10 compile check and `git diff --check`: passed.
- Arm lifecycle, message equivalence, shared cleanup, and XR fallback tests: 15 passed.
- Controller-locomotion tests with SDK boundary stubs: 7 passed.
- Inspire worker-selection tests with SDK/retargeting boundary stubs: 2 passed.
- No robot, DDS network, camera, or motion command was used.

## Phase 3 Test Results

- Python 3.10 compile check and `git diff --check`: passed.
- Mode selection/verification, restore gating, shutdown ordering, arm lifecycle, message equivalence, and XR fallback tests: 22 passed.
- Controller-locomotion tests with SDK boundary stubs: 7 passed.
- Inspire worker-selection tests with SDK/retargeting boundary stubs: 2 passed.
- No robot, DDS network, camera, or motion command was used.

## Phase 4 Review and Test Results

- Debug-mode entry now has bounded release attempts and aborts before low-level arm startup if entry is not verified.
- All cleanup resources default safely to `None`; `q`, `Ctrl+C`, startup failures, and runtime failures share the same guarded `finally` path.
- Shutdown order is home arm, stop arm publisher/subscriber, stop Inspire DFX worker, restore/verify G1 mode, then close remaining resources.
- Python 3.10 compile check and `git diff --check`: passed.
- Mode switching, shutdown ordering/lifecycle, partial startup, arm equivalence, and XR fallback tests: 26 passed.
- Controller-locomotion tests with SDK boundary stubs: 7 passed.
- Inspire lifecycle/topic/order and Dex mapping tests with hardware boundaries stubbed: 5 passed.
- No robot, DDS network, camera, or motion command was used.

Remaining hardware validation is the manual procedure above: repeat real non-motion G1 shutdown using both `q` and `Ctrl+C`, confirm logs verify `ai`, and confirm remote/app mode switching works afterward. Keep emergency damping/stop available.

## Important

Do not treat uncommenting `Exit_Debug_Mode()` alone as the complete fix.

The low-level DDS publisher lifecycle must be fixed first so that the program does not continue publishing low-level commands while restoring the high-level robot mode.
