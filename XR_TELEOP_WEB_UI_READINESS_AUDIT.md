# XR Teleop Web UI Readiness Audit

## Goal
Audit the current `xr_teleoperate` Web UI and decide:

- `READY`
- `READY WITH MINOR ISSUES`
- `NOT READY`

## Rules
- Do not modify source code.
- Do not perform real robot motion.
- Use hardware-free tests/mocks only.
- Do not narrate progress.
- Inspect first, then report once.

## Baseline Command
The UI must be able to produce and launch the equivalent of:

```bash
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode hand \
  --motion \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

## Audit Checklist

### 1. CLI compatibility
Verify:
- existing parameters are preserved
- no incorrect renaming
- no hidden hard-coded values
- defaults match the real parser
- strict choices only exist when the CLI defines them

### 2. Parameter UI
Verify:
- parameters are editable
- presets never lock fields
- booleans generate correct flags
- validation identifies the exact bad field
- generated command matches selected values
- Advanced / All Parameters exposes supported options

### 3. Start / Stop
Verify:
- Start launches existing `teleop_hand_and_arm.py`
- duplicate Start is blocked
- Stop uses graceful cleanup
- exit code/state is detected
- no orphan managed process remains

### 4. Interactive terminal
Verify the intended path is effectively:

```text
Browser terminal
    <-> WebSocket
    <-> Python backend
    <-> PTY
    <-> teleop_hand_and_arm.py
```

Check:
- live stdout/stderr
- raw terminal output is preserved
- ANSI colors work
- Unicode/emoji work
- spacing/line breaks are preserved
- resize works
- scrollback works

### 5. Runtime keyboard input
Using a harmless mock process, verify:
- `r` reaches PTY exactly once
- `q` reaches PTY exactly once
- input forwards only while process is running
- keys are captured only when terminal is focused
- typing in form fields cannot accidentally send runtime commands

Do not test `r/q` on real robot hardware.

### 6. UI / UX
Verify:
- Light theme works
- Black & White theme works
- Dark theme works
- theme persists
- terminal/log area is large enough
- terminal can grow/shrink/maximize
- current compact `xr_teleop` visual style is preserved

### 7. Safety
Verify:
- opening page never moves robot
- editing parameters never moves robot
- presets never move robot
- status checks are read-only
- only explicit Start launches teleoperation

### 8. Validation
Run relevant hardware-free tests plus:

```bash
python -m compileall <changed_python_paths>
git diff --check
```

Also verify the original CLI argument parser still works.

## Report Format

Return one report only:

```text
STATUS: READY / READY WITH MINOR ISSUES / NOT READY

1. What was checked
2. Passed checks
3. Failed checks
4. Blocking issues
5. Minor issues
6. Test results
7. Remaining hardware-only validation
8. Final recommendation
```

For every failure, include the relevant file/path and a short explanation.

Do not change code during this audit.
