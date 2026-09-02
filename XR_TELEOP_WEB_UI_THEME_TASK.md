# XR Teleop Web UI implementation contract

`xr_teleop_control_ui_theme.html` is the visual reference. Preserve its compact
three-column developer-tool layout, thin borders, typography, status/command
placement, and Light, Black & White, and Dark theme system. Do not turn the UI
into a generic dashboard.

## Launch behavior

The default preset and fresh browser state must generate exactly:

```text
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode hand \
  --motion \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

Start launches the existing `teleop/teleop_hand_and_arm.py` process without
`shell=True`. Stop must let that process perform its existing shutdown and
cleanup. Editing configuration or applying a preset only changes the next
launch; it must not affect a running process. No robot motion testing is part
of this task.

Body Tracking is not a main control. If `--body-tracking` exists in the real
CLI, retain it only as an Advanced/All parameter. Motion is a main boolean
control and emits `--motion` only when enabled. Keep every other real CLI
parameter supported.

## Parameter editor

Treat the actual `argparse` declarations in `teleop_hand_and_arm.py` as the
source of truth for flag, destination, type, default, choices, required state,
boolean action, and help text. The backend launch builder and frontend must use
the same derived schema.

- Basic shows frequent parameters, Advanced shows less common parameters, and
  All Parameters shows every parser argument.
- Use toggles for `store_true` flags. Never emit fake boolean values.
- Use editable text/number controls. Use editable suggestions for enumerated
  values, enforcing a list only when `argparse` defines `choices`.
- Show friendly labels, real CLI flags, parser help, field-specific errors,
  modified state, per-field reset, and reset-all-to-CLI-defaults.
- Keep Extra CLI Arguments for uncommon future/debug options; it does not
  replace controls for known flags.
- Presets populate values and inclusion state but never lock controls.
- The generated command updates from the edited values and is the exact
  argument list Start uses.

## Interactive terminal

Keep generated-command preview separate from a substantially larger xterm.js
terminal. The authoritative session path is:

```text
browser xterm.js <-> WebSocket <-> Python backend <-> Linux PTY <-> teleop process
```

Forward raw PTY bytes without parsing, timestamps, cards, summaries, or other
reformatting. Preserve stdout/stderr ordering, line breaks, spacing, ANSI,
Unicode, emoji, selection, and scrollback. Forward terminal input to the same
PTY only while the process is running and the terminal is focused, so the
existing program handles `r`, `q`, and all other keys. Support drag and button
height changes, maximize/restore, resize propagation, copy, clear, auto-scroll,
exit code, and stopped/disconnected states.

## Hardware-free acceptance gate

Validate parser/schema coverage, editable strings, strict choices, boolean
generation, CLI defaults/reset, editable presets, exact baseline args, shared
preview/start argument building, field errors, duplicate process protection,
PTY output and input (`r`/`q`), stopped-input rejection, WebSocket raw-byte
streaming, resize, and graceful shutdown. Do not launch the real teleop entry
point during tests.
