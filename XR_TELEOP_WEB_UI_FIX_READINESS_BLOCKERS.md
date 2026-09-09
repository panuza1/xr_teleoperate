# XR Teleop Web UI — Fix Readiness Blockers

## Goal

Fix the issues found by the latest Web UI readiness audit.

Current status:

```text
NOT READY
```

Do not perform real robot motion during this task.

Work through all fixes, run hardware-free validation, then rerun the readiness audit and report once.

## 1. Fix cross-origin `/api/start` safety issue

Current issue:

```text
POST /api/start accepts requests from an untrusted Origin and with text/plain,
allowing the frontend motion confirmation to be bypassed.
```

Fix all state-changing endpoints, especially:

```text
/api/start
/api/stop
terminal input / PTY write endpoints
any other POST/PUT/PATCH/DELETE endpoint that changes runtime state
```

Requirements:

- Reject cross-origin requests by default.
- Accept only expected local/same-origin requests.
- Validate `Origin` and/or `Host` appropriately.
- Require the expected content type, preferably `application/json`.
- Reject unexpected `text/plain`, form posts, and malformed requests unless explicitly required.
- Do not rely on frontend confirmation as the security boundary.
- Do not add permissive CORS such as `Access-Control-Allow-Origin: *`.
- Keep the UI usable from its intended local address.

Add regression tests proving that:

```text
Origin: http://evil.example
```

cannot invoke Start.

Also test invalid/missing content types and malformed payloads.

## 2. Fix runtime environment / `logging_mp`

The audit could not run:

```bash
python teleop/teleop_hand_and_arm.py --help
```

because:

```text
ModuleNotFoundError: logging_mp
```

`logging_mp` is already declared in:

```text
requirements.txt
```

Inspect the repository/environment setup before changing source code.

Preferred fix:

- restore/use the documented Python environment
- ensure required dependencies are installed correctly
- do not add hacks or duplicate fallback modules unless the repository genuinely requires a source fix

After fixing the environment, verify:

```bash
python teleop/teleop_hand_and_arm.py --help
```

works successfully without connecting to hardware or moving the robot.

If this is only an audit-environment issue, document that clearly instead of changing unrelated source.

## 3. Fix `extra_args` field validation mapping

Current issue:

```text
Invalid or oversized extra_args produces an error but does not map the error
back to the extra_args UI field.
```

Update validation so failures return structured field errors.

Example:

```json
{
  "field_errors": {
    "extra_args": "..."
  }
}
```

Requirements:

- frontend highlights `extra_args`
- exact error is visible near that field
- other valid fields remain editable
- oversized input is rejected safely
- malformed input is rejected safely
- do not use `shell=True`
- keep argument construction as a structured argv list

Add regression tests.

## 4. Preserve existing passing behavior

Do not regress:

- all 20 CLI parameters
- parser/default/choice matching
- editable parameters
- editable presets
- correct boolean flag generation
- exact baseline command generation
- duplicate Start protection
- graceful Stop / SIGINT cleanup
- process-group cleanup
- exit detection
- PTY/WebSocket terminal
- raw stdout/stderr
- ANSI output
- Unicode / emoji
- terminal resize
- 10,000-line scrollback
- runtime `r` / `q` forwarding
- terminal focus gating
- Light theme
- Black & White theme
- Dark theme
- theme persistence
- resizable/maximizable terminal
- passive page/status checks that do not touch robot hardware

Baseline command must remain equivalent to:

```bash
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode hand \
  --motion \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

## 5. Safety

During implementation and testing:

- do not start real robot motion
- do not send `r` to a real G1 session
- do not send `q` to a real G1 session
- use mock/harmless PTY processes
- opening the Web UI must remain read-only
- editing parameters must remain read-only
- presets must remain read-only
- only explicit Start may launch teleoperation

## 6. Validation

Run the full relevant hardware-free suite.

At minimum:

```bash
python -m unittest -v test_web_ui.py
python -m compileall -q web_ui.py test_web_ui.py teleop/teleop_hand_and_arm.py
git diff --check
python teleop/teleop_hand_and_arm.py --help
```

Also explicitly test:

```text
cross-origin /api/start rejection
wrong Content-Type rejection
malformed POST rejection
extra_args field error mapping
duplicate Start protection
graceful Stop
PTY r/q forwarding with mock process
process-group cleanup
```

## 7. Rerun Readiness Audit

After fixes pass, rerun:

```text
XR_TELEOP_WEB_UI_READINESS_AUDIT.md
```

Do not weaken the audit criteria.

Expected result before hardware testing:

```text
READY
```

or, at minimum, no remaining software blocker.

## Final Report

Do not narrate progress.

When complete, report only:

```text
STATUS: READY / READY WITH MINOR ISSUES / NOT READY

1. What changed
2. Files changed
3. Security fixes
4. Environment/dependency result
5. Tests and results
6. Readiness audit result
7. Remaining hardware-only validation
```

If any blocker remains, explain exactly why and identify the relevant file/path.
