# IMPLEMENTATION_PLAN.md

## Quest 3 → Inspire Hand software readiness

**Status:** Implementation plan only. No implementation changes are authorized by this document alone.

**Prepared:** 2026-09-22, based on the repository inspection and focused checks described below.

**Primary target:** Quest 3 hand tracking → TeleVuer → existing DexPilot retargeting → two Inspire DFX hands.

**Meaning of “12-DoF”:** Six independently commanded motors per hand, twelve across both hands. The additional URDF finger joints are coupled through mimic relationships; they are not additional command channels.

**Completion target:** `SOFTWARE_READY` means the complete hand pipeline, its safety behavior, production transport adapters, recording, replay, and diagnostics pass the required hardware-free tests. It does **not** mean physical motion, collision clearance, latency, or calibration has been validated.

All implementation phases below are **SOFTWARE_ONLY**. Physical testing is isolated into later **HARDWARE_VALIDATION_REQUIRED** phases.

Paths in this document are relative to the `xr_teleoperate` repository root unless explicitly prefixed with `../`. References to `dex_retargeting/` source files are under `teleop/robot_control/dex-retargeting/src/`.

---

## 1. Audit baseline and evidence

### Repository state inspected

- Main repository HEAD: `681de6f0d6817cddc09e6b755040661638002f3f`.
- Dex-retargeting submodule: `d7753d38c9ff11f80bafea6cd168351fd3db9b0e`.
- Current TeleVuer checkout: `3590cba5b3a76cebc43dc1ae250c01a8e6d274dd`.
- TeleVuer’s checkout differs from the parent repository’s recorded submodule revision.
- Existing local modifications:
  - `teleop/utils/episode_writer.py`
  - TeleVuer submodule revision
  - Untracked `test/`, including timestamp coverage

Preserve these changes. Do not reset the repository or submodules to obtain a clean baseline.

Read the parent workspace documents before implementation:

- `../AGENTS.md`
- `../MASTER_PLAN.md`
- `../INTEGRATION_SPEC.md`
- `../HANDOFF.md`

The parent plan covers a broader G1/LeRobot project. This document adds a hand-pipeline readiness gate; it does not restart or replace that broader project.

### Checks performed during this analysis

| Check | Result |
|---|---|
| Focused unittest suite in the existing `tv` environment | **39 passed** |
| Tracking fallback, mode restoration, arm-message equivalence, and episode timestamps in `teleopit` | **20 passed, 6 subtests passed** |
| Construct both Inspire retargeters from `teleop/` | **Passed** |
| Construct Inspire retargeter from repository root | **Failed:** relative `../assets` path |
| Existing mapping indices | Both hands: `[4, 6, 2, 0, 9, 8]` for the current model |
| Existing retargeter filter | `alpha = 0.2` |
| Installed Pinocchio collision support in `tv` | Available |
| Broader initial pytest collection in `teleopit` | Blocked by missing `unitree_sdk2py`; this is an environment issue |

These checks establish a baseline. They do not establish hardware-free end-to-end hand correctness.

No hardware command tests were run.

---

## 2. Current architecture

### Existing data flow

```text
Quest browser / Vuer HAND_MOVE
    │
    │ 25 joint transforms per hand, flattened column-major
    ▼
TeleVuer.on_hand_move()
    │
    ├── wrist transforms
    ├── 25 × 3 world-space landmark positions
    ├── joint orientations
    └── pinch/squeeze values
    ▼
TeleVuerWrapper.get_tele_data()
    │
    ├── OpenXR → robot basis
    ├── world → wrist-local coordinates
    └── existing hand-frame convention transform
    ▼
TeleData.left_hand_pos / right_hand_pos
    │
    │ each 25 × 3
    ▼
teleop_hand_and_arm.py shared arrays
    ▼
Inspire_Controller_DFX or Inspire_Controller_FTP
    │
    ├── landmark difference vectors
    ├── DexPilot optimization
    ├── existing URDF limits and mimic joints
    ├── existing low-pass filter
    ├── name-based six-motor selection
    └── radians → normalized openness
    ▼
DFX: MotorCmds_, right hand first
or
FTP: separate left/right messages, integer angle values
    ▼
Hardware bridge / Inspire hands
```

### What already exists

| Area | Existing implementation |
|---|---|
| Quest input | `teleop/televuer/src/televuer/televuer.py` |
| Coordinate conversion | `teleop/televuer/src/televuer/tv_wrapper.py` |
| Retargeting wrapper | `teleop/robot_control/hand_retargeting.py` |
| Retargeting algorithm | Vendored `dex-retargeting` submodule |
| Inspire models | Left/right URDFs and meshes in `assets/inspire_hand/` |
| Retargeting configuration | `assets/inspire_hand/inspire_hand.yml` |
| DFX transport | `Inspire_Controller_DFX` |
| FTP transport | `Inspire_Controller_FTP` |
| Recording | `EpisodeWriter`, including local timestamp additions |
| Visualization | Existing Rerun episode visualization |
| Quest-only inspection | `teleop/inspect_hybrid_input.py`, without DDS initialization |
| G1 tracking fallback helpers | `teleop/utils/xr_tracking_fallback.py` |
| DFX bridge | Sibling repository `../dfx_inspire_service` |
| Regression coverage | Worker lifecycle, hybrid input, arm shutdown/mode restoration, timestamps, UI |

### Existing command contract

Per-hand logical motor order:

```text
pinky, ring, middle, index, thumb_bend, thumb_rotation
```

Canonical application order:

```text
[left hand × 6, right hand × 6]
```

DFX wire order:

```text
[right hand × 6, left hand × 6]
```

Existing normalization:

| Motor | URDF range, radians | Normalized command |
|---|---:|---|
| Pinky | `[0, 1.7]` | `(1.7 - q) / 1.7` |
| Ring | `[0, 1.7]` | Same |
| Middle | `[0, 1.7]` | Same |
| Index | `[0, 1.7]` | Same |
| Thumb bend | `[0, 0.5]` | `(0.5 - q) / 0.5` |
| Thumb rotation | `[-0.1, 1.3]` | `(1.3 - q) / 1.4` |

The current Python implementation and inspected DFX bridge agree on this ordering and normalized command convention.

---

## 3. Findings: incomplete, broken, conflicting, and risky

### Confirmed software defects and gaps

| Finding | Evidence and consequence |
|---|---|
| Hand readiness never expires | `motion_data_ready` becomes true after an event and has no hand-frame timeout. A disconnected headset can leave stale commands active. |
| Worker starts before operator start | Inspire constructors start their workers immediately. Initial targets are all ones, and publishing is not gated by the application’s `START` state. |
| Malformed input can partially update shared state | `on_hand_move()` writes wrist and landmark buffers before completing validation. A later exception leaves mixed data. |
| Input exceptions disappear | `on_hand_move()` uses a bare `except: pass`. Invalid frames are neither diagnosed nor counted. |
| No coherent hand snapshot | Wrist, landmarks, orientation, and readiness are read/written under separate locks. A consumer can combine different events. |
| Tracking validity is coupled across hands | The wrapper outputs zero landmarks for both hands when either wrist fails its current validity check. |
| Matrix validation is incomplete | A finite, nonzero determinant does not establish a rigid transform. Transpose-based inversion assumes an orthonormal rotation. |
| Asset loading depends on working directory | Inspire retargeting construction fails from repository root. |
| Safety logic is duplicated | DFX and FTP independently implement the same retargeting and normalization loops. |
| No final finite-value command guard | Clipping does not turn NaN into a valid command. DFX publication has no final finite-value rejection. |
| No modeled collision guard | DexPilot fingertip projection is not a thumb–index collision checker. |
| No configurable output calibration | Scale exists in retargeting YAML; motor gain, offset, deadzone, soft limits, and rate limits are not consolidated. |
| No state-age supervision | DFX waits for initial state but does not subsequently enforce fresh, valid feedback. |
| Malformed DFX state is unsafe to parse | `_on_hand_state()` indexes twelve state entries without validating message length or values. |
| Real DFX process owns inherited DDS objects | The publisher is created before `Process` starts. Simulation already avoids this by using a thread. The real path retains the same ownership risk. |
| FTP lifecycle is incomplete | Worker handle is local, subscription loop runs indefinitely, and no equivalent bounded `stop()` exists. |
| FTP readiness treats values as availability | `any(state)` rejects valid all-zero states and accepts one-sided availability. Startup proceeds after timeout. |
| No hardware-free executable hand pipeline | The main executable initializes DDS, image services, arms, and motion-mode infrastructure. `--sim` is not a mock backend. |
| Existing arm fallback helpers are not connected | The main loop directly calls `solve_ik()`; the inspected fallback helpers have tests but no production call site. |
| Replay lacks raw hand input and safety provenance | Episode recording stores robot state/action, not a reproducible XR-to-command hand trace. |
| Optimizer failure is hidden as success | The vendored optimizer catches `RuntimeError`, prints it, and returns the previous solution without a status. |
| Filter reset is incomplete | `SeqRetargeting.reset()` does not reset its filter or DexPilot projection history. |

### DFX bridge findings that matter to readiness

The inspected bridge already has a command timeout. Preserve it, but strengthen the surrounding behavior:

- Command access assumes twelve entries.
- Subscriber updates and runner reads are not protected by one coherent snapshot.
- Nonfinite values can reach float-to-integer conversion.
- `SetPosition()` casts before clamping, which is unsafe for extreme input values.
- Serial feedback failures leave old positions being published.
- The bridge increments `lost`; the Python controller currently ignores it.
- Serial `select()` reuses a mutable timeout object.
- Serial reads and writes do not robustly handle fragmented or partial transactions.
- `SetPosition()` reports success without validating its response.

These are software-side issues. They must not be deferred under “hardware validation.”

### Duplicated or misleading implementations

- DFX and FTP are legitimate different transports; their duplicated processing loops should be consolidated.
- The second `dex-retargeting/hand_retargeting.py` is an import probe, not a second production algorithm.
- `Unit_Test` enum variants encode working-directory differences; they are not mock hardware modes.
- Existing Rerun playback is visualization, not a safe command replay architecture.
- A simulated DDS hand is not equivalent to a backend that cannot access hardware.
- Existing timestamp additions supersede the parent integration document’s statement that timestamps are unavailable.

### Main architecture risks

1. Conflating “received once,” “valid now,” “fresh now,” and “armed.”
2. Keeping safety policy inside transport-specific loops.
3. Testing only with mocked retargeting and never executing DexPilot.
4. Applying additional smoothing without accounting for existing filtering.
5. Treating receipt of bridge state as proof of fresh serial feedback.
6. Using the full G1 executable for hand-only tests.
7. Accidentally changing wire order while “simplifying” motor mapping.
8. Claiming collision safety from joint limits or fingertip distance alone.
9. Losing existing user changes or submodule work.
10. Letting replay bypass safety or silently select real hardware.

---

## 4. Target architecture

Use one hand-processing pipeline and small source/backend adapters. Do not introduce a plugin framework, generic robotics bus, or replacement optimizer.

```text
Live Quest source ─┐
Synthetic source ──┼─> validated raw hand events
Replay source ─────┘            │
                               ▼
                 shared TeleVuer conversion
                               │
                               ▼
                   timestamped HandFrame
                               │
                               ▼
                existing DexPilot retargeting
                               │
                               ▼
             name-based six-motor selection
                               │
                               ▼
           normalized mapping + calibration
                               │
                               ▼
          deadzone → smoothing → rate limiting
                               │
                               ▼
       modeled thumb–index guard + final validation
                               │
                               ▼
                   armed command dispatcher
                      │        │        │
                    Mock      DFX      FTP
                      │        │        │
                      └── BackendState ─┘
                               │
                  freshness/fault supervision

At each boundary:
recording + structured telemetry + optional visualization
```

### Architectural decisions

- **DFX is the required production target.**
- Retain FTP compatibility through the common pipeline and its existing wire format.
- Use a dedicated hand-only executable.
- Keep the full G1 executable as an integration consumer of the same hand pipeline.
- Use a thread for the hand controller’s transport owner; do not inherit active DDS writers across a fork.
- Initialize DDS only when a DDS backend is explicitly selected.
- Constructing a source, processor, or backend must not send actuator commands.
- Live, synthetic, and replay inputs must share validation, conversion, mapping, and safety.
- Treat both hands as one armed session: either-hand loss stops advancing both hands. Track validity and freshness separately so the cause remains visible.
- Default timeout behavior is bounded hold followed by cessation of publication and a latched disarm. Do not automatically open the hand on disconnect.

---

## 5. Exact data contracts and processing order

### Raw XR contract

Each hand contains:

- Exactly 25 joint transforms.
- Each transform is 4×4, serialized column-major.
- Positions in meters.
- OpenXR world coordinates.
- Explicit side.
- Locally assigned receive sequence and monotonic receive time.
- Source tracking flags when supplied; do not invent tracking confidence.

Keep a documented 25-joint name/index table matching the existing stream. The current retargeter uses fingertips at indices `4, 9, 14, 19, 24`.

Validate shape, numeric type, finiteness, homogeneous bottom row, and rotation orthonormality before committing the snapshot.

Missing/untracked hands must be marked invalid, not represented as valid zero landmarks.

### `HandFrame`

Create a small immutable-style dataclass containing copied arrays:

```text
source
left/right source sequence
left/right received_at_monotonic
left/right valid
left/right invalid_reason
left/right wrist_world: 4×4
left/right landmarks_local: 25×3
```

No wall-clock timestamp is used for timeout decisions.

### `HandCommand`

```text
command_sequence
source sequences
created_at_monotonic
canonical normalized positions: 12
safety state
limiting/fault reasons
```

Only finite twelve-element vectors in the configured normalized range can reach a backend.

### `BackendState`

```text
received_at_monotonic
left/right feedback health
left/right feedback freshness
canonical measured positions: 12
transport status
serial-loss counters where available
```

Mock state must be labeled simulated. Do not present it as measured hardware state.

### Processing order

1. Validate and snapshot raw input.
2. Apply existing coordinate conversion.
3. Reject invalid, stale, or out-of-order source data.
4. Build existing configured landmark difference vectors.
5. Run each retargeter once per new source frame.
6. Capture solver status and reject nonfinite outputs.
7. Select six independent joints by name.
8. Convert radians into normalized openness.
9. Apply gain and offset.
10. Clamp to configured soft limits inside `[0, 1]`.
11. Apply deadzone against the previous accepted target.
12. Apply time-based smoothing.
13. Apply per-motor rate limits from the last accepted command.
14. Convert the candidate command back to model radians.
15. Expand mimic joints and check thumb–index geometry and the proposed transition.
16. Shorten the transition or hold the previous accepted command if blocked.
17. Revalidate finite values, limits, rate bounds, and modeled clearance.
18. Publish only if armed and feedback remains healthy.
19. Record the actual accepted/published result.

The safety limiter must return its accepted output to the filter state. Otherwise, hidden filter state can accumulate motion behind a clamp and produce a later jump.

---

## 6. Configuration structure

Create `config/inspire_hand.yaml`.

Keep robot kinematics and DexPilot parameters in the existing `assets/inspire_hand/inspire_hand.yml`. The new file contains runtime and calibration settings and references that existing configuration.

Recommended initial **software-test defaults**, subject to later physical tuning:

```yaml
schema_version: 1

source:
  kind: synthetic
  tracking_timeout_s: 0.25
  max_hand_skew_s: 0.05

runtime:
  control_hz: 50
  max_step_dt_s: 0.05
  start_armed: false

retargeting:
  config: assets/inspire_hand/inspire_hand.yml

backend:
  kind: mock
  state_timeout_s: 0.5
  connect_timeout_s: 5.0

safety:
  timeout_policy: hold_then_disarm
  hold_duration_s: 0.25
  require_explicit_rearm: true

filter:
  time_constant_s: 0.045

calibration:
  left:
    gain: [1, 1, 1, 1, 1, 1]
    offset: [0, 0, 0, 0, 0, 0]
    deadzone: [0, 0, 0, 0, 0, 0]
    min: [0, 0, 0, 0, 0, 0]
    max: [1, 1, 1, 1, 1, 1]
    max_rate_per_s: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
  right:
    gain: [1, 1, 1, 1, 1, 1]
    offset: [0, 0, 0, 0, 0, 0]
    deadzone: [0, 0, 0, 0, 0, 0]
    min: [0, 0, 0, 0, 0, 0]
    max: [1, 1, 1, 1, 1, 1]
    max_rate_per_s: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

collision:
  enabled: true
  clearance_m: 0.002
  path_step_rad: 0.005

recording:
  enabled: false
  queue_capacity: 1000

telemetry:
  summary_hz: 2
  visualization: false
```

Additional transport fields:

- DFX domain and network interface.
- Fixed supported topic names, unless explicitly configuring a matched bridge namespace.
- FTP connection settings.
- Profile identifier and calibration status.
- Mock feedback lag and fault-injection settings.

Rules:

- Resolve paths relative to the repository or configuration file, never the caller’s working directory.
- Reject unknown keys and invalid dimensions.
- Require positive finite timing/rate values.
- Require `min <= max`, with limits inside `[0, 1]`.
- Require nonnegative deadzones and positive gains.
- Keep motor ordering fixed by names; do not expose arbitrary wire permutations as casual tuning.
- Apply geometry scaling once through the existing retargeting scaling factor.
- No live configuration reload in the first implementation.
- Record the resolved configuration and hashes.

The collision margin and rate limits above are provisional test settings, not hardware certification.

---

## 7. File-by-file implementation inventory

### Existing files to change

| File | Required change |
|---|---|
| `teleop/televuer/src/televuer/televuer.py` | Validate hand events before writes; coherent snapshots; per-hand sequence, timestamp, validity, and diagnostics. |
| `teleop/televuer/src/televuer/tv_wrapper.py` | Reuse pure hand conversion; expose hand metadata in `TeleData`; process sides independently. |
| `teleop/televuer/src/televuer/__init__.py` | Allow pure hand conversion imports without starting or requiring live Vuer infrastructure. Preserve existing exports. |
| `teleop/televuer/tests/test_hybrid_input.py` | Preserve controller/hand separation and add metadata compatibility assertions. |
| `teleop/robot_control/hand_retargeting.py` | Absolute asset resolution; validate name mappings; accept Inspire runtime overrides; expose per-hand retarget/reset/status operations. |
| `teleop/robot_control/robot_hand_inspire.py` | Keep public controller names; replace duplicate pipelines with common runner/backend use; arm/disarm gate; coherent state; bounded lifecycle. |
| `assets/inspire_hand/inspire_hand.yml` | Preserve joint names and targets; clarify scaling applies to DexPilot too; document filter ownership. |
| `teleop/teleop_hand_and_arm.py` | Feed timestamped hand snapshots; propagate start/stop; use shared hand safety; stop hands promptly on faults; connect existing arm fallback helpers. |
| `teleop/inspect_hybrid_input.py` | Report hand freshness and validity instead of sticky readiness alone. |
| `teleop/utils/rerun_visualizer.py` | Add optional hand skeleton, target, command, and safety visualization. |
| `teleop/utils/episode_writer.py` | Preserve local timestamp work; add optional hand-trace correlation metadata without changing existing state/action layout. |
| `teleop/robot_control/test_inspire_sim_worker.py` | Replace process-preservation assertion with DDS ownership and no-write-before-arm assertions; retain mapping and stop tests. |
| `test/test_episode_writer_timestamps.py` | Preserve coverage; isolate temporary module stubs so they do not pollute unrelated tests. |
| `requirements.txt` | Document/include required compatible hand runtime dependencies without globally upgrading unrelated packages. |
| `README.md` | Hardware-free workflow, safety states, replay, preflight, readiness limitations. |

Small, scoped changes to the vendored retargeter:

| File | Change |
|---|---|
| `dex_retargeting/optimizer.py` | Expose success/failure status while preserving existing fallback return behavior. |
| `dex_retargeting/seq_retarget.py` | Reset optimizer history, filter state, and DexPilot projection state correctly. |

Do not rewrite optimizer mathematics.

### New files

| File | Purpose |
|---|---|
| `config/inspire_hand.yaml` | Runtime/calibration configuration. |
| `teleop/televuer/src/televuer/hand_frame.py` | Raw event parsing, immutable snapshots, pure coordinate conversion helpers. |
| `teleop/hand_sources.py` | Synthetic and recorded sources using the shared raw contract. |
| `teleop/robot_control/inspire_pipeline.py` | Config validation, canonical command types, mapping, calibration, filtering, state machine, processing runner. |
| `teleop/robot_control/inspire_backends.py` | Mock, DFX, FTP adapters and transport codecs; lazy SDK imports. |
| `teleop/robot_control/inspire_collision.py` | Thumb–index geometry checking using existing Pinocchio collision support. |
| `teleop/utils/hand_recording.py` | Versioned JSONL trace, bounded writer, validation, replay reader. |
| `teleop/teleop_inspire_hand.py` | Dedicated hand-only executable. |
| `teleop/hand_preflight.py` | Offline and explicit read-only hardware preflight. |
| `test/test_inspire_mapping.py` | Mapping/configuration/normalization tests. |
| `test/test_inspire_safety.py` | State machine, freshness, filtering, limits, collision tests. |
| `test/test_inspire_backends.py` | Mock and transport serialization/lifecycle tests. |
| `test/test_inspire_replay.py` | Recording/replay/validation tests. |
| `test/test_inspire_e2e.py` | Real retargeter hardware-free pipeline tests. |
| `teleop/televuer/tests/test_hand_frames.py` | Event parsing, snapshot consistency, transform tests. |
| `requirements-hand-test.txt` | Reproducible hardware-free test dependencies. |
| `scripts/check_inspire_software_ready.sh` | Single required readiness gate. |
| `docs/inspire_hand_validation.md` | Hardware procedures and tuning checklist. |
| `AGENTS.md` | Project-specific implementation/test rules, subordinate to parent instructions. |

Keep test fixture generation in `hand_sources.py`; use fixture files only where fixed recorded inputs or independent expected outputs add value.

### Required sibling DFX service changes

Because the real DFX backend depends on this bridge, these changes are part of software readiness:

- `../dfx_inspire_service/inspire_g1.cpp`
- `../dfx_inspire_service/include/dds/Subscription.h`
- `../dfx_inspire_service/include/inspire.h`
- `../dfx_inspire_service/include/SerialPort.h`
- `../dfx_inspire_service/include/param.h`
- `../dfx_inspire_service/CMakeLists.txt`
- New transport/command validation tests under that repository’s test directory.

During implementation, follow that repository’s own instructions and filesystem permissions. If sibling edits are unavailable, report `SOFTWARE_READY` as blocked; do not reclassify bridge defects as hardware-only work.

---

## 8. Phase 1 — Establish a reproducible software baseline

**Classification:** `SOFTWARE_ONLY`

**Objective:** Make subsequent results reproducible and preserve existing work.

**Files affected:** Dependency/test configuration, readiness script scaffold, project `AGENTS.md`, parent handoff/spec references.

**Exact tasks:**

1. Record main and submodule revisions and dirty files.
2. Preserve the current TeleVuer revision and episode timestamp changes.
3. Use Python 3.10 as the initial supported reference environment.
4. Separate hardware-free dependencies from optional DDS/FTP dependencies.
5. Include real retargeting dependencies in the hardware-free environment.
6. Make test invocation independent of globally installed ROS pytest plugins.
7. Inventory current tests; do not blindly collect hardware examples as tests.
8. Record the baseline test results from this analysis.
9. Establish an explicit list of mandatory readiness tests.
10. Correct stale timestamp documentation when updating the integration spec.

**Dependencies:** None.

**Tests:**

- Existing focused regression suite.
- Dependency import checks without constructing hardware controllers.
- Verify no source or backend sends commands during import.

**Expected result:** A known baseline and a repeatable test environment.

**Completion criteria:**

- Required dependencies resolve in the supported environment.
- Existing baseline tests pass.
- Pre-existing changes are preserved.
- Required tests are not silently skipped because packages are missing.

---

## 9. Phase 2 — Make XR input coherent, validated, and reproducible

**Classification:** `SOFTWARE_ONLY`

**Objective:** Establish a reliable input boundary used by live Quest, synthetic input, and replay.

**Files affected:** TeleVuer files, `hand_sources.py`, `test_hand_frames.py`, hybrid tests, inspector.

**Exact tasks:**

1. Extract hand-event parsing from `TeleVuer.on_hand_move()`.
2. Validate complete per-hand payloads before updating shared state.
3. Add coherent snapshot locking around data and metadata.
4. Assign receive timestamps and sequences only when new events arrive.
5. Expose per-hand validity and freshness metadata.
6. Preserve controller timestamps as separate input metadata.
7. Remove silent exception swallowing from the hand path; count and rate-limit diagnostics.
8. Extract the existing hand coordinate transform without changing its matrix conventions.
9. Validate rigid transforms before calling transpose-based inversion.
10. Process each hand independently; do not turn both into valid zeros after one-side failure.
11. Preserve `motion_data_ready` for compatibility, but stop using it as the hand safety signal.
12. Make the inspector display valid/stale/missing states explicitly.

### Synthetic Quest source

Generate anatomically structured 25-joint transforms in the same serialized format as a live `HAND_MOVE` event.

Required scenarios:

- Open hand.
- Fist.
- Index-only flexion.
- Each remaining finger independently flexed.
- Thumb bend.
- Thumb opposition/rotation.
- Thumb–index pinch.
- Left/right asymmetric gestures.
- Global translation and rotation of an unchanged hand pose.
- Small deterministic noise.
- Sudden pose jumps.
- One-hand disappearance.
- Both-hand disconnect.
- Malformed lengths.
- NaN/Inf.
- Degenerate transforms.
- Repeated sequence numbers.
- Out-of-order frames.
- Source pauses and variable frame intervals.

Use a fixed seed and explicit time input. Never require browser startup for synthetic tests.

Keep inverse-transform round-trip tests separate from independently specified landmark/gesture fixtures; an inverse generated from the same code cannot prove that convention is correct.

**Dependencies:** Phase 1.

**Tests:**

- Column-major parsing with independently specified transforms.
- Exact shape and finite-value rejection.
- No partial snapshot on invalid input.
- Per-hand dropout.
- Translation/rotation invariance of wrist-local landmarks.
- Controller events cannot refresh hand timestamps.
- Concurrent snapshot consistency.

**Expected result:** Live, synthetic, and replay sources can produce the same validated frame type.

**Completion criteria:**

- No invalid frame becomes actionable.
- Re-reading a frame does not refresh its age.
- A static but genuinely new frame remains valid.
- Synthetic execution needs no Quest, browser, DDS, image server, or G1.

---

## 10. Phase 3 — Consolidate retargeting and validate all twelve channels

**Classification:** `SOFTWARE_ONLY`

**Objective:** Preserve the working algorithm while making its mapping explicit and independently testable.

**Files affected:** `hand_retargeting.py`, `inspire_pipeline.py`, existing YAML, limited vendored status/reset changes, mapping tests.

**Exact tasks:**

1. Resolve assets from `Path(__file__)`, independent of working directory.
2. Retain `HandType` compatibility; make `Unit_Test` variants aliases for path-independent loading.
3. Add `retarget_hand(side, landmarks)` around existing vector construction and optimizer use.
4. Verify every configured motor name exists exactly once.
5. Verify all six selected motors are independent target joints.
6. Read normalization ranges from the validated model/mapping contract.
7. Centralize radians ↔ normalized command conversion.
8. Keep canonical left-first ordering internally.
9. Put DFX right-first conversion only at the transport boundary.
10. Expose solver failure status.
11. Implement complete per-hand reset, including filter and DexPilot projection history.
12. Disable the internal Inspire low-pass filter through runtime configuration when the common output filter is active; leave other hand types unchanged.
13. Apply the existing scaling factor once.

**Dependencies:** Phases 1–2.

**Tests:**

- Construct retargeters from repository root, `teleop/`, and an unrelated temporary directory.
- Endpoint and midpoint normalization for every channel.
- Round-trip normalization.
- Twelve unique sentinel values survive canonical → wire → canonical conversion.
- Missing, duplicate, and wrong-side joint names fail initialization.
- Mimic joints are excluded from command channels.
- Real DexPilot runs on synthetic open, fist, pinch, and asymmetric inputs.
- Solver error is observable and does not produce a new accepted command.
- Reset/replay starts without old filter or projection state.

**Expected result:** One mapping implementation with tested ordering and units.

**Completion criteria:**

- All twelve channels pass isolated mapping tests.
- Real retargeting executes without hardware.
- No hard-coded optimizer index list replaces the existing name-based mapping.
- DFX/FTP loops no longer need independent normalization implementations.

---

## 11. Phase 4 — Implement the shared safety and filtering pipeline

**Classification:** `SOFTWARE_ONLY`

**Objective:** Prevent invalid, stale, excessive, or modeled-colliding commands from reaching any backend.

**Files affected:** `inspire_pipeline.py`, `inspire_collision.py`, configuration, safety tests.

### State machine

Implement:

```text
DISARMED
  → ARMED, only after explicit arm and successful checks

ARMED
  → HOLDING on input invalidity, timeout, or recoverable processing failure
  → FAULT on transport/feedback failure or unsafe initialization

HOLDING
  → DISARMED after bounded hold period
  → no automatic resume

FAULT
  → DISARMED only after explicit reset and successful preflight
```

On either-hand failure, freeze both command targets. Require both sides to be fresh before rearming.

### Exact tasks: limits and calibration

1. Validate finiteness before arithmetic and before publication.
2. Apply `u = gain × normalized + offset`.
3. Clamp to per-motor configured limits inside protocol bounds.
4. Apply deadzone against the previous accepted target.
5. Preserve input scale as a separate retargeting parameter.
6. Make every clamp observable through reason flags and counters.

### Exact tasks: smoothing and rate limits

Use:

```text
alpha = 1 - exp(-dt / tau)
filtered = previous + alpha × (target - previous)
```

- Use monotonic time.
- Clamp effective `dt` after scheduler stalls.
- Rate-limit relative to the last accepted command.
- Seed from validated feedback when arming.
- Reset on disarm, source change, replay restart, and fault.
- Process the optimizer only for new source frames.
- Keep filtering/safety ticking independently from source arrival.
- Feed the final accepted command back into filter state.

### Exact tasks: thumb–index collision handling

1. Build collision geometry using the existing hand URDFs and installed Pinocchio collision support.
2. Select thumb-versus-index collision pairs by link names for each hand.
3. Include relevant phalanges, not just fingertip points.
4. Convert calibrated normalized commands back to model radians.
5. Apply existing mimic-joint relationships before geometry updates.
6. Check the proposed motion from the previous accepted command using bounded joint-space subdivisions.
7. Stop at the last clearance-valid subdivision and refine that interval if needed.
8. Return the previous command when no admissible progress exists.
9. Fault if initial feedback is already invalid under the configured model; do not invent an escape maneuver.
10. Treat geometry loading failure as a failed safety initialization.
11. Report minimum modeled clearance and limiting pairs.

This is a discrete model-based guard at a documented resolution. It is not proof of physical clearance. Physical mesh accuracy, compliance, and margin remain hardware calibration tasks.

### Timeout and disconnect policy

- No new input: bounded hold, then stop publication and latch disarmed.
- Invalid backend feedback or transport failure: cease publication and fault.
- No first valid frame: no command.
- No first valid state: no command.
- Reconnect: explicit rearm.
- Shutdown: cease publishing; do not automatically open or close.

Stopping publication is not a torque-off guarantee. Document bridge and device behavior separately.

**Dependencies:** Phases 2–3.

**Tests:**

- Fake-clock timeout boundaries.
- No command before arm.
- Zero-valued valid feedback.
- NaN/Inf at each processing boundary.
- Gain, offset, deadzone, and limit interactions.
- Rate bounds under jitter and long pauses.
- Filter response at different control rates.
- No hidden motion accumulation behind a clamp.
- Thumb–index safe, blocked, and crossing trajectories.
- Both mirrored hand models.
- Explicit rearm after dropout.
- Collision geometry failure blocks readiness.

**Expected result:** One deterministic safety path shared by all backends.

**Completion criteria:**

- Every published command is finite, bounded, rate-limited, and accepted by the configured model guard.
- Safety behavior is tested without sleeps or hardware.
- No fallback silently turns invalid input into a fresh target.

---

## 12. Phase 5 — Implement mock and production command backends

**Classification:** `SOFTWARE_ONLY`

**Objective:** Separate hand computation from transport and make transport behavior testable.

**Files affected:** `inspire_backends.py`, `robot_hand_inspire.py`, worker/backend tests.

### Common backend interface

Use a small protocol:

```text
connect()
read_state(now)
send(command)
close()
```

The runner owns arm/disarm and safety. Backends validate the command boundary again.

### Mock backend

Implement:

- Deterministic initial positions.
- Configurable first-order lag toward accepted commands.
- Bounded command history.
- Timestamped simulated feedback.
- Fault injection for stale feedback, disconnect, write failure, malformed state, and one-sided loss.
- Explicitly simulated state metadata.
- Idempotent close.
- No DDS/serial/socket imports or initialization.

The mock must reject unsafe commands itself; it must not repair invalid pipeline output and hide a test failure.

### DFX backend

1. Preserve topics and message types.
2. Preserve right-first wire ordering.
3. Initialize DDS in the selected runtime, once.
4. Own publishers/subscribers in the same process/thread where they are used.
5. Validate state length, finiteness, bounds, and health.
6. Read `lost` counters and detect continued failed serial reads.
7. Keep separate receipt age and feedback-health state.
8. Make publisher failure visible.
9. Close resources on startup failure and normal shutdown.
10. Do not write in constructors or `connect()`.

### FTP backend

1. Preserve separate left/right topics and integer angle encoding.
2. Use the shared pipeline.
3. Replace value-based availability checks with explicit receipt metadata.
4. Require both states before arming.
5. Use bounded connect timeout.
6. Add explicit worker ownership and bounded stop.
7. Test integer rounding and boundaries.
8. Lazy-load `inspire_sdkpy`.

FTP hardware remains outside the DFX readiness claim; its shared safety and adapter behavior must still pass software tests.

### Existing controller compatibility

Retain public controller class names as thin wrappers around the common runner. Remove duplicate retargeting loops.

Do not retain a mode that defaults to “fresh forever” when metadata is absent.

**Dependencies:** Phases 3–4.

**Tests:**

- Identical canonical commands delivered through mock, fake DFX, and fake FTP transports.
- Zero writes during import, construction, connection, and preflight.
- State ordering round-trip.
- Malformed feedback rejection.
- One-sided FTP state loss.
- Publisher exceptions.
- Idempotent bounded stop.
- No inherited DDS writer.
- No worker leak after initialization failure.

**Expected result:** A mock backend and production adapters behind one tested interface.

**Completion criteria:**

- Pipeline tests run without Unitree or Inspire SDKs installed.
- Production adapters receive transport-contract tests.
- No production path bypasses common safety.

---

## 13. Phase 6 — Harden the DFX bridge without physical hardware

**Classification:** `SOFTWARE_ONLY`

**Objective:** Close receiver-side defects that client-only tests cannot cover.

**Files affected:** Sibling DFX service files listed earlier.

**Exact tasks:**

1. Add a locked subscriber snapshot containing message and receive timestamp.
2. Stop reading mutable subscriber messages directly.
3. Validate exact command length and finite normalized values.
4. Do not refresh accepted-command age for rejected messages.
5. Preserve the existing command timeout, make its configured value explicit, and test expiration.
6. Clamp validated floating-point values before integer conversion.
7. Stop forwarding commands when required serial feedback is unhealthy.
8. Preserve and expose existing loss counters.
9. Avoid presenting initialization zeros as validated serial feedback.
10. Repair serial deadlines:
    - Recreate `select()` timeout for each wait.
    - Handle `EINTR`.
    - Accumulate partial reads until complete or deadline.
    - Handle partial writes and write failures.
11. Validate protocol response length, checksum, expected identity, and operation before reporting success.
12. Exercise those transactions with a pseudo-terminal emulator.
13. Build the service without starting a hardware-connected executable.
14. Retain serial path configuration and right/left ownership.

**Dependencies:** Phase 5’s transport contract.

**Tests:**

- Empty, short, oversized, nonfinite, and out-of-range commands.
- Concurrent subscriber/runner snapshot stress.
- Timeout with no command and after the last accepted command.
- Fragmented serial replies.
- Bad checksums and wrong reply identity.
- Missing replies and partial writes.
- Feedback failure stops serial command forwarding.
- Restored serial communication does not bypass the client’s rearm policy.

**Expected result:** A bridge that validates its own trust boundary.

**Completion criteria:**

- Build and protocol tests pass without physical serial devices.
- No stale or malformed command is treated as newly accepted.
- Serial failure is observable upstream.
- Remaining unknowns concern the real device, not unimplemented error handling.

---

## 14. Phase 7 — Add the hand-only executable and integrate existing teleoperation

**Classification:** `SOFTWARE_ONLY`

**Objective:** Run the complete hand pipeline without initializing G1 control.

**Files affected:** `teleop_inspire_hand.py`, `teleop_hand_and_arm.py`, inspector, configuration, regression tests.

### Dedicated executable

Provide:

```text
--source synthetic|quest|replay
--backend mock|dfx|ftp
--config PATH
--duration SECONDS
--record PATH
--replay PATH
--replay-mode inputs|commands
--visualize
--preflight
--enable-hardware
```

Defaults:

```text
source=synthetic
backend=mock
disarmed
```

Rules:

- Real backend selection requires explicit hardware enablement.
- Hardware enablement does not arm automatically.
- Synthetic source to real hardware is prohibited.
- Replay to real hardware is prohibited in the initial implementation.
- Quest → mock is supported.
- Real Quest → DFX requires successful preflight and explicit arm.
- The hand-only executable never imports or constructs arm control, locomotion, motion switching, or image clients.
- Provide deterministic scripted arm/disarm events for mock tests.
- Use `r` to arm and `q` to disarm/exit for interactive operation.

### Existing full teleoperation integration

1. Replace shared landmark-only hand input with coherent frame metadata.
2. Propagate application start/disarm to the hand runner.
3. Stop the hand runner promptly when the main loop fails or exits.
4. Preserve canonical episode hand action/state ordering.
5. Connect the existing arm tracking fallback helpers using genuine source timestamps.
6. Test the arm integration with fake arm controllers only.
7. Preserve existing G1 mode restoration and arm shutdown behavior outside the necessary hand changes.
8. Ensure a hand fault can be diagnosed without silently continuing stale hand motion.

**Dependencies:** Phases 2–6.

**Tests:**

- Subprocess synthetic → mock run with hardware constructors patched to fail if called.
- Quest-source constructor does not require an image server in pass-through mode.
- No command before start.
- Source loss and SIGINT shut down within a bounded interval.
- Main-executable hand wiring with fake controllers.
- Existing arm-message, mode-restoration, hybrid input, and CLI/UI regressions.

**Expected result:** One safe hand-only command and one shared integrated path.

**Completion criteria:**

- Entire synthetic → real DexPilot → mock path runs without Quest, Inspire, G1, cameras, DDS, or simulator.
- No physical G1 movement is part of this phase.
- `--sim` remains a separate DDS simulation feature, not the software-readiness mechanism.

---

## 15. Phase 8 — Add recording and safe replay

**Classification:** `SOFTWARE_ONLY`

**Objective:** Reproduce hand-processing behavior, including failures and safety decisions.

**Files affected:** `hand_recording.py`, hand runner, sources, episode writer integration, replay tests.

### Record format

Use append-only JSONL with a versioned header.

Header:

```text
type: header
schema_version
UTC session start
clock: monotonic-relative-seconds
source kind
joint-name order
coordinate convention
resolved configuration
configuration/model hashes
repository/submodule revisions
```

Frame/tick records:

```text
type: frame
tick sequence
relative monotonic time
source sequences and receive times
raw XR transforms when available
per-hand validity and reasons
converted landmarks
retargeted motor radians
calibrated target
accepted command
published/not-published
backend state and health
safety state/reasons
stage durations
```

Footer:

```text
type: footer
clean shutdown flag
frame count
drop/error count
final status
```

Serialization rules:

- Reject NaN/Inf rather than emitting nonstandard JSON.
- Encode absent measurements as `null` with an explicit reason.
- Do not reuse absolute monotonic timestamps across processes.
- Rebase replay onto its own monotonic clock.
- Retain source age and gaps.

### Replay modes

**Input replay:** Raw XR records pass through parsing, conversion, retargeting, calibration, and safety.

**Command replay:** Recorded canonical commands pass through limits, rate handling, collision checks, arming, and backend validation. Do not calibrate/filter already processed commands twice.

Both modes default to mock and are restricted to mock in this implementation.

### Exact tasks

1. Implement a bounded recording queue and background writer.
2. Detect queue overflow and storage failure.
3. Mark a recording incomplete on dropped records.
4. Disarm when an explicitly requested recording cannot continue reliably.
5. Validate schema, dimensions, ordering, values, and timestamps before replay.
6. Reset optimizer/filter/source state before replay.
7. Support deterministic virtual-time replay and paced replay.
8. Reject incompatible model/config hashes by default for deterministic comparison.
9. Allow explicit offline diagnostic override, prominently marked non-equivalent.
10. Preserve `EpisodeWriter`’s current format and timestamp changes.
11. Add hand-trace path/sequence correlation without replacing existing 26-D episode semantics.

**Dependencies:** Phases 2–7.

**Tests:**

- Record → read → replay with equivalent accepted commands within declared tolerance.
- Dropout and timeout reproduce at the same logical times.
- Truncated final line is reported as incomplete.
- Malformed schema/order/timestamps rejected.
- Queue overflow and disk write error are visible.
- Command replay cannot bypass safety.
- Hardware replay attempts are rejected before DDS initialization.
- Existing timestamp tests remain passing.

**Expected result:** Reproducible hand traces usable for debugging and regression fixtures.

**Completion criteria:**

- Replay runs fully offline.
- “Input replay” and “command replay” are unambiguous.
- No trace silently claims completeness after data loss.

---

## 16. Phase 9 — Add diagnostics, visualization, and preflight

**Classification:** `SOFTWARE_ONLY`

**Objective:** Make software state and hardware prerequisites observable.

**Files affected:** Visualizer, runner, recording module, `hand_preflight.py`, inspector, docs.

### Telemetry and logging

Expose:

- Input rate and age per hand.
- Validity and rejection counts.
- Current source sequences.
- Retargeting duration and failure count.
- Raw target, limited target, and published command.
- Backend state age.
- Serial loss-counter changes.
- Tracking error when real feedback exists.
- Clamp, rate-limit, and collision events.
- Minimum modeled thumb–index clearance.
- Safety state and last transition reason.
- Loop overruns.
- Recording queue depth and failures.

Use rate-limited summaries and structured transition/error logs. Do not print every command at INFO.

### Visualization

Reuse Rerun:

- Left/right skeletons with joint indices.
- Wrist axes and local coordinate axes.
- Per-motor target/command/state plots.
- Thumb/index geometry and clearance.
- Stale or invalid hand indicators.
- Safety state timeline.

Visualization is optional and lazily imported. A missing viewer must not affect control.

### Offline preflight

Check:

1. Configuration validity.
2. Required imports and supported versions.
3. Assets and hashes.
4. Joint names, limits, and mimic relationships.
5. Collision geometry loading.
6. Retargeter initialization.
7. Mock E2E gesture sequence.
8. Record/replay round-trip.
9. No hardware initialization.
10. Required DFX bridge software-test status.

### Read-only hardware preflight implementation

Implement, but do not run physical checks in this phase:

- Explicit NIC/domain selection.
- Read-only subscription to DFX state.
- Exact state shape/order/range checks.
- Both-hand receipt and healthy feedback counters.
- Bounded observation period.
- No actuator-command publisher.
- Quest live input → mock validation.
- Certificate/file/port diagnostics for the Quest source.

Report results as:

```text
PASS
FAIL_SOFTWARE
NOT_RUN_HARDWARE
```

Do not convert `NOT_RUN_HARDWARE` into a software test failure or a hardware pass.

**Dependencies:** Phases 5–8.

**Tests:**

- Hardware publisher constructors forbidden during preflight.
- Missing assets/config/modules produce actionable failures.
- One-sided and stale feedback fail read-only readiness.
- Viewer-disabled execution remains functional.
- Logs include fault causes without per-frame flooding.

**Expected result:** A single diagnostic path explains whether the problem is input, processing, transport, or hardware availability.

**Completion criteria:**

- Offline preflight passes without hardware.
- Read-only preflight is proven not to publish commands.
- Diagnostics can distinguish stale DDS receipt from unhealthy serial feedback.

---

## 17. Phase 10 — Run the complete software-readiness gate

**Classification:** `SOFTWARE_ONLY`

**Objective:** Demonstrate the whole supported software path before physical validation.

**Files affected:** E2E tests, readiness script, test dependency file, regression tests.

### Required unit tests

- Raw payload validation.
- Coordinate conversion.
- Freshness and sequence handling.
- Mapping and normalization.
- Configuration validation.
- Gain/offset/deadzone.
- Soft limits and finite guards.
- Filtering and rate limits.
- Safety state transitions.
- Collision guard.
- Backend serialization.
- Feedback health.
- Recording/replay validation.

### Required integration tests

- Shared parser → conversion → real DexPilot.
- Pipeline → mock backend → feedback.
- Pipeline → fake DFX endpoint.
- Pipeline → fake FTP endpoint.
- Main teleoperation hand wiring with fake arm interfaces.
- DFX bridge command validation and serial emulator.
- Episode trace correlation.

### Required hardware-free E2E scenarios

1. Startup disarmed.
2. Explicit arm after valid input and state.
3. Open → fist → open.
4. Independent finger movement.
5. Left/right asymmetric gestures.
6. Pinch and thumb opposition.
7. Wrist translation/rotation without changing local finger pose.
8. Input noise.
9. Large target jump.
10. One-hand loss.
11. Both-hand disconnect.
12. Malformed and nonfinite frames.
13. Retargeter failure.
14. Stale backend feedback.
15. Transport write failure.
16. Reconnect without automatic rearm.
17. Record and deterministic input replay.
18. Command replay through safety.
19. SIGINT and repeated close.
20. Recording failure.
21. No-hardware import/runtime guard.

### Performance and endurance

- Run at least a ten-minute synthetic/mock session.
- Report processing duration percentiles and missed deadlines.
- Verify bounded queues/history and no growing worker count.
- Verify each safety deadline under injected stalls.
- Distinguish virtual-clock correctness from real wall-clock performance.
- Require the configured control rate to be achievable on the declared reference machine; lower the documented default if measured results require it.
- Do not advertise 100 Hz because the old worker default was 100 Hz.

### Readiness gate behavior

The gate must:

- Run required tests explicitly.
- Fail on required skips.
- Fail on nonzero exit codes.
- Include the real optimizer, not only mocked retargeting.
- Include bridge software tests.
- Produce a machine-readable report.
- Record environment and revisions.
- Never launch G1 control or physical serial connections.

**Dependencies:** Phases 1–9.

**Expected result:** A repeatable binary software acceptance decision.

**Completion criteria:**

- Every required test passes.
- No known software-fixable failure remains.
- Only the hardware validation and calibration phases below remain pending.

---

## 18. Phase 11 — Finalize documentation and handoff

**Classification:** `SOFTWARE_ONLY`

**Objective:** Let another operator reproduce readiness and safely begin hardware validation.

**Files affected:** `README.md`, project `AGENTS.md`, validation guide, parent integration/handoff documents.

**Exact tasks:**

1. Document installation for the hardware-free environment.
2. Document the synthetic/mock command.
3. Document Quest → mock inspection.
4. Document recording and both replay modes.
5. Document safety states and rearming.
6. Explain canonical versus DFX wire order.
7. State that twelve motors means six per hand.
8. Document configuration units and tuning ownership.
9. Document the collision model’s limitations.
10. Document DFX/FTP support status separately.
11. Update timestamp documentation to match current code.
12. Link the readiness report and exact gate command.
13. Record outstanding hardware checks as `NOT_RUN_HARDWARE`.
14. Preserve broader G1/LeRobot contract documentation.
15. Add project agent rules:
    - Do not command hardware during software phases.
    - Do not bypass common safety.
    - Preserve mappings and user modifications.
    - Run readiness gate before declaring completion.
    - Never label mock feedback as measured hardware feedback.

**Dependencies:** Phase 10.

**Tests:** Verify documented software-only commands against the readiness report and ensure they reference the implemented CLI and configuration schema.

**Expected result:** Complete implementation handoff.

**Completion criteria:** Documentation commands reproduce the reported software result, and no required software task remains deferred.

---

## 19. Quest 3 validation procedure

**Classification:** `HARDWARE_VALIDATION_REQUIRED`

**Objective:** Validate the real browser/XR stream against the software contract.

**Files affected:** Validation report and, if needed, recorded input fixtures. Code changes are required only if hardware evidence reveals a defect.

**Dependencies:** `SOFTWARE_READY`.

**Procedure:**

1. Run the hand-only executable with `source=quest`, `backend=mock`.
2. Keep all G1 and real-hand command backends disabled.
3. Verify HTTPS certificate trust and browser permissions.
4. Verify twenty-five joints per hand and units.
5. Verify left/right identity using asymmetric gestures.
6. Inspect open hand, fist, index flexion, thumb bend, opposition, and pinch.
7. Translate/rotate the wrists while keeping fingers fixed.
8. Confirm converted local landmarks stay consistent.
9. Occlude one hand, then both.
10. Leave XR mode, background the browser, disconnect Wi-Fi, and reconnect.
11. Confirm stale-input hold/disarm timing.
12. Confirm controller events cannot conceal hand loss.
13. Record a representative session.
14. Replay it offline and compare results.
15. Measure actual frame rate and latency.

**Tests:** Real input → mock safety/recording checks.

**Expected result:** Validated Quest event semantics and recorded real-world fixtures.

**Completion criteria:** Hand identity, coordinate conventions, freshness, dropout, and replay behavior are verified.

No physical G1 movement is needed.

---

## 20. Inspire Hand validation procedure

**Classification:** `HARDWARE_VALIDATION_REQUIRED`

**Objective:** Verify software assumptions against the actual Inspire DFX hardware.

**Files affected:** Hardware validation report and calibrated profile.

**Dependencies:** `SOFTWARE_READY`, successful Quest → mock validation, explicit authorization for physical hand commands.

**Procedure:**

1. Secure the hands and keep the work area clear.
2. Keep G1 arm/leg control disabled; use the hand-only executable.
3. Confirm device variant, firmware, serial paths, IDs, and right/left wiring.
4. Run read-only preflight.
5. Verify both hands provide plausible feedback.
6. Confirm no command is sent during connect/preflight.
7. Confirm loss counters behave during a deliberately disconnected serial link.
8. Arm with validated feedback as the initial command.
9. Make small, rate-limited changes to one motor at a time.
10. Verify all twelve channels, direction, and feedback correspondence.
11. Verify the normalized open/closed convention within conservative ranges.
12. Tune soft limits before attempting full gestures.
13. Validate thumb rotation and bend separately before combined pinch.
14. Validate modeled clearance with generous margin.
15. Exercise operator disarm, source loss, process exit, and bridge timeout.
16. Observe actual device behavior after publication stops.
17. Verify reconnect requires deliberate rearm.
18. Record and review measured versus commanded trajectories.

**Tests:** One-motor-at-a-time mapping, bounded gestures, disconnect behavior.

**Expected result:** Confirmed transport/motor semantics and a safe initial physical profile.

**Completion criteria:** All twelve channels, feedback health, limits, and stop behavior are verified.

Do not add G1 movement as an implicit step.

---

## 21. Calibration and tuning checklist

**Classification:** `HARDWARE_VALIDATION_REQUIRED`

**Objective:** Tune parameters after the implementation is complete.

**Files affected:** Versioned calibration profiles and validation report.

**Dependencies:** Successful Quest and Inspire validation.

### Input and geometry

- Confirm meters and joint indices.
- Confirm handedness and local axes.
- Confirm actual tracked hand size.
- Tune retargeting scale once.
- Compare model fingertips and physical fingertips.
- Check thumb/index mesh alignment and mimic behavior.

### Per-motor calibration

- Record conservative usable endpoints.
- Tune soft minimum/maximum.
- Tune offset before gain.
- Tune gain without changing wire order or units.
- Set deadzone above observed tracking noise.
- Verify independent and combined motion.

### Dynamics

- Measure input and feedback rates.
- Tune filter time constant.
- Tune per-motor maximum command rate.
- Check responsiveness during pinch.
- Check jitter in a stationary pose.
- Check behavior after a scheduler or network pause.

### Safety

- Tune collision margin conservatively.
- Verify thumb–index transitions, not only endpoints.
- Validate feedback-loss thresholds.
- Validate input timeout.
- Verify actual stopped-publication behavior.
- Confirm no automatic reopen, reclose, or rearm.

**Tests:** Repeat the affected mapping, gesture, collision-clearance, latency, and disconnect checks after each profile change; replay captured input offline before another physical trial.

**Expected result:** A calibrated profile tied to the physical hands and tested software revision.

**Completion criteria:** Parameters, hardware identity, test results, and remaining limitations are recorded.

---

## 22. Rollback and safety strategy

### Software rollback

- Preserve the pre-implementation revision and configuration.
- Keep changes separated by implementation phase.
- Do not reset unrelated working-tree changes.
- Record submodule revisions explicitly.
- Roll back code and its matching configuration/schema together.
- Rerun the software gate after rollback.

### Operational rollback

On abnormal behavior:

1. Disarm and stop publishing.
2. Use the physical stop/isolation procedure where necessary.
3. Preserve logs and trace files.
4. Return to Quest → mock or replay → mock.
5. Diagnose the captured sequence.
6. Rerun the software gate before physical retries.

Do not restore an unsafe stale-command implementation merely because it was previously used.

### Prohibited shortcuts

- Disabling collision checks to make a hardware demo proceed.
- Replacing missing state with zero arrays marked valid.
- Treating a new DDS message containing failed serial reads as healthy feedback.
- Removing timeouts to avoid disconnects.
- Automatically arming after reconnect.
- Using `--sim` as proof that no hardware can be reached.
- Using full G1 teleoperation for hand-only diagnostics.

---

## 23. What GPT-5.6 must not blindly rewrite

1. **The name-based motor mapping.** It correctly bridges model ordering and hardware ordering. Numeric optimizer indices are not the stable contract.

2. **Canonical left-first and DFX right-first ordering.** The inspected application and bridge agree.

3. **The existing normalization direction and URDF ranges.** Consolidate and validate them; do not reverse them based on generic servo assumptions.

4. **The existing coordinate-transform matrices.** Extract them and test independently before changing mathematics.

5. **DexPilot’s optimization objective and fingertip projection.** Add observability and correct reset behavior; do not replace the solver with an unrequested angle heuristic.

6. **URDF mimic joints.** They represent coupled finger motion and must remain part of kinematics and collision checking.

7. **The existing low-pass behavior without accounting for it.** Move ownership deliberately; do not stack filters accidentally.

8. **DFX topic names and message types.** Changing them breaks the bridge and existing integration.

9. **The simulation thread fix.** Preserve in-process DDS ownership; extend that principle to the real hand runtime.

10. **Existing timestamp additions.** They are local user work and have passing coverage.

11. **Hybrid controller/hand separation.** Controller events must not replace hand wrist poses or refresh hand freshness.

12. **G1 arm shutdown, message composition, and mode restoration.** Preserve existing tested behavior outside the narrowly required integration.

13. **The main Web UI.** It does not need a redesign for hand software readiness. Maintain CLI compatibility and regression tests.

14. **The broader 26-D recording/LeRobot contract.** A new twelve-channel hand trace supplements it; it does not replace it.

---

## 24. Strict execution order

Follow this sequence without introducing a different architecture:

1. **Phase 1:** Preserve baseline, establish dependencies and explicit test gate.
2. **Phase 2:** Implement validated snapshots, source metadata, shared conversion, and synthetic input.
3. **Phase 3:** Fix paths, centralize mapping, expose solver status, and complete resets.
4. **Phase 4:** Implement safety state machine, calibration, filtering, rate limits, and collision guard.
5. **Phase 5:** Implement mock and production adapters; remove duplicated processing.
6. **Phase 6:** Harden and test the DFX bridge using software-only protocol emulation.
7. **Phase 7:** Add the hand-only executable and integrate the shared hand pipeline.
8. **Phase 8:** Add trace recording and offline replay.
9. **Phase 9:** Add telemetry, visualization, and offline/read-only preflight.
10. **Phase 10:** Pass the complete hardware-free acceptance gate and endurance run.
11. **Phase 11:** Finalize documentation and handoff.
12. Declare **`SOFTWARE_READY`** only if every condition below is satisfied.
13. Stop software implementation work; proceed to hardware phases only when hardware and authorization are available.

### Mandatory `SOFTWARE_READY` checklist

- [ ] Synthetic raw Quest-format events use the production parser and conversion.
- [ ] Real DexPilot executes in hardware-free tests.
- [ ] All twelve motor channels are independently validated.
- [ ] Canonical/wire ordering is tested in both directions.
- [ ] Configuration and paths work independently of working directory.
- [ ] Finite guards, limits, calibration, deadzone, filtering, and rate limits pass.
- [ ] Thumb–index modeled collision handling passes.
- [ ] No command is sent before explicit arming.
- [ ] Freshness, disconnect, feedback failure, and rearm behavior pass.
- [ ] Mock backend requires no hardware SDK or DDS initialization.
- [ ] DFX/FTP adapter software tests pass.
- [ ] DFX bridge validation and serial-emulator tests pass.
- [ ] Record/replay reproduces expected behavior.
- [ ] Diagnostics and read-only preflight are implemented and tested.
- [ ] No hand-only path initializes G1 control.
- [ ] Existing relevant regressions pass.
- [ ] Required readiness tests contain no unexplained skips.
- [ ] Endurance results and environment are recorded.
- [ ] README, agent instructions, and handoff are current.
- [ ] Remaining work is explicitly limited to hardware validation, calibration, and tuning.

**Final acceptance statement:**

> `SOFTWARE_READY`: the complete Quest-format input → existing retargeter → twelve-channel Inspire command pipeline passes software-only validation, including safety, mock execution, transport contracts, recording, replay, and preflight. Physical Quest behavior, Inspire motion, calibration, and real collision clearance remain `HARDWARE_VALIDATION_REQUIRED`.
 