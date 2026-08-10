# Sim ↔ Real and Testing in Simulation

This guide covers how to use `teleop_hand_and_arm.py` parameters for **testing in simulation** and **moving to the physical robot (sim → real)**.

> Entry point: `teleop/teleop_hand_and_arm.py`  
> Sim engine: [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) (separate repo)

---

## Quick summary

| Mode | Flag | DDS Domain | Meaning |
|---|---|---|---|
| **Simulation** | pass `--sim` | `1` | Talk to Isaac Lab on DDS domain 1 |
| **Real (physical)** | **omit** `--sim` | `0` | Talk to the physical Unitree robot on domain 0 |

There is no `--real` flag — real mode is the default when `--sim` is not set.

**Sim → Real:** use the same teleop command and **remove `--sim`**, then make sure the image server / hand service on PC2 are running.

---

## 1. Parameters related to Sim / Real

### 1.1 Main switch

| Parameter | Type | Default | Description |
|---|---|---|---|
| `--sim` | flag (`store_true`) | off | Enable Isaac simulation mode |

When `--sim` is set, the system will:

1. Use **DDS domain ID = 1** (real uses `0`) — keeps traffic isolated from a physical robot
2. **Skip** switching the robot into debug mode (`MotionSwitcher.Enter_Debug_Mode`)
3. Pass `simulation_mode=True` into arm/hand controllers → command-only mode (often skip waiting on lowstate)
4. Enable sim-only topics:
   - publish `rt/reset_pose/cmd` (scene reset after saving an episode)
   - subscribe `rt/sim_state` (record sim state into the episode)

### 1.2 Parameters shared by sim and real

| Parameter | Options | Default | Notes |
|---|---|---|---|
| `--arm` | `G1_29`, `G1_23`, `H1_2`, `H1`, `H2` | `G1_29` | Arm type |
| `--ee` | `dex1`, `dex3`, `inspire_ftp`, `inspire_dfx`, `brainco` | *(required)* | End-effector |
| `--input-mode` | `hand`, `controller` | `hand` | XR tracking source |
| `--display-mode` | `immersive`, `ego`, `pass-through` | `immersive` | Headset display mode |
| `--img-server-ip` | IPv4 | `192.168.123.164` | Image server IP (PC2 / sim camera) |
| `--image-transport` | `auto`, `webrtc`, `zmq` | `auto` | How images are sent to XR |
| `--network-interface` | e.g. `eth0` | *(empty)* | CycloneDDS network interface |
| `--frequency` | float | `30.0` | Control / record rate |
| `--record` | flag | off | Enable data recording |
| `--task-dir` / `--task-name` / `--task-goal` / `--task-desc` / `--task-steps` | string | see argparse | Episode metadata |
| `--headless` | flag | off | No display |
| `--motion` | flag | off | Co-run with motion controller (walking) — mainly for physical |
| `--ipc` | flag | off | Control via IPC instead of keyboard |
| `--affinity` | flag | off | Pin CPU / raise priority |

### 1.3 Waist body-tracking parameters

| Parameter | Options | Default | Restriction |
|---|---|---|---|
| `--body-tracking` | `off`, `upper` | `off` | `upper` requires hand input and G1_29 |
| `--allow-real-waist` | flag | off | real only; requires `upper` and `--motion` |
| `--dry-run-waist` | flag | off | requires `upper`; incompatible with DDS/motion opt-in |
| `--tracking-timeout` | positive float | `0.5` | applies independently to arm and body frames |
| `--tracking-fallback` | `hold`, `home` | `home` | applies to both arms and waist |

When using `--body-tracking upper`, all of the following are required or argparse will error:

- `--input-mode hand`
- `--arm G1_29`
- simulation: `--sim` and no `--motion`
- real command: `--motion --allow-real-waist`
- real log-only validation: `--dry-run-waist` and neither real-control flag

`--allow-real-waist` is deliberately not implied by any other flag. Omitting it prevents real waist commands.

Initial waist safety envelope:

- software position limits: yaw ±25°, roll ±12°, pitch ±12°
- maximum DDS-cycle change at 250 Hz: yaw 0.002 rad, roll/pitch 0.0015 rad
- real `arm_sdk` blend handover: 0→1 over two seconds after `[r]`, then 1→0 over two seconds on exit
- body freshness uses its own `BODY_MOVE` timestamp; the default `home` fallback ramps stale waist targets toward calibrated zero

### 1.4 Behavioral differences: Sim vs Real

| Aspect | Sim (`--sim`) | Real (no `--sim`) |
|---|---|---|
| DDS domain | `1` | `0` |
| Debug-mode switch | skipped | calls `Enter_Debug_Mode()` unless `--motion` is selected |
| Arm velocity clip | off | on |
| Dex3 / Dex1 state wait | often skipped (command-only) | wait for DDS state |
| Dex1 filter / delta clip | off | on (safer) |
| Image server | from Isaac (`--enable_cameras`) | run `teleimager` on PC2 |
| Hand DDS | enabled in sim (`--enable_dex3_dds` / `--enable_inspire_dds`) | onboard / PC2 services |
| After episode save | publish reset to `rt/reset_pose/cmd` | none |
| Record `sim_state` | yes (from `rt/sim_state`) | no |

For opted-in real waist control, `--motion` bypasses debug mode and publishes upper-body commands on `rt/arm_sdk`. The shared SDK blend weight ramps from 0 to 1 after `[r]` and back to 0 during exit.

---

## 2. Testing in simulation

### 2.1 Prerequisites

1. Install `xr_teleoperate` and the `tv` conda env (see README §1)
2. Install [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) and the `unitree_sim_env` env
3. televuer SSL certs (for XR / WebRTC)

### 2.2 Basic run (Dex3)

**Terminal 1 — start Isaac sim**

```bash
conda activate unitree_sim_env
cd ~/unitree_sim_isaaclab
python sim_main.py --device cpu --enable_cameras \
    --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
    --enable_dex3_dds --robot_type g129
```

Click once inside the sim window → wait for something like `controller started, start main loop...`

**Terminal 2 — start teleop**

```bash
conda activate tv
cd ~/xr_teleoperate/teleop/
python teleop_hand_and_arm.py --ee=dex3 --sim --record
```

Defaults in effect: `--arm G1_29`, `--input-mode hand`, `--display-mode immersive`

### 2.3 Example: Sim + Inspire + upper body

**Sim**

```bash
python sim_main.py \
  --device cuda:0 \
  --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Inspire-Wholebody \
  --robot_type g129 \
  --enable_inspire_dds \
  --xr_upperbody
```

**Teleop**

```bash
cd ~/xr_teleoperate/teleop/
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --ee inspire_dfx \
  --input-mode hand \
  --body-tracking upper \
  --sim \
  --display-mode ego \
  --image-transport webrtc
```

Keep matching flags on both sides:

| Isaac (`sim_main.py`) | Teleop |
|---|---|
| `--enable_dex3_dds` | `--ee dex3` |
| `--enable_inspire_dds` | `--ee inspire_dfx` (or `inspire_ftp`) |
| `--xr_upperbody` | `--body-tracking upper --sim` |
| `--robot_type g129` | `--arm G1_29` |

### 2.4 Headset interaction order

1. Put on the headset and join the same Wi-Fi as the Host
2. Open the Vuer URL on the Host, e.g. `https://<HOST_IP>:8012/?ws=wss://<HOST_IP>:8012`
3. Click **Virtual Reality** and allow all prompts
4. **Align your arms to the robot’s initial pose** before starting (avoids sudden motion)
5. In the teleop terminal, press:
   - **`r`** — start / stop teleoperation
   - **`s`** — start / stop recording an episode (repeatable)
   - **`q`** — quit

Recorded data lands in `teleop/utils/data/` (or the path from `--task-dir`).

In `--sim` mode, after an episode is saved the system publishes a scene reset on `rt/reset_pose/cmd` for the next episode.

### 2.5 Checklist: “test in sim”

- [ ] Sim is running and the window has been clicked/activated
- [ ] Teleop includes `--sim`
- [ ] `--ee` matches the DDS enabled in sim (`dex3` ↔ `--enable_dex3_dds`, etc.)
- [ ] If using torso / waist → both `--xr_upperbody` (sim) and `--body-tracking upper --sim` (teleop)
- [ ] Headset Wi-Fi / Vuer connected / video visible
- [ ] After pressing `r`, the sim robot follows your hands

---

## 3. Sim → Real

Idea: **same teleop command, drop `--sim`**, then prepare hardware services.

### 3.1 Command comparison

**Sim**

```bash
python teleop_hand_and_arm.py --ee=dex3 --sim --record
```

**Real**

```bash
python teleop_hand_and_arm.py --ee=dex3 --record \
  --img-server-ip <PC2_IP>
```

What changes automatically without `--sim`:

- DDS domain → `0`
- Enter debug mode on the physical robot
- Enable arm / gripper safety clip and filters
- No `rt/sim_state` subscribe / no reset-pose publish

### 3.2 Physical prerequisites (before launch)

1. **Image server on PC2**

```bash
# On PC2 — after certs and cam_config_server.yaml are set up
python -m teleimager.image_server
```

Point `--img-server-ip` at the PC2 IP.

2. **Hand service** (depends on `--ee`)

| `--ee` | Service to run |
|---|---|
| `inspire_dfx` / `inspire_ftp` | [DFX_inspire_service](https://github.com/unitreerobotics/DFX_inspire_service) |
| `brainco` | [brainco_hand_service](https://github.com/unitreerobotics/brainco_hand_service) |
| `dex1` | [dex1_1_service](https://github.com/unitreerobotics/dex1_1_service) |
| `dex3` | follow Unitree / Dex3 service docs |

3. **Safety**

- Keep people clear of the robot
- Read the [official teleop docs](https://support.unitree.com/home/zh/Teleoperation) first
- Align arms near the start pose before pressing `r` and before pressing `q`
- If using `--motion`, the robot must be in control mode via the [R3 remote](https://www.unitree.com/R3)

### 3.3 Common real example

```bash
python teleop_hand_and_arm.py \
  --img-server-ip 10.61.6.62 \
  --arm G1_29 \
  --ee inspire_dfx \
  --input-mode hand \
  --display-mode ego \
  --image-transport webrtc
```

Before enabling real waist DDS, run the log-only path:

```bash
python teleop_hand_and_arm.py \
  --img-server-ip 10.61.6.62 \
  --arm G1_29 \
  --input-mode hand \
  --body-tracking upper \
  --dry-run-waist \
  --display-mode ego \
  --image-transport webrtc  
```

The dry-run path does not initialize DDS and constructs no robot or hand controller.

If the WebRTC feed is black / stuck, try:

```bash
--image-transport zmq
```

### 3.4 Checklist before going real

- [ ] Same flow already works in sim (tracking, EE, record)
- [ ] `--sim` removed from the command
- [ ] `teleimager` running on PC2 and `--img-server-ip` is correct
- [ ] Hand service matches `--ee` and open/close test succeeded
- [ ] DDS on domain 0 (real default)
- [ ] Clear workspace / e-stop ready
- [ ] Arms aligned to the robot before pressing `r`
- [ ] Waist dry-run completed with correct signs and conservative limits
- [ ] First waist DDS test uses a stand/gantry, never a freely standing robot
- [ ] Second person spotting from outside the waist/arm swing radius

### 3.5 Manual first-waist test checklist (human operator only)

This software has not been validated on physical hardware. The first test must remain mechanically secured throughout; completing it does not authorize a freely standing test.

Preparation:

- [ ] Robot is hung or secured on a load-rated stand/gantry, with feet unable to support free standing
- [ ] E-stop is functional and continuously within the primary operator's reach
- [ ] A second person is spotting from outside the complete waist and arm swing radius
- [ ] Workspace is clear; cables cannot snag as the torso rotates
- [ ] Confirm the configured limits are still yaw ±25°, roll ±12°, pitch ±12°
- [ ] Confirm `--tracking-fallback home` is present for the first test

Dry-run, before any waist DDS send:

- [ ] Run the `--dry-run-waist` command above and confirm the `no robot or hand command` banner
- [ ] Stand upright and press `[r]`; confirm calibration produces a near-zero target
- [ ] Move one torso axis at a time and verify yaw/roll/pitch signs against the intended robot direction
- [ ] Confirm logged values never exceed ±0.4364 rad yaw or ±0.2094 rad roll/pitch
- [ ] Stop or cover body tracking; within the configured timeout, confirm a loss warning and targets ramping toward zero
- [ ] Press `[q]` and confirm clean dry-run exit

First DDS-enabled test, still secured:

- [ ] Put the robot in the required Regular/control mode and keep locomotion at zero
- [ ] Start with `--body-tracking upper --motion --allow-real-waist --tracking-fallback home`
- [ ] Confirm the startup log says waist topic `rt/arm_sdk` and SDK weight remains zero before `[r]`
- [ ] Operator holds the calibrated upright pose; spotter and e-stop operator signal ready
- [ ] Press `[r]` and observe the two-second weight handover before making any torso movement
- [ ] Test a very small yaw input first, then small roll and pitch inputs separately
- [ ] Stop immediately for wrong sign, oscillation, balance-controller reaction, unexpected leg motion, or abrupt motion
- [ ] Intentionally interrupt body tracking and confirm the waist ramps toward calibrated zero
- [ ] Press `[q]` and confirm the log reports SDK weight released to `0.0` before process exit
- [ ] Do not widen limits or proceed to free standing without a separate review and explicit test plan

---

## 4. Common command recipes

### Fastest sim test (Dex3 + record)

```bash
# Terminal 1
python sim_main.py --device cpu --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
  --enable_dex3_dds --robot_type g129

# Terminal 2
python teleop_hand_and_arm.py --ee=dex3 --sim --record
```

### Sim test — upper body + Inspire

```bash
# Terminal 1
python sim_main.py --device cuda:0 --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Inspire-Wholebody \
  --robot_type g129 --enable_inspire_dds --xr_upperbody

# Terminal 2
python teleop_hand_and_arm.py \
  --arm G1_29 --ee inspire_dfx --input-mode hand \
  --body-tracking upper --sim --record
```

### Same recipe on real

```bash
# First DDS-enabled waist test: robot mechanically secured on a stand/gantry.
python teleop_hand_and_arm.py \
  --arm G1_29 --ee inspire_dfx --input-mode hand \
  --body-tracking upper --motion --allow-real-waist \
  --tracking-fallback home \
  --img-server-ip <PC2_IP> --record
```

### Debug joints published to DDS

```bash
python teleop_hand_and_arm.py --ee=dex3 --sim --print-joints
```

---

## 5. Common troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| real body tracking requires explicit `--allow-real-waist` | real `upper` command lacks opt-in | dry-run first, then add `--motion --allow-real-waist` only for the secured test |
| real waist control requires `--motion` | real waist would otherwise use `rt/lowcmd`/debug mode | add `--motion`; do not bypass this guard |
| Sim robot does not move | sim window not activated / no `r` / `--ee` ≠ DDS | activate sim → match `--enable_*_dds` ↔ `--ee` → press `r` |
| Teleop conflicts with a physical robot while sim is open | mixed DDS domains | use `--sim` (domain 1); don’t run real teleop at the same time unless intentional |
| Black headset video (WebRTC) | negotiation / certs | try `--image-transport zmq` and check certs |
| Physical robot jerks at start | arms not aligned to start pose | align before pressing `r` |
| Real hangs waiting for state | hand/arm DDS not up | check PC2 services and cabling |

---

## 6. Related files in this repo

| File | Role |
|---|---|
| `teleop/teleop_hand_and_arm.py` | argparse + `--sim` / domain / body-tracking switch |
| `teleop/utils/sim_state_topic.py` | subscribe `rt/sim_state` for sim recording |
| `teleop/robot_control/robot_arm.py` | arm behavior under `simulation_mode` |
| `teleop/robot_control/robot_hand_unitree.py` | Dex3 / Dex1 sim vs real |
| `teleop/robot_control/robot_hand_inspire.py` | Inspire (still waits on state DDS) |
| `README.md` §2 / §3 | sim quick start and physical deployment |

---

## 7. See also

- Main README: [README.md](../README.md) (§2 Simulation, §3 Physical)
- Isaac Lab companion: [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab)
- Convert recorded data for IL: [unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot)
