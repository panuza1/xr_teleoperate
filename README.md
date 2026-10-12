# XR Teleoperate

> Full original documentation: [README_old.md](README_old.md)

Quest hand or controller tracking controls the Unitree G1 arms. Locomotion input is selectable with `--locomotion-input` (`quest` by default):

- Left stick vertical: forward/backward
- Left stick horizontal: strafe left/right
- Right stick horizontal: yaw left/right
- Right stick vertical: ignored

- `quest`: Quest thumbsticks command high-level locomotion.
- `unitree`: the native Unitree wireless controller remains responsible for locomotion; xr_teleoperate only monitors `rt/lowstate`.
- `none`: xr_teleoperate sends no locomotion commands.

The Unitree wireless remote monitor is read-only. This project does not directly command low-level leg joints in motion mode.

## Safety

Before using a physical G1:

- Put the robot in **Regular Mode**, not Running Mode.
- Use a stand or gantry for the first test.
- Keep the physical e-stop ready and use a second person as a spotter.
- Test Quest input with the no-DDS inspector before enabling walking.
- The program does not home arms automatically on shutdown. It requests an acknowledged zero-velocity Loco command, holds the latest fresh arm joint positions briefly, then stops its arm publisher. If zero velocity cannot be acknowledged, use the physical e-stop.
- `LocoClient.Damp()` is a locomotion mode request, not an emergency stop. `--return-arms-home-on-exit` is an explicit opt-in for an orderly `q` or controller exit only; it is skipped after Ctrl+C, errors, or an unconfirmed locomotion stop.

## Installation

Tested with Ubuntu 20.04/22.04 and Python 3.10.

### 1. Clone the repository

```bash
source ~/miniconda3/etc/profile.d/conda.sh

cd ~/Documents/fibo/project_humanoid
git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd xr_teleoperate
git submodule update --init --depth 1
```

Skip the clone command if the repository already exists.

### 2. Install the teleoperation environment

```bash
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv

cd ~/Documents/fibo/project_humanoid/xr_teleoperate
pip install -r requirements.txt
pip install -e teleop/teleimager --no-deps
pip install -e teleop/televuer
```

Install Unitree SDK2:

```bash
cd ~/Documents/fibo/project_humanoid
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

Use Unitree SDK2 commit `404fe44` or newer.

### 3. Install the image-server environment

```bash
cd ~
git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd ~/xr_teleoperate
git submodule update --init --depth 1

conda create -n teleimager python=3.10
conda activate teleimager

cd ~/xr_teleoperate/teleop/teleimager
pip install -e ".[server]"
```

Skip the clone command if `~/xr_teleoperate` already exists on the image-server computer.

### 4. Create the Quest HTTPS certificate

```bash
conda activate tv
cd ~/Documents/fibo/project_humanoid/xr_teleoperate/teleop/televuer

openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout key.pem -out cert.pem

mkdir -p ~/.config/xr_teleoperate
cp cert.pem key.pem ~/.config/xr_teleoperate/
sudo ufw allow 8012
```

## Simulation

### G1_23 arms in MuJoCo

This runs the checked-in `scene_23dof.xml` with a fixed base and the existing G1_23 arm IK/controller. Arm commands and state use `rt/lowcmd` and `rt/lowstate` on DDS domain 1 and interface `lo`; teleop is explicitly in `--sim` mode. Quest passthrough uses the headset cameras, so no image server is needed.

First, in a separate terminal, check the headset event streams without DDS or robot commands:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate tv
cd ~/Documents/fibo/project_humanoid/g1_inspire_workspace/xr_teleoperate
python teleop/inspect_hybrid_input.py --input-mode body-controller --locomotion-input quest
```

Enable Quest Browser's WebXR Experiments flag, then open `https://<HOST_IP>:8012` and enter VR. Hold both Touch Controllers, move both arms, and operate both sticks. `BODY TRACKING`, `LEFT CONTROLLER`, and `RIGHT CONTROLLER` must remain active while the six shoulder/elbow/wrist values and joystick values update. This check requires the headset.

Then use these two terminals:

Terminal 1 — G1_23 MuJoCo simulation:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate unitree
cd ~/Documents/fibo/project_humanoid/unitree_mujoco/simulate_python
python g1_23_arm_sim.py
```

Terminal 2 — Quest body wrists to both arms:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate tv
cd ~/Documents/fibo/project_humanoid/g1_inspire_workspace/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --arm G1_23 \
  --input-mode body-controller \
  --display-mode pass-through \
  --locomotion-input none \
  --sim
```

Press `r` to start arm tracking and `q` to exit. `BODY_MOVE` wrist poses drive the existing G1_23 IK; controller poses are ignored and controller sticks remain available. Stale body data holds the last arm target, while stale controller data produces zero velocity. The simulation is arms-only and fixed-base because this local MuJoCo repository has no compatible G1_23 walking policy. Do not add `--motion`; MuJoCo does not implement the physical G1 `LocoClient`.

### Existing Isaac Lab simulation

Install [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) and its `unitree_sim_env` environment by following that repository’s installation instructions.

### Terminal 1 — Start the simulator

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate unitree_sim_env
cd ~/unitree_sim_isaaclab

python sim_main.py \
  --device cpu \
  --enable_cameras \
  --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
  --enable_dex3_dds \
  --robot_type g129
```

Click once inside the simulator window after it opens.

### Terminal 2 — Start XR teleoperation

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate tv
cd ~/Documents/fibo/project_humanoid/g1_inspire_workspace/xr_teleoperate/teleop

python teleop_hand_and_arm.py \
  --arm G1_29 \
  --ee dex3 \
  --input-mode hand \
  --sim
```

The existing simulator does not implement the G1 `LocoClient` sport API, so this simulation command does not verify controller walking.

## Physical G1

Use two terminals. Keep Terminal 1 running while teleoperation runs in Terminal 2.

### Terminal 1 — Start the RealSense image server

```bash
source ~/miniconda3/etc/profile.d/conda.sh

conda activate teleimager
cd ~/xr_teleoperate/teleop/teleimager
sudo systemctl stop realsense_gst.service
python -m teleimager.image_server
```

### Terminal 2 — Arms only

This mode uses Quest hand tracking for the arms and does not enable controller walking.

```bash
source ~/miniconda3/etc/profile.d/conda.sh

conda activate tv
cd ~/Documents/fibo/project_humanoid/g1_inspire_workspace/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode hand \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

### Terminal 2 — G1_23 controller-tracked arms + controller walking

This is the official Unitree controller flow: Touch Plus 6DoF poses drive the existing arm IK while the same controllers provide locomotion input. Do not pass `--ee` for a standard G1_23 without dexterous hands.

```bash
source ~/miniconda3/etc/profile.d/conda.sh

conda activate tv
cd ~/Documents/fibo/project_humanoid/g1_inspire_workspace/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --arm G1_23 \
  --input-mode controller \
  --motion \
  --locomotion-input quest \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

### Terminal 2 — Hand-tracked arms + Unitree controller walking

Quest hand tracking still drives the arms and Inspire DFX hands. Quest controller locomotion is disabled, and the native Unitree motion system owns the legs:

```bash
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode hand \
  --motion \
  --locomotion-input unitree \
  --ee inspire_dfx \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

```text
Quest 3 hand tracking -> IK -> rt/arm_sdk -> G1 arms
Quest 3 hand tracking -> Inspire DFX hands

Unitree wireless controller -> native G1 motion system -> G1 legs
```

Current maximum command scales:

- Translation: `0.30`
- Yaw: `0.30`
- Controller timeout: `0.5 s`
- Loco RPC timeout: `0.1 s` by default; configurable from `0.01` to `0.5` seconds with `--locomotion-rpc-timeout`. RPC work runs on a single bounded worker so the IK/control loop does not wait for acknowledgements.

The left controller pose drives the left wrist target and the right controller pose drives the right wrist target. Right **A** exits teleoperation. Pressing both thumbsticks requests damping mode. If either controller pose becomes stale, the arms hold their last IK target; stale joystick axes command zero velocity.

## Quest connection

1. Put the Quest and host computer on the same network.
2. Open `https://<HOST_IP>:8012` in the Quest browser.
3. Accept the certificate warning and select **Virtual Reality**.
4. Allow hand/controller permissions.
5. Align both controllers with the robot’s initial arm pose.
6. Press **r** in the teleoperation terminal to start.
7. Press **q** to quit.

## Safe Quest-only input test

This command starts no DDS client and sends nothing to the G1:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate tv
cd ~/Documents/fibo/project_humanoid/g1_inspire_workspace/xr_teleoperate

python teleop/inspect_hybrid_input.py --input-mode controller --frequency 10
```

Move each controller separately and verify that only its corresponding `LEFT WRIST XYZ` or `RIGHT WRIST XYZ` value changes. Verify that both controllers and `ARM TRACKING` become active, the joysticks continue updating while the controllers move, centered sticks produce `Move(0, 0, 0)`, and stale input returns the affected axes to zero before running the physical motion-mode command.

To inspect the Unitree-owned or no-locomotion paths without DDS or robot commands:

```bash
python teleop/inspect_hybrid_input.py --locomotion-input unitree --duration 10
python teleop/inspect_hybrid_input.py --locomotion-input none --duration 10
```

## XR Teleop Web UI

From the repository root, start the local UI and open
`http://127.0.0.1:8080`:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate tv
python web_ui.py
```

The `light | b&w | dark` selector and last configuration are stored in browser
localStorage. The generated command is the exact argument list used by
`start`; `stop` sends the managed teleop process through its existing cleanup
path. The resizable xterm.js diagnostics terminal receives the process PTY over
a WebSocket, preserves ANSI/Unicode output, shows its exit status, and forwards
keys only while that terminal has focus. This lets the existing CLI handle
runtime keys such as `r` and `q` unchanged. Terminal output can be copied or
cleared without affecting the process.

The Basic, Advanced, and All Parameters views are generated from the real
`argparse` declarations in `teleop_hand_and_arm.py`. CLI choices, defaults, and
help text therefore stay aligned with the launcher. Every field remains
editable after applying a preset; modified values have field-level reset and
validation controls, and uncommon future flags can be appended through Extra
CLI Arguments.

## Real G1 Controller Teleoperation — Phase 5

**Physical motion was not tested or authorized by this preparation.** Use the
`tv` environment. The arm path is controller-only and does not require an
end-effector argument. Do not start motor publishing until the network, live
lowstate, physical robot variant, operating mode, and emergency-stop checks
below have been confirmed by the operator.

### Wired network and read-only state

The wired adapter observed for this host is `enp43s0`; it reported carrier but
NetworkManager showed it disconnected and it had no active IPv4 address. A
saved inactive profile named `unitree_g1` specifies host address
`192.168.123.99/24` with no gateway, but it is not bound to a particular
interface and an address-conflict check was unavailable (`arping` is not
installed). Treat it as a candidate existing host assignment until the
operator verifies the exact robot network. The host was on Wi-Fi
`172.20.10.4/28`, and `192.168.123.164` routed through Wi-Fi. This repo uses
`.164` as an image-server default, so it is not a verified DDS endpoint or
physical variant identifier. Do not route DDS through Wi-Fi.

After the robot subnet is confirmed and a duplicate-address probe reports no
conflict, the existing wired profile can be activated and reversed without
editing its IP settings:

```bash
arping -D -I enp43s0 -c 3 192.168.123.99
nmcli connection up unitree_g1 ifname enp43s0
ip -br addr
ip route
ip route get <verified-robot-ip>
# To restore the prior disconnected state:
nmcli connection down unitree_g1
```

These activation and rollback commands were not run. Confirm the Wi-Fi default
route remains unchanged after activation before any DDS inspection.

Read-only host checks:

```bash
ip -br addr
ip route
ip route get <verified-robot-ip>
ip link show enp43s0
nmcli device show enp43s0
cat /sys/class/net/enp43s0/carrier
nmcli connection show unitree_g1
```

After the host address and dedicated wired link are confirmed, run the bounded
SDK2 subscriber (domain 0, `rt/lowstate` only). It reports receive rate/times,
`mode_machine`, all motor q/dq finiteness, arm q/dq, and raw motor status words.
Exit status is nonzero on no data, stale data, inconsistent mode, or invalid
state. It never creates a publisher or calls a motion service:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate tv
python teleop/robot_control/g1_23_readonly_state.py --interface enp43s0 --duration 10
```

Do not treat a 35-slot message as proof of 35 physical joints or proof of the
G1_23 variant. Confirm the exact variant from the robot’s authoritative
configuration and reconcile observed state with the official SDK G1 index
map. Expected G1_23 arm slots are left `15–19`, right `22–26`; confirm the
observed `mode_machine` is supported before proceeding.

### Offline Quest-to-IK preview

This uses synthetic controller poses, the existing Televuer transform and the
real G1_23 model/IK. It constructs no DDS participant or publisher and sends no
commands. The unreachable-left case must reject that side and hold its
synthetic nominal seed while keeping a valid right-side target:

```bash
python teleop/robot_control/g1_23_dry_run.py --scenario reachable
python teleop/robot_control/g1_23_dry_run.py --scenario unreachable-left
```

The default configurable Cartesian residual ceiling is 0.05 m: targets at the
boundary pass and targets above it are rejected in the offline tests. This is
a software starting value, not a measured G1 calibration tolerance; no robot
accuracy or workspace margin was established in this phase.

For Quest input alone, the existing inspector has no DDS or robot connection:

```bash
python teleop/inspect_hybrid_input.py --input-mode controller --locomotion-input none --duration 30
```

### Operator-controlled trial stages — not run by this preparation

Proceed one stage at a time. Stage A must pass on the actual wired G1 link.
Before any arm trial, confirm the verified G1_23 variant and mode, have an
operator beside the supported/restraint-ready robot, clear nearby people and
obstacles, verify an independently accessible physical E-stop, and ensure no
other arm controller is active.

**Stage A — read-only lowstate:** run the subscriber above and record fresh
message rate, timestamps, `mode_machine`, state finiteness, and the physical
variant evidence. Stop if any field is unknown or stale.

**Stage B — arm-only command, operator-controlled:** this starts the existing
`rt/arm_sdk` publisher after fresh lowstate validation. The controller seeds
its initial arm target from measured q and starts publishing that hold before
the operator presses `r`. Motion mode also sets arm SDK weight slot 29 to 1.0
on its first write. The exact effect of enabling this SDK path on this robot
has not been observed. Do not launch this command until Stage A and the
mode/variant checks pass and the operator is ready to take responsibility for
the arm command path:

```bash
python teleop/teleop_hand_and_arm.py --arm G1_23 --input-mode controller --motion --locomotion-input none --display-mode pass-through
```

Only after observing safe single-arm small movements, recovery, and shutdown
should the operator test the other arm and then both arms. Do not use
`--return-arms-home-on-exit` unless a home motion is separately intended and
supervised.

**Stage D — combined arm and walking, gated on all arm-only tests:**

```bash
python teleop/teleop_hand_and_arm.py --arm G1_23 --input-mode controller --motion --locomotion-input quest --display-mode pass-through
```

Quest tracking older than 0.5 seconds holds the last arm target; stale
joystick axes request zero velocity. Loss of `rt/lowstate` stops arm publishing
after the controller’s 0.5-second state timeout. On shutdown the program
requests a fresh measured-position hold, sends it briefly, and stops its
publisher. The robot’s behavior after publisher shutdown and its independent
watchdog response to lost DDS remain unverified. `Damp`, a zero-velocity
request, and keyboard quit are not emergency stops; keep the physical E-stop
accessible and follow the operator’s recovery procedure.
