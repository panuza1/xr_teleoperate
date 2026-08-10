<div align="center">
  <h1>xr_teleoperate</h1>
  <p>XR-based teleoperation for Unitree humanoid robots (Apple Vision Pro / PICO 4 Ultra Enterprise / Meta Quest 3)</p>
  <p>
    <a href="https://github.com/unitreerobotics/xr_teleoperate/wiki">📖 Wiki</a> ·
    <a href="https://discord.gg/ZwcVwxv5rq">💬 Discord</a> ·
    <a href="https://deepwiki.com/unitreerobotics/xr_teleoperate">🤖 DeepWiki</a> ·
    <a href="CHANGELOG.md">🔖 Changelog</a>
  </p>
</div>

> New here? Read up to "Application Development" in the [Unitree official docs](https://support.unitree.com/main/en) first, then come back.     

> 📄 This is a reorganized/quick-reference version of the original README. Full original: [README_old.md](README_old.md)

---

## 🚀 TL;DR — Simulation Quick Start

If you already know what you're doing and just want the robot moving in sim, this is the whole path:

```bash
# ── One-time setup ──────────────────────────────────────────
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv
git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd xr_teleoperate
git submodule update --init --depth 1

cd teleop/teleimager && pip install -e . --no-deps && cd ../..
cd teleop/televuer   && pip install -e .           && cd ../..
# (see §1.1 below for the one-time SSL cert setup televuer needs)

git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python && pip install -e . && cd ..

# ── Every time you run it ───────────────────────────────────
# Terminal 1: start the sim (needs unitree_sim_isaaclab installed separately)
conda activate unitree_sim_env
cd ~/unitree_sim_isaaclab
python sim_main.py --device cpu --enable_cameras \
    --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
    --enable_dex3_dds --robot_type g129
# → click inside the sim window once to activate it

# Terminal 2: start teleop
conda activate tv
cd ~/xr_teleoperate/teleop/
python teleop_hand_and_arm.py --ee=dex3 --sim --record
```

Then: put on the headset → connect to the same Wi-Fi → open the Vuer URL in a browser → click **Virtual Reality** → align your arms to the robot's start pose → press **r** to start teleop → press **s** to start/stop recording → press **q** to quit.

Full step-by-step with screenshots is in [§2.2 Launch (Simulation)](#22--launch).  
Sim ↔ real parameter guide (test in sim, then deploy): [docs/sim_to_real.md](docs/sim_to_real.md).

---

## 🎯 Real Example — Physical G1 + Inspire Hand, Upper Body Only

This is a full physical-deployment launch command (no `--sim`), tracking only the upper body, viewing through the pass-through window, with images sent over WebRTC:

```bash
python teleop_hand_and_arm.py \
  --img-server-ip 10.61.6.62 \
  --arm G1_29 \
  --ee inspire_dfx \
  --input-mode hand \
  --body-tracking upper \
  --motion \
  --allow-real-waist \
  --tracking-fallback home \
  --display-mode ego \
  --image-transport webrtc
```

**What each flag is doing here:**

| Flag | This run |
|---|---|
| `--img-server-ip 10.61.6.62` | PC2's IP on your network — where teleimager is serving the camera feed from. Must match the machine actually running `teleimager.image_server`. |
| `--arm G1_29` | Robot is a G1 with the 29-DoF arm. |
| `--ee inspire_dfx` | Inspire DFX dexterous hand as the end-effector — needs the [DFX_inspire_service](https://github.com/unitreerobotics/DFX_inspire_service) running on PC2 (§3.2). |
| `--input-mode hand` | Controlling with hand tracking (no physical controller needed). |
| `--body-tracking upper` | Tracks the torso and retargets it to the three G1 waist joints; it does not command a head joint. |
| `--motion` | Uses `rt/arm_sdk`, leaving the robot motion controller responsible for balance. Required for real waist control. |
| `--allow-real-waist` | Explicit safety opt-in required before computed waist targets can reach real hardware. |
| `--tracking-fallback home` | Slowly returns arms and waist toward home if their XR streams become stale. |
| `--display-mode ego` | Headset shows pass-through video plus a small first-person robot-camera window, instead of going fully immersive. |
| `--image-transport webrtc` | Camera stream comes in over WebRTC (lower latency, needs the SSL certs from §1.1 configured on both Host and PC2). |

**Before running this:**
1. Complete a `--dry-run-waist` session first and verify signs, calibration, limits, and timeout fallback without DDS.
2. For the first DDS test, mechanically secure the robot on a stand/gantry; never test while freely standing.
3. Keep the e-stop ready and have a second person spot from outside the waist/arm swing radius.
4. `teleimager.image_server` is running on PC2 (`10.61.6.62`), configured with the certs from §1.1.
5. If using `inspire_dfx`, the `DFX_inspire_service` is running on PC2 and the hand test (`hand_example`) opened/closed successfully at least once.
6. You're in the `tv` conda environment on the host.

Dry-run command (no DDS initialization and no robot/hand controller):

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

**Then, same interaction pattern as the simulation flow (§2.2):** put on headset → connect Wi-Fi → open the Vuer URL → click **Virtual Reality** → align arms to the robot's initial pose → press **r** to start teleop → **s** to record → **q** to quit.

> 💡 If you get a black/frozen camera feed with `--image-transport webrtc` (a known WebRTC negotiation issue in some `vuer` versions), switch to `--image-transport zmq` as a workaround.

---

## 🤖 Supported Hardware

| Robot / End-effector | Status |
|---|:---:|
| G1 (29 DoF) | ✅ |
| G1 (23 DoF) | ✅ |
| H1 (4-DoF arm) | ✅ |
| H1_2 (7-DoF arm) | ✅ |
| H2 (7-DoF arm) | ✅ |
| Dex1-1 gripper | ✅ |
| Dex3-1 dexterous hand | ✅ |
| Inspire dexterous hand | ✅ |
| BrainCo dexterous hand | ✅ |

Wiring / system diagram: [full-size image](https://oss-global-cdn.unitree.com/static/55fb9cd245854810889855010da296f7_3415x2465.png)

Tested on Ubuntu 20.04 / 22.04. Other OSes may need adjustments — see [OpenTeleVision](https://github.com/OpenTeleVision/TeleVision) for background.

---

## 1. 📦 Installation

### 1.1 Base setup + televuer SSL certs

```bash
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv
git clone https://github.com/unitreerobotics/xr_teleoperate.git
cd xr_teleoperate
git submodule update --init --depth 1

cd teleop/teleimager
pip install -e . --no-deps
```

<details>
<summary><b>televuer install + SSL certificate setup</b> (required once — XR devices need HTTPS/WebRTC)</summary>

```bash
cd teleop/televuer
pip install -e .

# Pico / Quest:
openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem

# Apple Vision Pro (extra CA steps):
openssl genrsa -out rootCA.key 2048
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 365 -out rootCA.pem -subj "/CN=xr-teleoperate"
openssl genrsa -out key.pem 2048
openssl req -new -key key.pem -out server.csr -subj "/CN=localhost"

# server_ext.cnf — IP.2 must be your host's IP (check with ifconfig)
cat > server_ext.cnf <<'EOF'
subjectAltName = @alt_names
[alt_names]
DNS.1 = localhost
IP.1 = 192.168.123.164
IP.2 = 192.168.123.2
EOF

openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key \
    -CAcreateserial -out cert.pem -days 365 -sha256 -extfile server_ext.cnf

# AirDrop rootCA.pem to Apple Vision Pro and install it there.

sudo ufw allow 8012

# Point the code at the certs (pick ONE):
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/
# — or —
echo 'export XR_TELEOP_CERT="$HOME/xr_teleoperate/teleop/televuer/cert.pem"' >> ~/.bashrc
echo 'export XR_TELEOP_KEY="$HOME/xr_teleoperate/teleop/televuer/key.pem"' >> ~/.bashrc
source ~/.bashrc
```
</details>

### 1.2 unitree_sdk2_python

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

> ⚠️ For `xr_teleoperate` **v1.1+**, `unitree_sdk2_python` must be at commit [`404fe44`](https://github.com/unitreerobotics/unitree_sdk2_python/commit/404fe44d76f705c002c97e773276f2a8fefb57e4) or newer.
> The old `unitree_dds_wrapper` (h1_2 branch) has been fully replaced by this package.

<details>
<summary>What does <code>(tv) unitree@Host:~$</code> mean in these commands?</summary>

- `(tv)` — you're in the conda env named `tv`
- `unitree@Host:~` — user `unitree`, machine `Host`, cwd = `$HOME`
- everything after `$` is the actual command to run

More: [Conda User Guide](https://docs.conda.io/projects/conda/en/latest/user-guide/getting-started.html) · [Unix guide](https://www.harley.com/unix-book/book/chapters/04.html#H)
</details>

### 1.3 🔘 Launch Parameters (reference)

**Core:**

| Parameter | What it does | Options | Default |
|---|---|---|---|
| `--frequency` | FPS for recording/control | any float | `30.0` |
| `--input-mode` | XR input source | `hand`, `controller` | `hand` |
| `--display-mode` | XR view mode | `immersive`, `ego`, `pass-through` | `immersive` |
| `--arm` | Robot arm type | `G1_29`, `G1_23`, `H1_2`, `H1` | `G1_29` |
| `--ee` | End-effector type | `dex1`, `dex3`, `inspire_ftp`, `inspire_dfx`, `brainco` | none |
| `--img-server-ip` | Image server IP (WebRTC signaling) | IPv4 | `192.168.123.164` |
| `--network-interface` | CycloneDDS network interface | interface name | none |
| `--body-tracking` | Waist body-tracking mode | `off`, `upper` | `off` |
| `--tracking-timeout` | Seconds before XR tracking is stale | positive float | `0.5` |
| `--tracking-fallback` | Stale-tracking behavior for arms and waist | `hold`, `home` | `home` |
| `--image-transport` | How the camera stream is delivered | `auto`, `webrtc`, `zmq` | `auto` |

**Mode switches:**

| Flag | Effect |
|---|---|
| `--motion` | Run alongside the robot's motion controller. Hand mode → R3 controller drives walking. Controller mode → joysticks also drive walking. Only "Regular mode" (R1+X), not "Running mode". |
| `--headless` | For headless machines (e.g. PC2) with no display |
| `--sim` | [Simulation mode](https://github.com/unitreerobotics/unitree_sim_isaaclab) |
| `--allow-real-waist` | Explicit opt-in for real G1 waist commands; requires `--body-tracking upper --motion` |
| `--dry-run-waist` | Retarget and log waist targets without initializing DDS or robot/hand controllers |
| `--ipc` | Control the program's state via IPC (for agent integration) |
| `--affinity` | Pin CPU cores — leave alone unless you know why you need it |
| `--record` | Press **r** to start teleop, **s** to start/stop recording (repeatable) |
| `--task-*` | Save path / target / description / steps for recorded tasks |

State machine diagram: [full-size image](https://oss-global-cdn.unitree.com/static/712c312b0ac3401f8d7d9001b1e14645_11655x4305.jpg)

---

## 2. 💻 Simulation Deployment

### 2.1 Setup
Install [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) (separate repo, follow its README), then:

```bash
conda activate unitree_sim_env
cd ~/unitree_sim_isaaclab
python sim_main.py --device cpu --enable_cameras \
    --task Isaac-PickPlace-Cylinder-G129-Dex3-Joint \
    --enable_dex3_dds --robot_type g129
```

⚠️ **Click once inside the sim window to activate it.** Terminal should show `controller started, start main loop...`

<details>
<summary><b>Example: G1 + Inspire hand, whole-body task, GPU-accelerated</b></summary>

```bash
# Terminal 1: sim, running on GPU with an Inspire whole-body task
python sim_main.py \
  --device cuda:0 \
  --enable_cameras \
  --task Isaac-Move-Cylinder-G129-Inspire-Wholebody \
  --robot_type g129 \
  --enable_inspire_dds \
  --xr_upperbody
```

| Flag | This run |
|---|---|
| `--device cuda:0` | Runs the sim on GPU 0 instead of CPU — much faster, but check VRAM headroom. |
| `--task Isaac-Move-Cylinder-G129-Inspire-Wholebody` | The Inspire-hand, whole-body variant of the move/cylinder task. |
| `--enable_inspire_dds` | Turns on the DDS topic for the Inspire hand (equivalent role to `--enable_dex3_dds` for Dex3). |
| `--xr_upperbody` | Tells the sim to only expect/drive upper-body XR data — pair this with `--body-tracking upper` on the teleop side below. |

```bash
# Terminal 2: matching teleop launch (upper-body only, Inspire DFX hand)
cd ~/xr_teleoperate/teleop/
python teleop_hand_and_arm.py \
  --img-server-ip 10.61.6.62 \
  --arm G1_29 \
  --ee inspire_dfx \
  --input-mode hand \
  --body-tracking upper \
  --sim \
  --display-mode ego \
  --image-transport webrtc
```

⚠️ The `--xr_upperbody` on the sim side and `--body-tracking upper` on the teleop side need to match — otherwise the sim is set up to expect a different set of tracked joints than what teleop is actually sending.
</details>

### 2.2 🚀 Launch

```bash
cd ~/xr_teleoperate/teleop/
python teleop_hand_and_arm.py --xr-mode=hand --arm=G1_29 --ee=dex3 --sim --record
# same thing, relying on defaults:
python teleop_hand_and_arm.py --ee=dex3 --sim --record
```

**Then, in order:**

1. Put on the XR headset and connect it to the matching Wi-Fi.
2. *(Only if head camera WebRTC is enabled in `cam_config_server.yaml`)* Open `https://192.168.123.164:60001` in a browser, accept the security warning, press **start**, confirm you see the head-camera preview. *(This is PC2's IP — the machine running teleimager. Only needed once per device/cert.)*
3. Open `https://192.168.123.2:8012/?ws=wss://192.168.123.2:8012` (replace with your **Host** IP). Accept the security warning.
   - PICO fallback if the websocket doesn't connect: `https://vuer.ai?ws=wss://192.168.123.2:8012`
4. In the Vuer page, click **Virtual Reality** and allow all prompts.
5. You should see the robot's first-person view. Terminal shows `websocket is connected...` / `Uplink task running...`.
6. **Align your arm to the robot's initial pose** before starting, to avoid sudden movement.
7. Press **r** in the terminal to begin teleoperation.
8. Press **s** to start recording, **s** again to stop/save. Repeatable.
9. Press **q** to quit.

Recorded episodes save to `xr_teleoperate/teleop/utils/data` — see [unitree_IL_lerobot](https://github.com/unitreerobotics/unitree_IL_lerobot/tree/main?tab=readme-ov-file#data-collection-and-conversion) for how to use them. Watch your disk space.

---

## 3. 🤖 Physical Deployment

Same flow as simulation, plus:

### 3.1 Image service (PC2)

```bash
# On PC2:
git clone https://github.com/silencht/teleimager
# configure per teleimager's README

# On Host: copy the certs you generated in §1.1 to PC2
scp ~/xr_teleoperate/teleop/televuer/key.pem \
    ~/xr_teleoperate/teleop/televuer/cert.pem \
    unitree@192.168.123.164:~/teleimager

# On PC2:
mkdir -p ~/.config/xr_teleoperate/
cp cert.pem key.pem ~/.config/xr_teleoperate/

# Configure cam_config_server.yaml, then start the service:
python -m teleimager.image_server
# (equivalently: teleimager-server)
```

```bash
# On Host: subscribe to images
cd ~/xr_teleoperate/teleop/teleimager/src
python -m teleimager.image_client --host 192.168.123.164
# or test via browser: https://192.168.123.164:60001 → click Start
```

### 3.2 Dexterous hand services (pick what applies)

| Hand | Repo | Notes |
|---|---|---|
| Inspire | [DFX_inspire_service](https://github.com/unitreerobotics/DFX_inspire_service) | See [issue #46](https://github.com/unitreerobotics/xr_teleoperate/issues/46) (DFX) / [issue #48](https://github.com/unitreerobotics/xr_teleoperate/issues/48) (FTP) |
| BrainCo | [brainco_hand_service](https://github.com/unitreerobotics/brainco_hand_service) | — |
| Dex1-1 | [dex1_1_service](https://github.com/unitreerobotics/dex1_1_service) | — |

<details>
<summary>Inspire hand build steps (on PC2)</summary>

```bash
sudo apt install libboost-all-dev libspdlog-dev
cd DFX_inspire_service && mkdir build && cd build
cmake ..
make -j6

# Terminal 1 (choose one):
sudo ./inspire_g1
sudo ./inspire_h1 -s /dev/ttyUSB0

# Terminal 2: run example
./hand_example
```
Success = both hands open/close continuously. Then close `hand_example`.
</details>

### 3.3 🚀 Launch

> ⚠️ **Safety first**
> - Keep everyone at a safe distance from the robot.
> - Read the [official teleop docs](https://support.unitree.com/home/zh/Teleoperation) at least once before running this.
> - `--motion` requires the robot in control mode via [R3 remote](https://www.unitree.com/R3).
> - In motion mode: right **A** = exit · both joysticks pressed = soft e-stop (damping mode) · left stick = drive · right stick = turn · max speed is capped in code.

Launch identically to simulation (§2.2), minus `--sim`.

### 3.4 🔚 Exit

> ⚠️ Move the robot's arms near the initial pose before pressing **q**.
> - **Debug mode:** arms return to initial pose over 5s, then control ends.
> - **Motion mode:** arms return to motion-control pose over 5s, then control ends.

---

## 4. 🗺️ Codebase Map

```
xr_teleoperate/
├── assets/                        Robot URDF files
└── teleop/
    ├── teleimager/                 Image service library
    ├── televuer/
    │   └── src/televuer/
    │       ├── television.py       Captures head/wrist/hand/controller data via Vuer
    │       └── tv_wrapper.py        Post-processes captured data
    ├── robot_control/
    │   ├── src/dex-retargeting/     Dexterous-hand retargeting algorithms
    │   ├── robot_arm_ik.py          Arm inverse kinematics
    │   ├── robot_arm.py             Dual-arm joint control / part locking
    │   ├── hand_retargeting.py      Wrapper around dex-retargeting
    │   ├── robot_hand_inspire.py    Inspire hand control
    │   └── robot_hand_unitree.py    Unitree hand control
    ├── utils/
    │   ├── episode_writer.py        Records data for imitation learning
    │   ├── weighted_moving_filter.py Joint-data filter
    │   ├── rerun_visualizer.py      Visualizes recorded data
    │   ├── ipc.py                   IPC with proxy programs
    │   ├── motion_switcher.py       Motion-control state switching
    │   └── sim_state_topic.py       Simulation deployment support
    └── teleop_hand_and_arm.py      Main entry point
```

---

## 5. 🧰 Troubleshooting quick-refs

| Symptom | Likely cause |
|---|---|
| Websocket won't connect from PICO | Try `https://vuer.ai?ws=wss://<host-ip>:8012` instead |
| Certificate warnings on every launch | Expected once per device — click through "Advanced → Proceed" |
| Robot jerks on start | Arm wasn't aligned to initial pose before pressing **r** |
| Simulation window unresponsive | Click inside the sim window once to activate it |

For deeper debugging, check the [Wiki](https://github.com/unitreerobotics/xr_teleoperate/wiki) or [Discord](https://discord.gg/ZwcVwxv5rq).

---

## 🙏 Built on top of

[TeleVision](https://github.com/OpenTeleVision/TeleVision) · [dex-retargeting](https://github.com/dexsuite/dex-retargeting) · [vuer](https://github.com/vuer-ai/vuer) · [pinocchio](https://github.com/stack-of-tasks/pinocchio) · [casadi](https://github.com/casadi/casadi) · [meshcat-python](https://github.com/meshcat-dev/meshcat-python) · [pyzmq](https://github.com/zeromq/pyzmq) · [BunnyVisionPro](https://github.com/Dingry/BunnyVisionPro) · [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) · [beavr-bot](https://github.com/ARCLab-MIT/beavr-bot)

## 📝 Citation

```bibtex
@misc{xr-teleoperate,
  author       = {{Unitree Robotics}},
  title        = {{XR-Teleoperate}: An Open-Source Teleoperation Framework and Data Collection Toolkit for Embodied Intelligence},
  howpublished = {\url{https://github.com/unitreerobotics/xr_teleoperate}},
  year         = {2024},
  note         = {Accessed: 2026-02}
}
```
