# XR Teleoperate

> Full original documentation: [README_old.md](README_old.md)

Quest hand tracking controls the Unitree G1 arms. In physical motion mode, Quest controllers command the Unitree high-level locomotion controller:

- Right stick vertical: forward/backward
- Right stick horizontal: strafe left/right
- Left stick horizontal: yaw left/right
- Left stick vertical: ignored

The Unitree controller remains responsible for the legs and balance. This project does not directly command low-level leg joints in motion mode.

## Safety

Before using a physical G1:

- Put the robot in **Regular Mode**, not Running Mode.
- Use a stand or gantry for the first test.
- Keep the physical e-stop ready and use a second person as a spotter.
- Test Quest input with the no-DDS inspector before enabling walking.

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
cd ~/Documents/fibo/project_humanoid/xr_teleoperate/teleop

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
cd ~/Documents/fibo/project_humanoid/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode hand \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

### Terminal 2 — Hand-tracked arms + controller walking

This mode keeps hand tracking on the arms and enables high-level Unitree locomotion:

```bash
source ~/miniconda3/etc/profile.d/conda.sh

conda activate tv
cd ~/Documents/fibo/project_humanoid/xr_teleoperate/teleop
python teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode hand \
  --motion \
  --img-server-ip 192.168.123.164 \
  --image-transport zmq
```

Current maximum command scales:

- Translation: `0.30`
- Yaw: `0.15`
- Controller timeout: `0.5 s`

Right **A** exits teleoperation. Pressing both thumbsticks requests damping mode.

## Quest connection

1. Put the Quest and host computer on the same network.
2. Open `https://<HOST_IP>:8012` in the Quest browser.
3. Accept the certificate warning and select **Virtual Reality**.
4. Allow hand/controller permissions.
5. Align your hands with the robot’s initial arm pose.
6. Press **r** in the teleoperation terminal to start.
7. Press **q** to quit.

## Safe Quest-only input test

This command starts no DDS client and sends nothing to the G1:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate tv
cd ~/Documents/fibo/project_humanoid/xr_teleoperate

python teleop/inspect_hybrid_input.py --frequency 10
```

Verify that both controllers become active, centered sticks produce `Move(0, 0, 0)`, and stale input returns the affected axes to zero before running the physical motion-mode command.

## XR Teleop Web UI

From the repository root, start the local UI and open
`http://127.0.0.1:8080`:

```bash
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
