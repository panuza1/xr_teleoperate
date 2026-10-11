#!/usr/bin/env python3
"""Inspect Quest body/controller events and locomotion input without DDS."""

import argparse
import importlib.metadata
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "televuer" / "src"))
sys.path.append(str(Path(__file__).resolve().parent.parent))

from televuer import BODY_JOINT_INDEX, TeleVuerWrapper
from teleop.utils.controller_locomotion import controller_input_fresh, controller_velocity
import vuer


def main():
    parser = argparse.ArgumentParser(description="Inspect Quest input; no DDS or robot commands.")
    parser.add_argument("--frequency", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; zero runs until Ctrl-C.")
    parser.add_argument("--input-mode", choices=("controller", "hybrid", "body-controller"), default="body-controller")
    parser.add_argument("--locomotion-input", choices=("quest", "unitree", "none"), default="quest")
    parser.add_argument("--debug-xr-events", action="store_true", help="Log Vuer client lifecycle and raw XR event metadata.")
    args = parser.parse_args()

    print(f"Input mode: {args.input_mode.upper()} | Locomotion source: {args.locomotion_input.upper()}", flush=True)
    if args.debug_xr_events:
        print(f"Vuer: {importlib.metadata.version('vuer')} at {vuer.__file__}", flush=True)
    if args.locomotion_input != "quest":
        print("Quest locomotion commands: disabled (read-only/no DDS inspection)", flush=True)

    tv = TeleVuerWrapper(
        use_hand_tracking=args.input_mode == "hybrid",
        use_body_tracking=args.input_mode == "body-controller",
        use_controller_input=True,
        img_shape=(480, 1280),
        display_mode="pass-through",
        debug_xr_events=args.debug_xr_events,
    )
    print("NO DDS: open https://<HOST-IP>:8012 and enter VR", flush=True)
    started = time.monotonic()
    try:
        while not args.duration or time.monotonic() - started < args.duration:
            data = tv.get_tele_data()
            now = time.monotonic()
            body_age = now - data.body_data_updated_at if data.body_data_updated_at else float("inf")
            left_age = now - data.left_controller_data_updated_at if data.left_controller_data_updated_at else float("inf")
            right_age = now - data.right_controller_data_updated_at if data.right_controller_data_updated_at else float("inf")
            vx, vy, vyaw = controller_velocity(data, now) if args.locomotion_input == "quest" else (0.0, 0.0, 0.0)
            poses = data.body_poses
            xyz = lambda name: poses[BODY_JOINT_INDEX[name]][:3, 3].round(3) if poses is not None else "waiting"
            print(
                f"BODY TRACKING: {'active' if 0.0 <= body_age < 0.5 else 'stale'} ({body_age:.3f}s) | "
                f"LEFT SHOULDER XYZ: {xyz('left-shoulder')} | LEFT ELBOW XYZ: {xyz('left-arm-lower')} | "
                f"LEFT WRIST XYZ: {xyz('left-hand-wrist-twist')} | "
                f"RIGHT SHOULDER XYZ: {xyz('right-shoulder')} | RIGHT ELBOW XYZ: {xyz('right-arm-lower')} | "
                f"RIGHT WRIST XYZ: {xyz('right-hand-wrist-twist')} | "
                f"LEFT CONTROLLER: {'active' if controller_input_fresh(data, now, 'left') else 'stale'} ({left_age:.3f}s) | "
                f"RIGHT CONTROLLER: {'active' if controller_input_fresh(data, now, 'right') else 'stale'} ({right_age:.3f}s) | "
                f"LEFT JOYSTICK: {data.left_ctrl_thumbstickValue} | RIGHT JOYSTICK: {data.right_ctrl_thumbstickValue} | "
                f"Move(vx={vx:+.3f}, vy={vy:+.3f}, vyaw={vyaw:+.3f})",
                flush=True,
            )
            time.sleep(1.0 / args.frequency)
    except KeyboardInterrupt:
        pass
    finally:
        tv.close()


if __name__ == "__main__":
    main()
