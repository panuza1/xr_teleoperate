#!/usr/bin/env python3
"""Inspect Quest controller poses and locomotion input without initializing DDS."""

import argparse
import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from televuer import TeleVuerWrapper
from teleop.utils.controller_locomotion import controller_input_fresh, controller_tracking_fresh, controller_velocity


def main():
    parser = argparse.ArgumentParser(description="Inspect Quest input; no DDS or robot commands.")
    parser.add_argument("--frequency", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; zero runs until Ctrl-C.")
    parser.add_argument("--input-mode", choices=("controller", "hybrid"), default="controller")
    parser.add_argument("--locomotion-input", choices=("quest", "unitree", "none"), default="quest")
    args = parser.parse_args()

    print(f"Locomotion source: {args.locomotion_input.upper()}", flush=True)
    if args.locomotion_input != "quest":
        print("Quest locomotion commands: disabled (read-only/no DDS inspection)", flush=True)

    tv = TeleVuerWrapper(
        use_hand_tracking=args.input_mode == "hybrid",
        use_controller_input=args.locomotion_input == "quest",
        img_shape=(480, 1280),
        display_mode="pass-through",
    )
    print("NO DDS: open https://<HOST-IP>:8012 and enter VR", flush=True)
    started = time.monotonic()
    try:
        while not args.duration or time.monotonic() - started < args.duration:
            data = tv.get_tele_data()
            now = time.monotonic()
            left_age = now - data.left_controller_data_updated_at if data.left_controller_data_updated_at else float("inf")
            right_age = now - data.right_controller_data_updated_at if data.right_controller_data_updated_at else float("inf")
            vx, vy, vyaw = controller_velocity(data, now) if args.locomotion_input == "quest" else (0.0, 0.0, 0.0)
            print(
                f"ARM TRACKING: {'active' if controller_tracking_fresh(data, now) else 'waiting/stale'} | "
                f"RIGHT CONTROLLER: {'active' if controller_input_fresh(data, now, 'right') else 'waiting/stale'} | "
                f"LEFT CONTROLLER: {'active' if controller_input_fresh(data, now, 'left') else 'waiting/stale'} | "
                f"LEFT WRIST XYZ: {data.left_wrist_pose[:3, 3].round(3)} | "
                f"RIGHT WRIST XYZ: {data.right_wrist_pose[:3, 3].round(3)} | "
                f"RIGHT STICK: {data.right_ctrl_thumbstickValue} | "
                f"LEFT STICK: {data.left_ctrl_thumbstickValue} | "
                f"RIGHT AGE: {right_age:.3f}s | LEFT AGE: {left_age:.3f}s | "
                f"Move({vx:+.3f}, {vy:+.3f}, {vyaw:+.3f})",
                flush=True,
            )
            time.sleep(1.0 / args.frequency)
    except KeyboardInterrupt:
        pass
    finally:
        tv.close()


if __name__ == "__main__":
    main()
