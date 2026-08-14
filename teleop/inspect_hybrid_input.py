#!/usr/bin/env python3
"""Inspect simultaneous Quest hand/controller input without initializing DDS."""

import argparse
import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from televuer import TeleVuerWrapper
from teleop.utils.controller_locomotion import controller_input_fresh, controller_velocity


def main():
    parser = argparse.ArgumentParser(description="Inspect hybrid Quest input; no DDS or robot commands.")
    parser.add_argument("--frequency", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; zero runs until Ctrl-C.")
    args = parser.parse_args()

    tv = TeleVuerWrapper(
        use_hand_tracking=True,
        use_controller_input=True,
        img_shape=(480, 1280),
        display_mode="pass-through",
    )
    print("NO DDS: open https://<HOST-IP>:8012 and enter VR", flush=True)
    started = time.monotonic()
    try:
        while not args.duration or time.monotonic() - started < args.duration:
            data = tv.get_tele_data()
            now = time.monotonic()
            age = now - data.controller_data_updated_at if data.controller_data_updated_at else float("inf")
            vx, vy, vyaw = controller_velocity(data, now)
            print(
                f"HAND: {'active' if data.motion_data_ready else 'waiting'} | "
                f"CONTROLLER: {'active' if controller_input_fresh(data, now) else 'waiting/stale'} | "
                f"LEFT STICK: {data.left_ctrl_thumbstickValue} | "
                f"RIGHT STICK: {data.right_ctrl_thumbstickValue} | "
                f"CONTROLLER AGE: {age:.3f}s | "
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
