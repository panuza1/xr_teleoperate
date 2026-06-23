#!/usr/bin/env python3
"""Quest viewer for Isaac Sim camera stream. No robot DDS/control."""

import argparse
import time

from teleimager.image_client import ImageClient
from televuer import TeleVuerWrapper


def main():
    parser = argparse.ArgumentParser(description="Quest viewer only: Vuer + teleimager, no robot control")
    parser.add_argument("--img-server-ip", required=True, help="LAN IP of the machine running sim_core_smoke.py --quest")
    parser.add_argument("--input-mode", choices=["hand", "controller"], default="hand")
    parser.add_argument("--display-mode", choices=["immersive", "ego", "pass-through"], default="ego")
    parser.add_argument("--frequency", type=float, default=30.0)
    args = parser.parse_args()

    img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
    camera_config = img_client.get_cam_config()
    head_config = camera_config["head_camera"]
    need_local_img = not (args.display_mode == "pass-through" or head_config["enable_webrtc"])

    tv_wrapper = TeleVuerWrapper(
        use_hand_tracking=args.input_mode == "hand",
        binocular=head_config["binocular"],
        img_shape=head_config["image_shape"],
        display_fps=head_config.get("fps", args.frequency),
        display_mode=args.display_mode,
        zmq=head_config["enable_zmq"],
        webrtc=head_config["enable_webrtc"],
        webrtc_url=f"https://{args.img_server_ip}:{head_config['webrtc_port']}/offer",
        arm_reference_mode="head_yaw",
    )

    print("[quest_viewer] running: no robot DDS/control", flush=True)
    print(f"[quest_viewer] open Quest browser: https://{args.img_server_ip}:8012/?ws=wss://{args.img_server_ip}:8012", flush=True)

    try:
        while True:
            if need_local_img:
                head_img = img_client.get_head_frame()
                if head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
            time.sleep(1.0 / args.frequency)
    except KeyboardInterrupt:
        pass
    finally:
        tv_wrapper.tvuer.close()


if __name__ == "__main__":
    main()
