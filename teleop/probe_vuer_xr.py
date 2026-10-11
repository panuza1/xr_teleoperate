#!/usr/bin/env python3
"""Standalone Vuer WebXR event probe; does not import xr_teleoperate."""

import asyncio
import os
import time
import traceback
from pathlib import Path

from vuer import Vuer
from vuer.schemas import Body, MotionControllers

BODY_JOINTS = (
    "hips", "spine-lower", "spine-middle", "spine-upper", "chest", "neck", "head",
    "left-shoulder", "left-scapula", "left-arm-upper", "left-arm-lower", "left-hand-wrist-twist",
    "right-shoulder", "right-scapula", "right-arm-upper", "right-arm-lower", "right-hand-wrist-twist",
)


async def event_logger(event, session):
    value = getattr(event, "value", None)
    value = value if isinstance(value, dict) else {}
    print(
        f"{time.strftime('%Y-%m-%d %H:%M:%S')} EVENT {getattr(event, 'etype', '?')} "
        f"ts={getattr(event, 'ts', None)} event_keys={list(getattr(event, '__dict__', {}))} "
        f"value_keys={list(value)}",
        flush=True,
    )
    if getattr(event, "etype", None) == "CONTROLLER_MOVE":
        for side in ("left", "right"):
            state = value.get(f"{side}State") or {}
            print(f"  {side} joystick={state.get('thumbstickValue')} buttons={state}", flush=True)
        print("  XR controller stream observed (immersive XR input is active)", flush=True)
    elif getattr(event, "etype", None) in ("BODY_MOVE", "BODY_TRACKING_MOVE"):
        if isinstance(value.get("body"), (list, tuple)):
            n = len(value["body"]) // 16
            print(f"  body joint order first={list(BODY_JOINTS[:min(n, 8)])}; count={n}", flush=True)
        else:
            print(f"  body joint names first={list(value)[:8]}", flush=True)


def main():
    cert = os.getenv("XR_TELEOP_CERT") or str(Path.home() / ".config/xr_teleoperate/cert.pem")
    key = os.getenv("XR_TELEOP_KEY") or str(Path.home() / ".config/xr_teleoperate/key.pem")
    print(f"Starting standalone Vuer probe pid={os.getpid()} cert={cert} key={key}", flush=True)
    app = Vuer(host="0.0.0.0", cert=cert, key=key, queries={"grid": False}, queue_len=3)

    for name in ("INIT", "CAMERA_MOVE", "CONTROLLER_MOVE", "BODY_MOVE", "BODY_TRACKING_MOVE"):
        app.add_handler(name, event_logger)
        print(f"Registered handler before app.run: {name}", flush=True)

    @app.spawn
    async def on_client(session):
        print("CLIENT CONNECTED: INIT received; inserting XR components", flush=True)
        try:
            session.upsert(
                MotionControllers(stream=True, key="probe-controllers", left=True, right=True),
                to="bgChildren",
            )
            print("SCENE INSERTED MotionControllers stream=True left=True right=True to=bgChildren", flush=True)
            session.upsert(
                Body(stream=True, key="probe-body", leftHand=False, rightHand=False,
                     fps=30, showBody=False, showFrame=False),
                to="bgChildren",
            )
            print("SCENE INSERTED Body stream=True to=bgChildren", flush=True)
        except Exception:
            print("SCENE INSERT FAILED", flush=True)
            traceback.print_exc()
            raise
        print("Awaiting XR events. Start immersive VR in Quest Browser; controller events confirm XR session.", flush=True)
        await session.forever()

    try:
        app.run()
    except Exception:
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
