"""Read-only Unitree G1 wireless remote monitor."""

import math
import struct
from dataclasses import dataclass


REMOTE_DATA_SIZE = 40


@dataclass(frozen=True)
class UnitreeRemoteState:
    lx: float
    ly: float
    rx: float
    ry: float
    a: bool
    b: bool
    x: bool
    y: bool
    l1: bool
    l2: bool
    r1: bool
    r2: bool
    start: bool
    select: bool
    dpad_up: bool
    dpad_down: bool
    dpad_left: bool
    dpad_right: bool
    connected: bool


def _axis(data, offset):
    value = struct.unpack_from("<f", data, offset)[0]
    return value if math.isfinite(value) else 0.0


def parse_wireless_remote(remote_data):
    """Parse the 40-byte G1 lowstate wireless_remote payload."""
    try:
        data = bytes(remote_data)
    except (TypeError, ValueError) as exc:
        raise ValueError("wireless_remote must be a byte sequence") from exc
    if len(data) < REMOTE_DATA_SIZE:
        raise ValueError(f"wireless_remote must contain {REMOTE_DATA_SIZE} bytes")

    buttons1, buttons2 = data[2], data[3]
    return UnitreeRemoteState(
        lx=_axis(data, 4),
        rx=_axis(data, 8),
        ry=_axis(data, 12),
        ly=_axis(data, 20),
        r1=bool(buttons1 & (1 << 0)),
        l1=bool(buttons1 & (1 << 1)),
        start=bool(buttons1 & (1 << 2)),
        select=bool(buttons1 & (1 << 3)),
        r2=bool(buttons1 & (1 << 4)),
        l2=bool(buttons1 & (1 << 5)),
        a=bool(buttons2 & (1 << 0)),
        b=bool(buttons2 & (1 << 1)),
        x=bool(buttons2 & (1 << 2)),
        y=bool(buttons2 & (1 << 3)),
        dpad_up=bool(buttons2 & (1 << 4)),
        dpad_right=bool(buttons2 & (1 << 5)),
        dpad_down=bool(buttons2 & (1 << 6)),
        dpad_left=bool(buttons2 & (1 << 7)),
        connected=any(data),
    )


class UnitreeRemoteMonitor:
    """Subscribe to G1 lowstate and observe, but never command, the remote."""

    def __init__(self, subscriber=None, logger=None):
        self.subscriber = subscriber
        self.logger = logger
        self.connected = False
        self.last_state = None
        self._started = False

    def start(self):
        if self._started:
            return self
        if self.subscriber is None:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

            self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._on_lowstate, 10)
        self._started = True
        return self

    def _on_lowstate(self, lowstate):
        try:
            state = parse_wireless_remote(lowstate.wireless_remote)
        except (AttributeError, TypeError, ValueError) as exc:
            if self.logger:
                self.logger.warning("Unitree remote: invalid wireless_remote: %s", exc)
            return

        self.last_state = state
        if state.connected != self.connected:
            self.connected = state.connected
            if self.logger:
                self.logger.info(
                    "Unitree remote: %s", "connected" if state.connected else "disconnected"
                )

    def close(self):
        if self._started:
            self.subscriber.Close()
            self._started = False

    stop = close
