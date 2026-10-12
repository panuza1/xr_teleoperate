#!/usr/bin/env python3
"""Bounded Unitree G1 lowstate inspection; creates no DDS publishers."""

import argparse
import time

import numpy as np


ARM_SLOTS = {
    "left": (15, 16, 17, 18, 19),
    "right": (22, 23, 24, 25, 26),
}
STATE_MAX_AGE = 0.5


def inspect_samples(subscriber, duration_s=5.0, startup_timeout_s=5.0):
    """Read lowstate for a bounded interval and return measurements plus readiness."""
    if duration_s <= 0 or startup_timeout_s <= 0:
        raise ValueError("timeouts must be positive")
    started = time.monotonic()
    startup_deadline = started + startup_timeout_s
    samples = []
    deadline = startup_deadline
    while time.monotonic() < deadline:
        msg = subscriber.Read(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
        received_at = time.monotonic()
        if msg is None:
            if not samples and received_at >= startup_deadline:
                break
            continue
        samples.append((received_at, msg))
        if len(samples) == 1:
            deadline = received_at + duration_s

    if not samples:
        return {"ready": False, "reason": "no lowstate received before startup timeout", "samples": []}

    received_at, latest = samples[-1]
    motor_state = getattr(latest, "motor_state", ())
    try:
        q = np.asarray([motor.q for motor in motor_state], dtype=np.float64)
        dq = np.asarray([motor.dq for motor in motor_state], dtype=np.float64)
        motor_finite = q.shape == (35,) and dq.shape == (35,) and np.isfinite(q).all() and np.isfinite(dq).all()
    except (AttributeError, TypeError, ValueError):
        q = dq = np.asarray([], dtype=np.float64)
        motor_finite = False
    modes = [getattr(msg, "mode_machine", None) for _, msg in samples]
    gaps = [right[0] - left[0] for left, right in zip(samples, samples[1:])]
    rate_hz = (len(samples) - 1) / (received_at - samples[0][0]) if len(samples) > 1 and received_at > samples[0][0] else 0.0
    age_s = time.monotonic() - received_at
    ready = (
        motor_finite and all(mode is not None for mode in modes)
        and len(set(modes)) == 1 and len(samples) > 1 and rate_hz >= 1.0
        and max(gaps, default=float("inf")) < STATE_MAX_AGE and age_s < STATE_MAX_AGE
    )
    return {
        "ready": ready,
        "reason": "fresh, finite, stable lowstate" if ready else "stale or invalid lowstate",
        "samples": samples,
        "rate_hz": rate_hz,
        "age_s": age_s,
        "mode_machine": modes[-1],
        "mode_consistent": len(set(modes)) == 1,
        "motor_count": len(motor_state),
        "motor_finite": motor_finite,
        "max_gap_s": max(gaps, default=0.0),
        "q": q,
        "dq": dq,
        "latest": latest,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="verified wired interface connected to the G1")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds to sample after first state")
    parser.add_argument("--startup-timeout", type=float, default=5.0)
    args = parser.parse_args()
    if not args.interface or args.duration <= 0 or args.startup_timeout <= 0:
        parser.error("interface and positive timeouts are required")

    # Imports are kept here so --help and offline tests cannot initialize DDS.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(0, networkInterface=args.interface)
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    try:
        subscriber.Init()
        result = inspect_samples(subscriber, args.duration, args.startup_timeout)
    finally:
        subscriber.Close()

    print(f"LOWSTATE READINESS: {'PASS' if result['ready'] else 'FAIL'} — {result['reason']}")
    if not result["samples"]:
        return 1
    print(
        f"samples={len(result['samples'])} rate={result['rate_hz']:.1f} Hz "
        f"age={result['age_s']:.3f}s max_gap={result['max_gap_s']:.3f}s "
        f"mode_machine={result['mode_machine']} consistent={result['mode_consistent']} "
        f"motor_state_count={result['motor_count']} finite_q_dq={result['motor_finite']}"
    )
    print(
        f"rx_monotonic_start={result['samples'][0][0]:.6f} "
        f"rx_monotonic_end={result['samples'][-1][0]:.6f} "
        f"mode_pr={getattr(result['latest'], 'mode_pr', None)} "
        f"tick={getattr(result['latest'], 'tick', None)}"
    )
    for side, slots in ARM_SLOTS.items():
        print(f"{side} arm slots {slots}: " + " ".join(
            f"{i}:q={result['q'][i]:+.4f},dq={result['dq'][i]:+.4f},"
            f"tau={result['latest'].motor_state[i].tau_est:+.3f},"
            f"V={result['latest'].motor_state[i].vol:.2f},"
            f"temp={tuple(result['latest'].motor_state[i].temperature)},"
            f"motorstate=0x{int(result['latest'].motor_state[i].motorstate):08x}"
            for i in slots
        ))
    return 0 if result["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
