import time
import unittest
from types import SimpleNamespace

from teleop.robot_control.g1_23_readonly_state import inspect_samples


def lowstate(mode_machine=3):
    motor = lambda: SimpleNamespace(q=0.0, dq=0.0)
    return SimpleNamespace(mode_machine=mode_machine, motor_state=[motor() for _ in range(35)])


class _FakeSubscriber:
    def __init__(self, message=None):
        self.message = message

    def Read(self, timeout=None):
        time.sleep(min(timeout or 0.001, 0.002))
        return self.message


class _SequenceSubscriber:
    def __init__(self, messages):
        self.messages = iter(messages)

    def Read(self, timeout=None):
        time.sleep(min(timeout or 0.001, 0.002))
        return next(self.messages, None)


class G123ReadOnlyStateTest(unittest.TestCase):
    def test_fresh_finite_consistent_state_passes(self):
        result = inspect_samples(_FakeSubscriber(lowstate()), duration_s=0.03, startup_timeout_s=0.02)
        self.assertTrue(result["ready"])
        self.assertEqual(result["motor_count"], 35)
        self.assertEqual(result["mode_machine"], 3)
        self.assertGreaterEqual(result["rate_hz"], 1.0)

    def test_missing_state_fails_bounded_startup(self):
        result = inspect_samples(_FakeSubscriber(), duration_s=0.02, startup_timeout_s=0.01)
        self.assertFalse(result["ready"])
        self.assertEqual(result["samples"], [])

    def test_nonfinite_or_wrong_length_state_fails(self):
        bad = lowstate()
        bad.motor_state[15].q = float("nan")
        result = inspect_samples(_FakeSubscriber(bad), duration_s=0.02, startup_timeout_s=0.01)
        self.assertFalse(result["ready"])
        self.assertFalse(result["motor_finite"])

        short = lowstate()
        short.motor_state.pop()
        result = inspect_samples(_FakeSubscriber(short), duration_s=0.02, startup_timeout_s=0.01)
        self.assertFalse(result["ready"])
        self.assertEqual(result["motor_count"], 34)

    def test_state_loss_and_mode_change_fail_readiness(self):
        lost = inspect_samples(_SequenceSubscriber([lowstate()]), duration_s=0.52, startup_timeout_s=0.02)
        self.assertFalse(lost["ready"])
        self.assertGreaterEqual(lost["age_s"], 0.5)

        changed = inspect_samples(
            _SequenceSubscriber([lowstate(3), lowstate(4)]),
            duration_s=0.02,
            startup_timeout_s=0.02,
        )
        self.assertFalse(changed["ready"])
        self.assertFalse(changed["mode_consistent"])


if __name__ == "__main__":
    unittest.main()
