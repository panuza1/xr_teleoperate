import unittest
from types import SimpleNamespace

from teleop.utils.controller_locomotion import (
    CONTROLLER_VELOCITY_SCALE,
    apply_controller_locomotion,
    controller_locomotion_enabled,
)
from teleop.utils.motion_switcher import LocoClientWrapper


class FakeLocoClient:
    def __init__(self):
        self.calls = []

    def Move(self, vx, vy, vyaw):
        self.calls.append(("Move", vx, vy, vyaw))

    def Damp(self):
        self.calls.append(("Damp",))


def tele_data(left=(0.0, 0.0), right=(0.0, 0.0), updated_at=10.0,
              left_pressed=False, right_pressed=False, a_button=False):
    return SimpleNamespace(
        controller_data_updated_at=updated_at,
        left_ctrl_thumbstickValue=left,
        right_ctrl_thumbstickValue=right,
        left_ctrl_thumbstick=left_pressed,
        right_ctrl_thumbstick=right_pressed,
        right_ctrl_aButton=a_button,
    )


class ControllerLocomotionTest(unittest.TestCase):
    def assert_move(self, left, right, expected, now=10.1, updated_at=10.0):
        client = FakeLocoClient()
        apply_controller_locomotion(tele_data(left, right, updated_at), client, now)
        self.assertEqual(client.calls[0][0], "Move")
        for actual, wanted in zip(client.calls[0][1:], expected):
            self.assertAlmostEqual(actual, wanted)

    def test_g1_motion_mode_enables_controller_locomotion(self):
        self.assertTrue(controller_locomotion_enabled(True, "G1_29"))
        self.assertTrue(controller_locomotion_enabled(True, "G1_23"))
        self.assertFalse(controller_locomotion_enabled(False, "G1_29"))
        self.assertFalse(controller_locomotion_enabled(True, "H1_2"))

    def test_center_and_directions(self):
        scale = CONTROLLER_VELOCITY_SCALE
        cases = (
            ((0, 0), (0, 0), (0, 0, 0)),
            ((0, -1), (0, 0), (scale, 0, 0)),
            ((0, 1), (0, 0), (-scale, 0, 0)),
            ((-1, 0), (0, 0), (0, scale, 0)),
            ((1, 0), (0, 0), (0, -scale, 0)),
            ((0, 0), (-1, 0), (0, 0, scale)),
            ((0, 0), (1, 0), (0, 0, -scale)),
        )
        for left, right, expected in cases:
            with self.subTest(left=left, right=right):
                self.assert_move(left, right, expected)

    def test_stale_controller_commands_zero(self):
        self.assert_move((1, -1), (1, 0), (0, 0, 0), now=10.5, updated_at=10.0)
        self.assert_move((1, -1), (1, 0), (0, 0, 0), now=11.0, updated_at=10.0)

    def test_damping_does_not_move_in_same_iteration(self):
        client = FakeLocoClient()
        apply_controller_locomotion(
            tele_data((0, -1), (1, 0), left_pressed=True, right_pressed=True),
            client,
            now=10.1,
        )
        self.assertEqual(client.calls, [("Damp",)])

    def test_loco_wrapper_damp_delegates_to_sdk_client(self):
        wrapper = LocoClientWrapper.__new__(LocoClientWrapper)
        wrapper.client = FakeLocoClient()
        wrapper.Damp()
        self.assertEqual(wrapper.client.calls, [("Damp",)])

    def test_stale_thumbstick_press_does_not_damp(self):
        client = FakeLocoClient()
        apply_controller_locomotion(
            tele_data((0, -1), (1, 0), left_pressed=True, right_pressed=True),
            client,
            now=10.5,
        )
        self.assertEqual(client.calls, [("Move", 0.0, 0.0, 0.0)])

    def test_a_button_exit_requires_fresh_input(self):
        fresh = FakeLocoClient()
        stale = FakeLocoClient()
        self.assertTrue(apply_controller_locomotion(tele_data(a_button=True), fresh, now=10.1))
        self.assertFalse(apply_controller_locomotion(tele_data(a_button=True), stale, now=10.5))


if __name__ == "__main__":
    unittest.main()
