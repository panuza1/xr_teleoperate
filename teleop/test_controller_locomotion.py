import unittest
import struct
from types import SimpleNamespace

from teleop.utils.controller_locomotion import (
    CONTROLLER_VELOCITY_SCALE,
    CONTROLLER_YAW_SCALE,
    apply_controller_locomotion,
    controller_locomotion_enabled,
    controller_tracking_fresh,
    locomotion_owner,
)
from teleop.utils.motion_switcher import LocoClientWrapper
from teleop.utils.unitree_remote_monitor import UnitreeRemoteMonitor, parse_wireless_remote


class FakeLocoClient:
    def __init__(self):
        self.calls = []

    def Move(self, vx, vy, vyaw):
        self.calls.append(("Move", vx, vy, vyaw))

    def Damp(self):
        self.calls.append(("Damp",))


def tele_data(left=(0.0, 0.0), right=(0.0, 0.0),
              left_updated_at=10.0, right_updated_at=10.0,
              left_pressed=False, right_pressed=False, a_button=False):
    return SimpleNamespace(
        controller_data_updated_at=right_updated_at,
        left_controller_data_updated_at=left_updated_at,
        right_controller_data_updated_at=right_updated_at,
        left_ctrl_thumbstickValue=left,
        right_ctrl_thumbstickValue=right,
        left_ctrl_thumbstick=left_pressed,
        right_ctrl_thumbstick=right_pressed,
        right_ctrl_aButton=a_button,
    )


class ControllerLocomotionTest(unittest.TestCase):
    def assert_move(self, left, right, expected, now=10.1,
                    left_updated_at=10.0, right_updated_at=10.0):
        client = FakeLocoClient()
        apply_controller_locomotion(
            tele_data(left, right, left_updated_at, right_updated_at), client, now
        )
        self.assertEqual(client.calls[0][0], "Move")
        for actual, wanted in zip(client.calls[0][1:], expected):
            self.assertAlmostEqual(actual, wanted)

    def test_g1_motion_mode_enables_controller_locomotion(self):
        self.assertTrue(controller_locomotion_enabled(True, "G1_29"))  # default remains quest
        self.assertTrue(controller_locomotion_enabled(True, "G1_23"))
        self.assertFalse(controller_locomotion_enabled(False, "G1_29"))
        self.assertFalse(controller_locomotion_enabled(True, "H1_2"))

    def test_only_quest_source_can_move(self):
        client = FakeLocoClient()
        apply_controller_locomotion(SimpleNamespace(), client, now=10.1, source="unitree")
        self.assertEqual(client.calls, [])
        apply_controller_locomotion(SimpleNamespace(), client, now=10.1, source="none")
        self.assertEqual(client.calls, [])

    def test_locomotion_has_one_explicit_owner(self):
        self.assertEqual(locomotion_owner("quest", True, "G1_29"), "quest")
        self.assertEqual(locomotion_owner("unitree", True, "G1_29"), "unitree")
        self.assertIsNone(locomotion_owner("none", True, "G1_29"))
        self.assertIsNone(locomotion_owner("quest", False, "G1_29"))

    def test_center_directions_and_combinations(self):
        translation = CONTROLLER_VELOCITY_SCALE
        yaw = CONTROLLER_YAW_SCALE
        cases = (
            ((0, 0), (0, 0), (0, 0, 0)),
            ((0, -1), (0, 0), (translation, 0, 0)),
            ((0, 1), (0, 0), (-translation, 0, 0)),
            ((-1, 0), (0, 0), (0, translation, 0)),
            ((1, 0), (0, 0), (0, -translation, 0)),
            ((0, 0), (-1, 0), (0, 0, yaw)),
            ((0, 0), (1, 0), (0, 0, -yaw)),
            ((0, 0), (0, -1), (0, 0, 0)),
            ((-1, -1), (-1, 0), (translation, translation, yaw)),
            ((1, 0), (1, 0), (0, -translation, -yaw)),
        )
        for left, right, expected in cases:
            with self.subTest(left=left, right=right):
                self.assert_move(left, right, expected)

    def test_controller_freshness_is_independent(self):
        self.assert_move(
            (0, -1), (-1, 0), (0, 0, 0.3), now=10.5,
            left_updated_at=10.0, right_updated_at=10.4,
        )
        self.assert_move(
            (0, -1), (-1, 0), (0.3, 0, 0), now=10.5,
            left_updated_at=10.4, right_updated_at=10.0,
        )
        self.assert_move(
            (0, -1), (-1, 0), (0, 0, 0), now=10.5,
            left_updated_at=10.0, right_updated_at=10.0,
        )

    def test_arm_tracking_requires_both_controller_streams_to_be_fresh(self):
        self.assertTrue(controller_tracking_fresh(tele_data(), now=10.1))
        self.assertFalse(
            controller_tracking_fresh(
                tele_data(left_updated_at=9.5, right_updated_at=10.0), now=10.1
            )
        )

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
            tele_data(
                (0, -1), (-1, 0), left_updated_at=9.5,
                left_pressed=True, right_pressed=True,
            ),
            client,
            now=10.1,
        )
        self.assertEqual(client.calls, [("Move", 0.0, 0.0, 0.3)])

    def test_a_button_exit_requires_fresh_input(self):
        fresh = FakeLocoClient()
        stale = FakeLocoClient()
        self.assertTrue(apply_controller_locomotion(tele_data(a_button=True), fresh, now=10.1))
        self.assertFalse(
            apply_controller_locomotion(
                tele_data(left_updated_at=10.0, right_updated_at=9.5, a_button=True),
                stale,
                now=10.1,
            )
        )

    def test_wireless_remote_parser(self):
        data = bytearray(40)
        struct.pack_into("<f", data, 4, -0.1)
        struct.pack_into("<f", data, 8, 0.2)
        struct.pack_into("<f", data, 12, -0.3)
        struct.pack_into("<f", data, 20, 0.4)
        data[2] = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)
        data[3] = (1 << 0) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7)

        state = parse_wireless_remote(data)
        self.assertAlmostEqual(state.lx, -0.1)
        self.assertAlmostEqual(state.ly, 0.4)
        self.assertAlmostEqual(state.rx, 0.2)
        self.assertAlmostEqual(state.ry, -0.3)
        self.assertTrue(all(getattr(state, name) for name in (
            "a", "y", "l1", "l2", "r1", "r2", "start", "select",
            "dpad_up", "dpad_down", "dpad_left", "dpad_right",
        )))
        self.assertTrue(state.connected)

    def test_remote_monitor_is_read_only_and_tracks_connection(self):
        class Subscriber:
            def Init(self, handler, queue_len):
                self.handler = handler
                self.queue_len = queue_len

            def Close(self):
                self.closed = True

        subscriber = Subscriber()
        monitor = UnitreeRemoteMonitor(subscriber=subscriber).start()
        subscriber.handler(SimpleNamespace(wireless_remote=bytes(40)))
        self.assertFalse(monitor.connected)
        data = bytearray(40)
        data[3] = 1
        subscriber.handler(SimpleNamespace(wireless_remote=data))
        self.assertTrue(monitor.connected)
        self.assertIsNotNone(monitor.last_state)
        monitor.close()
        self.assertTrue(subscriber.closed)


if __name__ == "__main__":
    unittest.main()
