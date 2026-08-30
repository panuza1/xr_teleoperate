import ast
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from teleop.robot_control import robot_hand_inspire


class _Endpoint:
    def __init__(self, *_args):
        pass

    def Init(self, *_args):
        pass


class _Worker:
    instances = []

    def __init__(self, **_kwargs):
        self.daemon = False
        self.started = False
        self.alive = False
        self.join_calls = 0
        self.kwargs = _kwargs
        type(self).instances.append(self)

    def start(self):
        self.started = True
        self.alive = True

    def is_alive(self):
        return self.alive

    def join(self, _timeout=None):
        self.join_calls += 1
        self.alive = False


class _Thread(_Worker):
    instances = []


class _Process(_Worker):
    instances = []


class InspireSimulationWorkerTest(unittest.TestCase):
    def setUp(self):
        _Thread.instances.clear()
        _Process.instances.clear()

    def make_controller(self, simulation_mode):
        with (
            patch.object(robot_hand_inspire, "ChannelPublisher", _Endpoint),
            patch.object(robot_hand_inspire, "ChannelSubscriber", _Endpoint),
            patch.object(robot_hand_inspire, "HandRetargeting", return_value=object()),
            patch.object(robot_hand_inspire, "Process", _Process),
            patch.object(robot_hand_inspire.threading, "Thread", _Thread),
            patch.object(robot_hand_inspire.Inspire_Controller_DFX,
                         "_wait_for_hand_state"),
        ):
            return robot_hand_inspire.Inspire_Controller_DFX(
                object(), object(), simulation_mode=simulation_mode
            )

    def test_simulation_uses_thread_for_dds_writer(self):
        self.make_controller(True)
        self.assertTrue(_Thread.instances[0].started)
        self.assertFalse(_Process.instances)

    def test_real_robot_keeps_process_worker(self):
        self.make_controller(False)
        self.assertTrue(_Process.instances[0].started)
        self.assertFalse(_Thread.instances)

    def test_worker_stop_is_bounded_and_idempotent(self):
        controller = self.make_controller(False)
        worker = _Process.instances[0]
        self.assertTrue(controller.stop())
        self.assertFalse(worker.is_alive())
        self.assertEqual(worker.join_calls, 1)
        self.assertTrue(controller.stop())
        self.assertEqual(worker.join_calls, 1)

    def test_inspire_topics_and_right_first_wire_order_are_unchanged(self):
        self.assertEqual(robot_hand_inspire.kTopicInspireDFXCommand, "rt/inspire/cmd")
        self.assertEqual(robot_hand_inspire.kTopicInspireDFXState, "rt/inspire/state")
        self.assertEqual(
            [joint.value for joint in robot_hand_inspire.Inspire_Right_Hand_JointIndex],
            list(range(6)),
        )
        self.assertEqual(
            [joint.value for joint in robot_hand_inspire.Inspire_Left_Hand_JointIndex],
            list(range(6, 12)),
        )

    def test_dex3_trigger_grasp_mapping_is_unchanged(self):
        path = Path(__file__).with_name("robot_hand_unitree.py")
        tree = ast.parse(path.read_text(encoding="utf-8"))
        controller = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "Dex3_Controller_Button_Controller"
        )
        method = next(
            node
            for node in controller.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "_trigger_to_dex3_q"
        )
        namespace = {"np": np}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
        trigger_to_q = namespace["_trigger_to_dex3_q"]
        expected = np.array([0.35, 0.75, 0.75, 0.90, 0.90, 0.90, 0.90])
        np.testing.assert_allclose(trigger_to_q(None, 0.0), np.zeros(7))
        np.testing.assert_allclose(trigger_to_q(None, 1.0), expected)
        np.testing.assert_allclose(trigger_to_q(None, 2.0), expected)


if __name__ == "__main__":
    unittest.main()
