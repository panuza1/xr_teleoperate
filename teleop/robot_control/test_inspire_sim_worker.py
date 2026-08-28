import unittest
from unittest.mock import patch

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
        type(self).instances.append(self)

    def start(self):
        self.started = True


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
            robot_hand_inspire.Inspire_Controller_DFX(
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


if __name__ == "__main__":
    unittest.main()
