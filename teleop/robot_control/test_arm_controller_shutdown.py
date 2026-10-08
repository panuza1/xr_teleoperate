import ast
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

from teleop.robot_control.test_arm_only_message_equivalence import _module_stubs


class _Publisher:
    instances = []

    def __init__(self, _topic, _message_type):
        self.writes = 0
        self.first_command = None
        self.wrote = threading.Event()
        type(self).instances.append(self)

    def Init(self):
        pass

    def Write(self, _message):
        self.writes += 1
        if self.first_command is None:
            self.first_command = tuple(command.q for command in _message.motor_cmd)
        self.wrote.set()
        if getattr(self, "stop_controller", None) is not None:
            self.stop_controller._stop_event.set()


class _StuckThread:
    def __init__(self):
        self.join_timeout = None

    def is_alive(self):
        return True

    def join(self, timeout):
        self.join_timeout = timeout


class _InlineThread:
    seed_state = True
    stale_state = False

    def __init__(self, target):
        self.target = target
        self.daemon = False

    def start(self):
        controller = self.target.__self__
        if self.target.__name__ == "_subscribe_motor_state":
            if self.seed_state:
                state = controller.lowstate_buffer.GetData()
                if state is None:
                    state = types.SimpleNamespace(motor_state=[
                        types.SimpleNamespace(q=(index - 17) * 0.01, dq=0.0)
                        for index in range(35)
                    ], mode_machine=7)
                controller.lowstate_buffer.SetData(state)
                if self.stale_state:
                    with controller.lowstate_buffer.lock:
                        controller.lowstate_buffer.updated_at = time.monotonic() - 1.0
        elif self.target.__name__ == "_ctrl_motor_state":
            _Publisher.instances[-1].stop_controller = controller
            self.target()

    def is_alive(self):
        return False

    def join(self, _timeout=None, **_kwargs):
        pass


def _load_robot_arm():
    path = Path(__file__).with_name("robot_arm.py")
    stubs = _module_stubs()
    stubs["unitree_sdk2py.core.channel"].ChannelPublisher = _Publisher
    module = types.ModuleType("robot_arm_shutdown_test")
    module.__file__ = str(path)
    with patch.dict(sys.modules, stubs):
        exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"), module.__dict__)
    return module


class ArmControllerShutdownTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_robot_arm()

    def setUp(self):
        _Publisher.instances.clear()

    def test_publisher_stops_and_stop_is_idempotent(self):
        controller = self.module.G1_29_ArmController(simulation_mode=True)
        publisher = _Publisher.instances[-1]
        self.assertTrue(publisher.wrote.wait(0.5))
        self.assertTrue(controller.publish_thread.is_alive())

        self.assertTrue(controller.stop())
        writes_after_stop = publisher.writes
        time.sleep(0.02)

        self.assertEqual(publisher.writes, writes_after_stop)
        self.assertFalse(controller.publish_thread.is_alive())
        self.assertTrue(controller.stop())
        self.assertTrue(controller.close())

    def test_real_mode_publisher_and_subscriber_both_stop(self):
        controller = self.module.G1_29_ArmController(simulation_mode=False)
        publisher = _Publisher.instances[-1]
        self.assertTrue(publisher.wrote.wait(0.5))
        self.assertTrue(controller.publish_thread.is_alive())
        self.assertTrue(controller.subscribe_thread.is_alive())

        self.assertTrue(controller.stop())

        self.assertFalse(controller.publish_thread.is_alive())
        self.assertFalse(controller.subscribe_thread.is_alive())

    def test_all_arm_controllers_use_the_stoppable_lifecycle(self):
        lifecycle = self.module._ArmControllerLifecycle
        for name in (
            "G1_29_ArmController",
            "G1_23_ArmController",
            "H1_2_ArmController",
            "H1_ArmController",
            "H2_ArmController",
        ):
            with self.subTest(name=name):
                self.assertTrue(issubclass(getattr(self.module, name), lifecycle))

    def test_stop_is_safe_for_partial_initialization(self):
        controller = self.module.G1_29_ArmController.__new__(
            self.module.G1_29_ArmController
        )
        self.assertTrue(controller.stop())
        controller._init_lifecycle()
        self.assertTrue(controller.stop())

    def test_stop_join_is_bounded(self):
        controller = self.module.G1_29_ArmController.__new__(
            self.module.G1_29_ArmController
        )
        controller._init_lifecycle()
        controller.publish_thread = stuck = _StuckThread()

        started = time.monotonic()
        self.assertFalse(controller.stop(timeout=0.01))

        self.assertLess(time.monotonic() - started, 0.1)
        self.assertGreaterEqual(stuck.join_timeout, 0.0)
        self.assertLessEqual(stuck.join_timeout, 0.01)

    def test_g1_23_seeds_first_arm_command_from_measured_positions(self):
        _InlineThread.seed_state = True
        _InlineThread.stale_state = False
        with patch.object(self.module.threading, "Thread", _InlineThread):
            controller = self.module.G1_23_ArmController(
                motion_mode=True, simulation_mode=False
            )
        first_command = _Publisher.instances[-1].first_command
        arm_ids = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
        expected = tuple((index - 17) * 0.01 for index in arm_ids)
        np.testing.assert_allclose(controller.q_target, expected)
        np.testing.assert_allclose([first_command[index] for index in arm_ids], expected)
        self.assertTrue(controller.stop())

    def test_g1_23_rejects_missing_or_stale_state_before_publishing(self):
        _InlineThread.seed_state = False
        with patch.object(self.module, "G1_23_STATE_TIMEOUT", 0.01), patch.object(
            self.module.threading, "Thread", _InlineThread
        ):
            with self.assertRaisesRegex(RuntimeError, "fresh valid lowstate"):
                self.module.G1_23_ArmController(motion_mode=True)
        self.assertEqual(_Publisher.instances[-1].writes, 0)

        _InlineThread.seed_state = True
        _InlineThread.stale_state = True
        with patch.object(self.module, "G1_23_STATE_TIMEOUT", 0.01), patch.object(
            self.module.threading, "Thread", _InlineThread
        ):
            with self.assertRaisesRegex(RuntimeError, "fresh valid lowstate"):
                self.module.G1_23_ArmController(motion_mode=True)
        self.assertEqual(_Publisher.instances[-1].writes, 0)
        _InlineThread.stale_state = False

    def test_g1_23_rejects_nonfinite_and_out_of_range_joint_state(self):
        state = types.SimpleNamespace(motor_state=[
            types.SimpleNamespace(q=(index - 17) * 0.01, dq=0.0)
            for index in range(35)
        ], mode_machine=7)
        self.assertTrue(self.module.G1_23_ArmController._valid_g1_23_state(state))
        state.mode_machine = None
        self.assertFalse(self.module.G1_23_ArmController._valid_g1_23_state(state))
        state.mode_machine = 7
        state.motor_state[15].q = float("nan")
        self.assertFalse(self.module.G1_23_ArmController._valid_g1_23_state(state))
        state.motor_state[15].q = 9.0
        self.assertFalse(self.module.G1_23_ArmController._valid_g1_23_state(state))

    def test_hold_requires_fresh_state_and_never_targets_home(self):
        controller = self.module.G1_29_ArmController(simulation_mode=True)
        expected = controller.get_current_dual_arm_q().copy()
        self.assertTrue(controller.hold_current_position())
        np.testing.assert_allclose(controller.q_target, expected)
        np.testing.assert_allclose(controller.tauff_target, np.zeros_like(expected))
        with controller.lowstate_buffer.lock:
            controller.lowstate_buffer.updated_at = time.monotonic() - 1.0
        self.assertFalse(controller.hold_current_position())
        self.assertTrue(controller.stop())

    def test_q_and_keyboard_interrupt_share_finally_cleanup(self):
        path = Path(__file__).parents[1] / "teleop_hand_and_arm.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        main_try = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Try)
            and any(
                isinstance(handler.type, ast.Name)
                and handler.type.id == "KeyboardInterrupt"
                for handler in node.handlers
            )
        )
        cleanup_calls = {
            node.func.attr
            for node in ast.walk(ast.Module(body=main_try.finalbody, type_ignores=[]))
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        self.assertIn("stop", cleanup_calls)
        finalbody = ast.Module(body=main_try.finalbody, type_ignores=[])
        home_calls = [
            node for node in ast.walk(finalbody)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "ctrl_dual_arm_go_home"
        ]
        stop_line = next(
            node.lineno
            for node in ast.walk(finalbody)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "arm_ctrl"
            and node.func.attr == "stop"
        )
        restore_line = next(
            node.lineno
            for node in ast.walk(finalbody)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "_restore_g1_mode"
        )
        ee_stop_line = next(
            node.lineno
            for node in ast.walk(finalbody)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "stop_ee"
        )
        self.assertEqual(len(home_calls), 1)
        self.assertLess(home_calls[0].lineno, stop_line)
        self.assertLess(stop_line, ee_stop_line)
        self.assertLess(ee_stop_line, restore_line)
        source = path.read_text(encoding="utf-8")
        self.assertIn("--return-arms-home-on-exit", source)
        self.assertIn('EXIT_REASON in ("keyboard_quit", "controller_exit")', source)
        self.assertIn('EXIT_REASON = "keyboard_interrupt"', source)
        self.assertIn('EXIT_REASON = "exception"', source)
        self.assertIn("loco_wrapper.stop_motion()", source)
        self.assertIn("arm_ctrl.hold_current_position()", source)

        on_press = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "on_press"
        )
        self.assertTrue(
            any(
                isinstance(node, ast.Assign)
                and any(
                    isinstance(target, ast.Name) and target.id == "STOP"
                    for target in node.targets
                )
                and isinstance(node.value, ast.Constant)
                and node.value.value is True
                for node in ast.walk(on_press)
            )
        )

    def test_partial_startup_resources_default_to_none(self):
        path = Path(__file__).parents[1] / "teleop_hand_and_arm.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        main_guard = next(
            node
            for node in tree.body
            if isinstance(node, ast.If)
            and isinstance(node.test, ast.Compare)
            and isinstance(node.test.left, ast.Name)
            and node.test.left.id == "__name__"
        )
        main_try = next(node for node in main_guard.body if isinstance(node, ast.Try))
        initialized = {
            target.id
            for statement in main_guard.body
            if isinstance(statement, ast.Assign)
            and statement.lineno < main_try.lineno
            and isinstance(statement.value, ast.Constant)
            and statement.value.value is None
            for target in statement.targets
            if isinstance(target, ast.Name)
        }
        self.assertTrue(
            {
                "arm_ctrl",
                "ee_ctrl",
                "img_client",
                "ipc_server",
                "listen_keyboard_thread",
                "motion_switcher",
                "recorder",
                "sim_state_subscriber",
                "tv_wrapper",
            }.issubset(initialized)
        )


if __name__ == "__main__":
    unittest.main()
