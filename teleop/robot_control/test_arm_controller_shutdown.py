import ast
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from teleop.robot_control.test_arm_only_message_equivalence import _module_stubs


class _Publisher:
    instances = []

    def __init__(self, _topic, _message_type):
        self.writes = 0
        self.wrote = threading.Event()
        type(self).instances.append(self)

    def Init(self):
        pass

    def Write(self, _message):
        self.writes += 1
        self.wrote.set()


class _StuckThread:
    def __init__(self):
        self.join_timeout = None

    def is_alive(self):
        return True

    def join(self, timeout):
        self.join_timeout = timeout


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
        home_line = next(
            node.lineno
            for node in ast.walk(finalbody)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "ctrl_dual_arm_go_home"
        )
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
        self.assertLess(home_line, stop_line)
        self.assertLess(stop_line, ee_stop_line)
        self.assertLess(ee_stop_line, restore_line)

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
