import ast
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


def _load_motion_switcher():
    names = (
        "unitree_sdk2py",
        "unitree_sdk2py.core",
        "unitree_sdk2py.core.channel",
        "unitree_sdk2py.comm",
        "unitree_sdk2py.comm.motion_switcher",
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client",
        "unitree_sdk2py.g1",
        "unitree_sdk2py.g1.loco",
        "unitree_sdk2py.g1.loco.g1_loco_client",
    )
    stubs = {name: types.ModuleType(name) for name in names}
    stubs["unitree_sdk2py.core.channel"].ChannelFactoryInitialize = lambda *a, **k: None
    stubs[
        "unitree_sdk2py.comm.motion_switcher.motion_switcher_client"
    ].MotionSwitcherClient = object
    stubs["unitree_sdk2py.g1.loco.g1_loco_client"].LocoClient = object
    path = Path(__file__).with_name("motion_switcher.py")
    module = types.ModuleType("motion_switcher_restore_test")
    module.__file__ = str(path)
    with patch.dict(sys.modules, stubs):
        exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"), module.__dict__)
    return module


def _load_restore_helper(logger):
    path = Path(__file__).parents[1] / "teleop_hand_and_arm.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_restore_g1_mode"
    )
    namespace = {"logger_mp": logger}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
    return namespace["_restore_g1_mode"]


class _Client:
    def __init__(self, select=(0, None), checks=(), release=(0, None)):
        self.select = select
        self.checks = list(checks)
        self.release = release
        self.selected = []
        self.releases = 0

    def SelectMode(self, nameOrAlias):
        self.selected.append(nameOrAlias)
        if isinstance(self.select, Exception):
            raise self.select
        return self.select

    def CheckMode(self):
        return self.checks.pop(0)

    def ReleaseMode(self):
        self.releases += 1
        return self.release


class _Logger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class _Switcher:
    def __init__(self, response):
        self.response = response
        self.calls = 0

    def Exit_Debug_Mode(self):
        self.calls += 1
        return self.response


class MotionSwitcherRestoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_motion_switcher()

    def make_switcher(self, client):
        switcher = self.module.MotionSwitcher.__new__(self.module.MotionSwitcher)
        switcher.msc = client
        return switcher

    def test_exit_debug_mode_selects_and_verifies_ai(self):
        client = _Client(checks=[(0, {"name": ""}), (0, {"name": "ai"})])
        status, result = self.make_switcher(client).Exit_Debug_Mode(
            timeout_s=0.1, poll_interval_s=0
        )
        self.assertEqual((status, result), (0, {"name": "ai"}))
        self.assertEqual(client.selected, ["ai"])

    def test_enter_debug_mode_releases_and_verifies_no_active_mode(self):
        client = _Client(checks=[(0, {"name": "ai"}), (0, {"name": ""})])
        status, result = self.make_switcher(client).Enter_Debug_Mode(
            retry_interval_s=0
        )
        self.assertEqual((status, result), (0, {"name": ""}))
        self.assertEqual(client.releases, 1)

    def test_enter_debug_mode_propagates_release_failure(self):
        client = _Client(checks=[(0, {"name": "ai"})], release=(43, None))
        self.assertEqual(
            self.make_switcher(client).Enter_Debug_Mode(retry_interval_s=0),
            (43, {"name": "ai"}),
        )

    def test_enter_debug_mode_attempts_are_bounded(self):
        client = _Client(
            checks=[(0, {"name": "ai"}), (0, {"name": "ai"})]
        )
        status, result = self.make_switcher(client).Enter_Debug_Mode(
            max_attempts=1, retry_interval_s=0
        )
        self.assertIsNone(status)
        self.assertEqual(result, {"name": "ai"})
        self.assertEqual(client.releases, 1)

    def test_exit_debug_mode_does_not_claim_unverified_selection(self):
        client = _Client(checks=[(0, {"name": "other"})])
        status, result = self.make_switcher(client).Exit_Debug_Mode(timeout_s=0)
        self.assertIsNone(status)
        self.assertEqual(result, {"name": "other"})

    def test_exit_debug_mode_propagates_select_failure(self):
        client = _Client(select=(42, None))
        self.assertEqual(
            self.make_switcher(client).Exit_Debug_Mode(timeout_s=0), (42, None)
        )
        self.assertFalse(client.checks)

    def test_exit_debug_mode_handles_sdk_exception(self):
        client = _Client(select=RuntimeError("rpc failed"))
        self.assertEqual(
            self.make_switcher(client).Exit_Debug_Mode(timeout_s=0), (None, None)
        )

    def test_restore_helper_only_calls_real_non_motion_g1_path(self):
        logger = _Logger()
        restore = _load_restore_helper(logger)
        skipped = (
            ("G1_29", False, True, True),
            ("G1_29", True, False, True),
            ("G1_29", False, False, False),
            ("H1", False, False, True),
        )
        for arm, motion, sim, entered in skipped:
            switcher = _Switcher((0, {"name": "ai"}))
            with self.subTest(arm=arm, motion=motion, sim=sim, entered=entered):
                self.assertFalse(restore(switcher, arm, motion, sim, entered, True))
                self.assertEqual(switcher.calls, 0)

    def test_restore_helper_requires_stopped_arm_threads(self):
        logger = _Logger()
        restore = _load_restore_helper(logger)
        switcher = _Switcher((0, {"name": "ai"}))
        self.assertFalse(restore(switcher, "G1_29", False, False, True, False))
        self.assertEqual(switcher.calls, 0)
        self.assertTrue(logger.error_messages)

    def test_restore_helper_logs_success_only_for_verified_ai(self):
        logger = _Logger()
        restore = _load_restore_helper(logger)
        success = _Switcher((0, {"name": "ai"}))
        failure = _Switcher((0, {"name": "other"}))

        self.assertTrue(restore(success, "G1_29", False, False, True, True))
        self.assertFalse(restore(failure, "G1_29", False, False, True, True))

        self.assertEqual(success.calls, 1)
        self.assertEqual(failure.calls, 1)
        self.assertEqual(len(logger.info_messages), 1)
        self.assertTrue(logger.error_messages)


if __name__ == "__main__":
    unittest.main()
