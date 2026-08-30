import ast
import struct
import subprocess
import sys
import threading
import types
import unittest
import zlib
from pathlib import Path
from unittest.mock import patch

import numpy as np


# This is the last commit before experimental body tracking was introduced.
BASELINE_COMMIT = "c2479a3"
CYCLES = 5
MOTOR_COUNT = 35


class _MotorCommand:
    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.reserve = 0


class _LowCommand:
    def __init__(self):
        self.mode_pr = 0
        self.mode_machine = 0
        self.motor_cmd = [_MotorCommand() for _ in range(MOTOR_COUNT)]
        self.reserve = [0, 0, 0, 0]
        self.crc = 0


def _command_fields(message):
    return (
        int(message.mode_pr),
        int(message.mode_machine),
        tuple(
            (
                int(command.mode),
                float(command.q),
                float(command.dq),
                float(command.tau),
                float(command.kp),
                float(command.kd),
                int(command.reserve),
            )
            for command in message.motor_cmd
        ),
        tuple(int(value) for value in message.reserve),
        int(message.crc),
    )


def _command_bytes(message, include_crc=True):
    # Match the Unitree hg IDL schema and its 4-byte C alignment: two uint8
    # modes, 35 MotorCmd values (uint8, five float32, uint32), four uint32
    # reserves, then the uint32 CRC.
    payload = bytearray(
        struct.pack("<BB2x", int(message.mode_pr), int(message.mode_machine))
    )
    for command in message.motor_cmd:
        payload.extend(
            struct.pack(
                "<B3x5fI",
                int(command.mode),
                float(command.q),
                float(command.dq),
                float(command.tau),
                float(command.kp),
                float(command.kd),
                int(command.reserve),
            )
        )
    payload.extend(struct.pack("<4I", *(int(value) for value in message.reserve)))
    if include_crc:
        payload.extend(struct.pack("<I", int(message.crc)))
    return bytes(payload)


class _CRC:
    def Crc(self, message):
        return zlib.crc32(_command_bytes(message, include_crc=False))


class _Logger:
    def __getattr__(self, _name):
        return lambda *_args, **_kwargs: None


def _motor_state():
    return [
        types.SimpleNamespace(q=(index - 17) * 0.01, dq=(index + 1) * 0.001)
        for index in range(MOTOR_COUNT)
    ]


class _ChannelPublisher:
    def __init__(self, topic, _message_type):
        self.topic = topic

    def Init(self):
        pass

    def Write(self, _message):
        raise AssertionError("constructor publisher thread must not run in this test")


class _ChannelSubscriber:
    def __init__(self, topic, _message_type):
        self.topic = topic

    def Init(self):
        pass

    def Read(self):
        return types.SimpleNamespace(mode_machine=7, motor_state=_motor_state())


class _FakeThread:
    def __init__(self, target):
        self.target = target
        self.daemon = False

    def start(self):
        if self.target.__name__ != "_subscribe_motor_state":
            return
        controller = self.target.__self__
        state = types.SimpleNamespace(motor_state=_motor_state())
        controller.lowstate_buffer.SetData(state)


class _StopPublishing(Exception):
    pass


class _CapturePublisher:
    def __init__(self, controller, targets, torques):
        self.controller = controller
        self.targets = targets
        self.torques = torques
        self.fields = []
        self.payloads = []

    def Write(self, message):
        self.fields.append(_command_fields(message))
        self.payloads.append(_command_bytes(message))
        next_cycle = len(self.fields)
        if next_cycle == CYCLES:
            raise _StopPublishing
        self.controller.q_target = self.targets[next_cycle]
        self.controller.tauff_target = self.torques[next_cycle]


def _module_stubs():
    channel = types.ModuleType("unitree_sdk2py.core.channel")
    channel.ChannelPublisher = _ChannelPublisher
    channel.ChannelSubscriber = _ChannelSubscriber
    channel.ChannelFactoryInitialize = lambda *_args, **_kwargs: None

    hg_dds = types.ModuleType("unitree_sdk2py.idl.unitree_hg.msg.dds_")
    hg_dds.LowCmd_ = _LowCommand
    hg_dds.LowState_ = object

    go_dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
    go_dds.LowCmd_ = _LowCommand
    go_dds.LowState_ = object

    defaults = types.ModuleType("unitree_sdk2py.idl.default")
    defaults.unitree_hg_msg_dds__LowCmd_ = _LowCommand
    defaults.unitree_go_msg_dds__LowCmd_ = _LowCommand

    crc = types.ModuleType("unitree_sdk2py.utils.crc")
    crc.CRC = _CRC

    logging_mp = types.ModuleType("logging_mp")
    logging_mp.getLogger = lambda _name: _Logger()
    logging_mp.get_logger = logging_mp.getLogger

    module_names = [
        "unitree_sdk2py",
        "unitree_sdk2py.core",
        "unitree_sdk2py.idl",
        "unitree_sdk2py.idl.unitree_hg",
        "unitree_sdk2py.idl.unitree_hg.msg",
        "unitree_sdk2py.idl.unitree_go",
        "unitree_sdk2py.idl.unitree_go.msg",
        "unitree_sdk2py.utils",
    ]
    stubs = {name: types.ModuleType(name) for name in module_names}
    stubs.update(
        {
            channel.__name__: channel,
            hg_dds.__name__: hg_dds,
            go_dds.__name__: go_dds,
            defaults.__name__: defaults,
            crc.__name__: crc,
            logging_mp.__name__: logging_mp,
        }
    )
    return stubs


def _load_controller_module(name, source, filename):
    module = types.ModuleType(name)
    module.__file__ = str(filename)
    with patch.dict(sys.modules, _module_stubs()):
        exec(compile(source, str(filename), "exec"), module.__dict__)
    module.threading = types.SimpleNamespace(
        Event=threading.Event,
        Lock=threading.Lock,
        Thread=_FakeThread,
        current_thread=threading.current_thread,
    )
    return module


def _capture_arm_messages(module, motion_mode):
    controller = module.G1_29_ArmController(
        motion_mode=motion_mode,
        simulation_mode=False,
    )
    topic = controller.lowcmd_publisher.topic

    targets = [
        np.linspace(-0.03 + cycle * 0.01, 0.04 + cycle * 0.01, 14)
        for cycle in range(CYCLES)
    ]
    torques = [
        np.linspace(-0.7 + cycle * 0.1, 0.6 + cycle * 0.1, 14)
        for cycle in range(CYCLES)
    ]
    controller.q_target = targets[0]
    controller.tauff_target = torques[0]
    capture = _CapturePublisher(controller, targets, torques)
    controller.lowcmd_publisher = capture

    try:
        controller._ctrl_motor_state()
    except _StopPublishing:
        pass
    return topic, capture.fields, capture.payloads


class ArmOnlyMessageEquivalenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = Path(__file__).resolve().parents[2]
        current_path = cls.repo_root / "teleop" / "robot_control" / "robot_arm.py"
        current_source = current_path.read_text(encoding="utf-8")
        baseline_source = subprocess.check_output(
            [
                "git",
                "show",
                f"{BASELINE_COMMIT}:teleop/robot_control/robot_arm.py",
            ],
            cwd=str(cls.repo_root),
            text=True,
        )
        cls.current_source = current_source
        cls.baseline_source = baseline_source
        cls.current = _load_controller_module(
            "robot_arm_current", current_source, current_path
        )
        cls.baseline = _load_controller_module(
            "robot_arm_baseline",
            baseline_source,
            f"{BASELINE_COMMIT}:teleop/robot_control/robot_arm.py",
        )

    def test_g1_joint_layout_matches_known_working_baseline(self):
        def joint_layout_ast(source):
            tree = ast.parse(source)
            return next(
                node
                for node in tree.body
                if isinstance(node, ast.ClassDef)
                and node.name == "G1_29_JointIndex"
            )

        self.assertEqual(
            ast.dump(joint_layout_ast(self.baseline_source), include_attributes=False),
            ast.dump(joint_layout_ast(self.current_source), include_attributes=False),
        )

    def test_paused_body_tracking_flags_are_absent_from_entrypoint(self):
        entrypoint = (
            self.repo_root / "teleop" / "teleop_hand_and_arm.py"
        ).read_text(encoding="utf-8")
        argument_flags = {
            node.args[0].value
            for node in ast.walk(ast.parse(entrypoint))
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "add_argument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
        }
        self.assertTrue(
            {"--body-tracking", "--allow-real-waist", "--dry-run-waist"}.isdisjoint(
                argument_flags
            )
        )

    def test_arm_only_hg_lowcmd_is_byte_identical_for_several_cycles(self):
        for motion_mode in (False, True):
            with self.subTest(motion_mode=motion_mode):
                baseline = _capture_arm_messages(self.baseline, motion_mode)
                current = _capture_arm_messages(self.current, motion_mode)
                self.assertEqual(len(current[1]), CYCLES)
                self.assertEqual(baseline[0], current[0])
                self.assertEqual(baseline[1], current[1])
                self.assertEqual(baseline[2], current[2])


if __name__ == "__main__":
    unittest.main()
