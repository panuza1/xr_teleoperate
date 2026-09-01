import ast
import base64
import json
import os
from pathlib import Path
import socket
import struct
import tempfile
import threading
import time
import unittest
from urllib.request import Request, urlopen

import web_ui


class ConfigurationTests(unittest.TestCase):
    def test_default_is_current_real_command(self):
        args = web_ui.build_teleop_args({})
        self.assertEqual(args, [
            "--arm", "G1_29",
            "--input-mode", "hand",
            "--motion",
            "--img-server-ip", "192.168.123.164",
            "--image-transport", "zmq",
        ])
        self.assertEqual(web_ui.format_command(args), """python teleop_hand_and_arm.py \\
  --arm G1_29 \\
  --input-mode hand \\
  --motion \\
  --img-server-ip 192.168.123.164 \\
  --image-transport zmq""")

    def test_all_current_cli_parameters_have_structured_controls(self):
        config = {
            "arm": "H2", "ee": "brainco", "input_mode": "controller",
            "motion": False, "display_mode": "pass-through", "frequency": 50,
            "img_server_ip": "127.0.0.1", "image_transport": "webrtc",
            "network_interface": "lo", "headless": True, "sim": True,
            "ipc": True, "affinity": True, "record": True,
            "task_dir": "/tmp/data", "task_name": "test", "task_goal": "goal",
            "task_desc": "description", "task_steps": "one; two",
            "extra_args": "--future-option value",
        }
        args = web_ui.build_teleop_args(config)
        for flag in web_ui.CLI_FLAGS:
            if flag != "--motion":
                self.assertIn(flag, args)
        self.assertNotIn("--motion", args)
        self.assertEqual(args[-2:], ["--future-option", "value"])

    def test_invalid_values_are_rejected(self):
        invalid = (
            {"arm": "G9"},
            {"img_server_ip": "robot.local"},
            {"frequency": 0},
            {"network_interface": "lo; reboot"},
            {"motion": "yes"},
            {"extra_args": "--arm H2"},
            {"extra_args": "positional"},
            {"unknown": True},
        )
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(web_ui.ConfigError):
                web_ui.build_teleop_args(config)

    def test_ui_flag_set_matches_real_cli_parser(self):
        tree = ast.parse((Path(__file__).parent / "teleop" / "teleop_hand_and_arm.py").read_text())
        flags = {
            node.args[0].value
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "add_argument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
            and node.args[0].value.startswith("--")
        }
        self.assertEqual(flags, web_ui.CLI_FLAGS)


class ProcessManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.temp_dir.cleanup()

    def script(self, source):
        path = Path(self.temp_dir.name) / "terminal_child.py"
        path.write_text(source)
        return path

    def wait_for(self, manager, text, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            output = b"".join(entry["data"] for entry in manager.chunks()).decode(errors="replace")
            if text in output:
                return output
            time.sleep(.02)
        self.fail(f"did not receive terminal output: {text!r}")

    def test_pty_streams_output_and_forwards_r_and_q(self):
        manager = web_ui.ProcessManager(self.script("""
import sys
from sshkeyboard import listen_keyboard, stop_listening
print("STDOUT ready", flush=True)
print("STDERR ready", file=sys.stderr, flush=True)
def pressed(key):
    print(f"KEY:{key}", flush=True)
    if key == "q":
        stop_listening()
listen_keyboard(on_press=pressed, until=None, sequential=True)
"""))
        manager.start([])
        self.wait_for(manager, "STDERR ready")
        with self.assertRaisesRegex(RuntimeError, "already running"):
            manager.start([])
        manager.write_input("r")
        self.wait_for(manager, "KEY:r")
        manager.write_input("q")
        self.wait_for(manager, "KEY:q")
        deadline = time.monotonic() + 3
        while manager.status()["running"] and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertEqual(manager.status()["exit_code"], 0)
        with self.assertRaisesRegex(RuntimeError, "not running"):
            manager.write_input("r")
        output = b"".join(entry["data"] for entry in manager.chunks()).decode(errors="replace")
        self.assertIn("STDOUT ready", output)
        self.assertIn("STDERR ready", output)
        manager.clear_output()
        self.assertEqual(manager.chunks(), [])

    def test_input_is_rejected_before_start(self):
        with self.assertRaisesRegex(RuntimeError, "not running"):
            web_ui.ProcessManager().write_input("q")

    def test_graceful_stop_sends_sigint_to_pty_process(self):
        manager = web_ui.ProcessManager(self.script("""
import signal, time
def stop(_signum, _frame):
    print("CLEANUP complete", flush=True)
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
print("READY", flush=True)
while True:
    time.sleep(.1)
"""))
        manager.start([])
        self.wait_for(manager, "READY")
        status = manager.stop(timeout=2)
        self.wait_for(manager, "CLEANUP complete")
        self.assertFalse(status["running"])
        self.assertEqual(status["exit_code"], 0)


class WebSocketTerminalTests(unittest.TestCase):
    @staticmethod
    def recv_exact(connection, length):
        data = bytearray()
        while len(data) < length:
            data.extend(connection.recv(length - len(data)))
        return bytes(data)

    def receive_frame(self, connection):
        first, second = self.recv_exact(connection, 2)
        length = second & 0x7f
        if length == 126:
            length = struct.unpack("!H", self.recv_exact(connection, 2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self.recv_exact(connection, 8))[0]
        self.assertFalse(second & 0x80)
        return first & 0x0f, self.recv_exact(connection, length)

    @staticmethod
    def send_text(connection, value):
        payload = json.dumps(value).encode()
        mask = os.urandom(4)
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        header = bytes((0x81, 0x80 | len(payload)))
        connection.sendall(header + mask + masked)

    def receive_until(self, connection, expected, timeout=3):
        connection.settimeout(timeout)
        output = bytearray()
        while expected not in output:
            opcode, payload = self.receive_frame(connection)
            if opcode == 2:
                output.extend(payload)
        return bytes(output)

    def test_websocket_preserves_raw_pty_output_and_forwards_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            script = Path(directory) / "terminal_child.py"
            script.write_text("""
import sys
from sshkeyboard import listen_keyboard, stop_listening
sys.stdout.write("\\x1b[31mRED\\x1b[0m  aligned  🚀\\n")
sys.stdout.flush()
print("STDERR same stream", file=sys.stderr, flush=True)
def pressed(key):
    print(f"KEY:{key}", flush=True)
    if key == "q":
        stop_listening()
listen_keyboard(on_press=pressed, until=None, sequential=True)
""")
            manager = web_ui.ProcessManager(script)
            server = web_ui.make_server(port=0, manager=manager)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            connection = None
            try:
                manager.start([])
                connection = socket.create_connection(server.server_address, timeout=3)
                key = base64.b64encode(os.urandom(16)).decode()
                connection.sendall((
                    "GET /ws/terminal HTTP/1.1\r\n"
                    f"Host: 127.0.0.1:{server.server_address[1]}\r\n"
                    f"Origin: http://127.0.0.1:{server.server_address[1]}\r\n"
                    "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                    f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
                ).encode())
                response = bytearray()
                while b"\r\n\r\n" not in response:
                    response.extend(connection.recv(1))
                self.assertIn(b"101 Switching Protocols", response)
                output = self.receive_until(connection, b"STDERR same stream")
                self.assertIn(b"\x1b[31mRED\x1b[0m  aligned  \xf0\x9f\x9a\x80", output)
                self.send_text(connection, {"type": "input", "data": "r"})
                self.assertIn(b"KEY:r", self.receive_until(connection, b"KEY:r"))
                self.send_text(connection, {"type": "resize", "columns": 100, "rows": 30})
                self.send_text(connection, {"type": "input", "data": "q"})
                self.assertIn(b"KEY:q", self.receive_until(connection, b"KEY:q"))
                deadline = time.monotonic() + 3
                while manager.status()["running"] and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertEqual(manager.status()["exit_code"], 0)
                self.send_text(connection, {"type": "input", "data": "r"})
                while True:
                    opcode, payload = self.receive_frame(connection)
                    if opcode == 1:
                        self.assertIn("not running", json.loads(payload)["message"])
                        break
            finally:
                if connection is not None:
                    connection.close()
                manager.stop(timeout=.2)
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)


class StaticUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = web_ui.make_server(port=0)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f"http://127.0.0.1:{cls.server.server_address[1]}"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def test_root_and_static_assets_load(self):
        for path, marker in (
            ("/", "Session configuration"),
            ("/static/style.css", 'data-theme="mono"'),
            ("/static/app.js", "xrTeleopTheme"),
            ("/static/vendor/xterm.js", "Terminal"),
            ("/static/vendor/xterm.css", ".xterm"),
            ("/static/vendor/addon-fit.js", "FitAddon"),
        ):
            with self.subTest(path=path), urlopen(self.base + path) as response:
                self.assertEqual(response.status, 200)
                self.assertIn(marker, response.read().decode())

    def test_preview_uses_server_side_launch_builder(self):
        request = Request(
            self.base + "/api/preview",
            data=json.dumps({}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request) as response:
            data = json.load(response)
        self.assertEqual(data["args"], web_ui.build_teleop_args({}))
        self.assertEqual(data["command"], web_ui.format_command(data["args"]))

    def test_theme_persistence_and_motion_ui_without_body_tracking(self):
        html = (web_ui.STATIC_DIR / "index.html").read_text()
        css = (web_ui.STATIC_DIR / "style.css").read_text()
        js = (web_ui.STATIC_DIR / "app.js").read_text()
        for theme in ("light", "mono", "dark"):
            self.assertIn(theme, html + css + js)
        self.assertIn("localStorage.setItem(\"xrTeleopTheme\"", js)
        self.assertIn("localStorage.setItem(\"xrTeleopConfig\"", js)
        self.assertIn('id="motion"', html)
        self.assertNotIn("body tracking", html.lower())

    def test_interactive_terminal_is_focused_resizable_and_pty_backed(self):
        html = (web_ui.STATIC_DIR / "index.html").read_text()
        css = (web_ui.STATIC_DIR / "style.css").read_text()
        js = (web_ui.STATIC_DIR / "app.js").read_text()
        source = Path(web_ui.__file__).read_text()
        for marker in ('id="terminalSurface" tabindex="0"', 'id="terminalResize"', 'id="terminalMaximize"', 'id="terminalLarger"', 'id="terminalSmaller"'):
            self.assertIn(marker, html)
        self.assertIn("height:400px", css)
        self.assertIn("new Terminal(", js)
        self.assertIn("new WebSocket(", js)
        self.assertIn("terminal.write(data", js)
        self.assertIn("terminal.onData(data", js)
        self.assertNotIn('document.addEventListener("keydown"', js)
        self.assertNotIn("terminalKey", js)
        self.assertNotIn(".replace(/\\x1b", js)
        self.assertIn("/static/vendor/xterm.js", html)
        self.assertIn("/static/vendor/addon-fit.js", html)
        self.assertIn("pty.fork()", source)
        self.assertIn('parsed.path == "/ws/terminal"', source)
        self.assertIn("os.execv", source)
        self.assertNotIn("shell=True", source)

    def test_reference_visual_direction_is_preserved(self):
        html = (web_ui.STATIC_DIR / "index.html").read_text()
        css = (web_ui.STATIC_DIR / "style.css").read_text()
        for section in ("config", "camera", "robot", "recordings", "diagnostics"):
            self.assertIn(f'data-target="{section}"', html)
        for fragment in (
            "grid-template-columns:210px", "380px", "max-width:640px",
            "--accent:#0e9488", "--accent:#111111", "--accent:#56d6c8",
            "@media(max-width:1100px)", "@media(max-width:620px)",
        ):
            self.assertIn(fragment, css)
        self.assertNotIn("gradient", css.lower())


if __name__ == "__main__":
    unittest.main()
