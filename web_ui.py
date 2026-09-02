#!/usr/bin/env python3
"""Local, hardware-passive control UI for the existing teleop entry point."""

from __future__ import annotations

import argparse
import ast
import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pty
import shlex
import signal
import select
import socket
import struct
import sys
import termios
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parent
TELEOP_SCRIPT = ROOT / "teleop" / "teleop_hand_and_arm.py"
STATIC_DIR = ROOT / "web_ui"

UI_META = {
    "arm": ("Robot", "basic", "Arm"),
    "ee": ("Robot", "basic", "End effector"),
    "input_mode": ("XR / Input", "basic", "Input mode"),
    "display_mode": ("XR / Input", "advanced", "Display mode"),
    "motion": ("Motion", "basic", "Motion"),
    "img_server_ip": ("Vision / Streaming", "basic", "Image server IP"),
    "image_transport": ("Vision / Streaming", "basic", "Image transport"),
    "network_interface": ("Network", "basic", "Network interface"),
    "sim": ("Simulation", "advanced", "Simulation"),
    "frequency": ("Runtime", "advanced", "Frequency"),
    "headless": ("Runtime", "advanced", "Headless"),
    "ipc": ("Runtime", "advanced", "IPC input"),
    "affinity": ("Runtime", "advanced", "CPU affinity"),
    "record": ("Recording", "advanced", "Record session"),
    "task_dir": ("Recording", "advanced", "Task directory"),
    "task_name": ("Recording", "advanced", "Task name"),
    "task_goal": ("Recording", "advanced", "Task goal"),
    "task_desc": ("Recording", "advanced", "Task description"),
    "task_steps": ("Recording", "advanced", "Task steps"),
}
BASELINE_ORDER = ("arm", "input_mode", "motion", "img_server_ip", "image_transport")


def load_cli_schema(path: Path = TELEOP_SCRIPT) -> list[dict]:
    """Extract literal argparse metadata without importing hardware modules."""
    tree = ast.parse(path.read_text())
    found = []
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "add_argument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, str)
            and node.args[0].value.startswith("--")
        ):
            continue
        flag = node.args[0].value
        keywords = {item.arg: item.value for item in node.keywords if item.arg}

        def literal(name, default=None):
            try:
                return ast.literal_eval(keywords[name])
            except (KeyError, TypeError, ValueError, SyntaxError):
                return default

        action = literal("action")
        type_node = keywords.get("type")
        value_type = type_node.id if isinstance(type_node, ast.Name) else "str"
        dest = literal("dest", flag[2:].replace("-", "_"))
        default = literal("default", False if action == "store_true" else None)
        group, level, label = UI_META.get(dest, ("Advanced", "advanced", dest.replace("_", " ").title()))
        found.append((node.lineno, {
            "flag": flag,
            "dest": dest,
            "type": "bool" if action == "store_true" else value_type,
            "default": default,
            "choices": literal("choices"),
            "required": bool(literal("required", False)),
            "action": action,
            "help": literal("help", ""),
            "group": group,
            "level": level,
            "label": label,
        }))
    return [spec for _, spec in sorted(found)]


CLI_SCHEMA = load_cli_schema()
SCHEMA_BY_DEST = {spec["dest"]: spec for spec in CLI_SCHEMA}
CLI_FLAGS = frozenset(spec["flag"] for spec in CLI_SCHEMA)
BOOLEAN_FLAGS = frozenset(spec["flag"] for spec in CLI_SCHEMA if spec["action"] == "store_true")
CLI_DEFAULTS = {spec["dest"]: spec["default"] for spec in CLI_SCHEMA}
BASELINE_VALUES = CLI_DEFAULTS | {
    "arm": "G1_29",
    "input_mode": "hand",
    "motion": True,
    "img_server_ip": "192.168.123.164",
    "image_transport": "zmq",
}
BASELINE_INCLUDED = list(BASELINE_ORDER)
PRESETS = {
    "baseline": {"label": "G1 + hand + motion + ZMQ", "values": BASELINE_VALUES, "included": BASELINE_INCLUDED},
    "inspire": {"label": "G1 + Inspire", "values": BASELINE_VALUES | {"ee": "inspire_dfx"}, "included": BASELINE_INCLUDED + ["ee"]},
    "simulation": {"label": "Simulation", "values": CLI_DEFAULTS | {"arm": "G1_29", "input_mode": "hand", "sim": True}, "included": ["arm", "input_mode", "sim"]},
}


class ConfigError(ValueError):
    def __init__(self, message: str, field_errors: dict | None = None):
        super().__init__(message)
        self.field_errors = field_errors or {}


def _clean_text(name: str, value: object, limit: int = 4096) -> str:
    if not isinstance(value, str):
        raise ConfigError(f"{name} must be text")
    if len(value) > limit or any(ord(char) < 32 and char not in "\t" for char in value):
        raise ConfigError(f"{name} contains invalid text")
    return value


def _same_value(value: object, default: object, value_type: str) -> bool:
    if default is None and value == "":
        return True
    if value_type == "float":
        try:
            return float(value) == float(default)
        except (TypeError, ValueError):
            return False
    return value == default


def normalize_config(raw: object) -> tuple[dict, set, list[str]]:
    if not isinstance(raw, dict):
        raise ConfigError("configuration must be a JSON object")
    if "values" in raw:
        values = raw.get("values")
        included_raw = raw.get("included", [])
        extra = raw.get("extra_args", "")
    else:  # compatibility for direct callers: start from the requested baseline preset
        values = BASELINE_VALUES | raw
        included_raw = list(BASELINE_INCLUDED) + [name for name in raw if name != "extra_args"]
        extra = values.pop("extra_args", "")
    if not isinstance(values, dict) or not isinstance(included_raw, list) or not all(isinstance(name, str) for name in included_raw):
        raise ConfigError("values and included parameters must be valid")
    included = set(included_raw)
    unknown = set(values) - set(SCHEMA_BY_DEST)
    errors = {name: "Unknown CLI parameter." for name in unknown}
    errors.update({name: "Unknown CLI parameter." for name in included - set(SCHEMA_BY_DEST)})
    normalized = {}
    for spec in CLI_SCHEMA:
        name = spec["dest"]
        value = values.get(name, spec["default"])
        if spec["action"] == "store_true":
            if not isinstance(value, bool):
                errors[name] = "Expected an on/off value."
            else:
                normalized[name] = value
            continue
        if value is None:
            value = ""
        if not isinstance(value, (str, int, float)):
            errors[name] = f"Expected {spec['type']} value."
            continue
        value = str(value)
        if spec["required"] and not value:
            errors[name] = "This parameter is required by the CLI."
        elif not value and spec["default"] is None and name not in included:
            pass
        elif spec["choices"] and value not in spec["choices"]:
            errors[name] = "Choose one of: " + ", ".join(map(str, spec["choices"])) + "."
        elif spec["type"] == "float":
            try:
                float(value)
            except ValueError:
                errors[name] = "Expected a number."
        else:
            try:
                _clean_text(name, value)
            except ConfigError as exc:
                errors[name] = str(exc)
        normalized[name] = value

    extra = _clean_text("extra_args", extra, 1024).strip()
    try:
        extra_tokens = shlex.split(extra)
    except ValueError as exc:
        errors["extra_args"] = f"Invalid quoting: {exc}"
        extra_tokens = []
    if len(extra_tokens) > 64 or (extra_tokens and not extra_tokens[0].startswith("--")):
        errors["extra_args"] = "Start with an option and use at most 64 tokens."
    if "--" in extra_tokens or any(token.split("=", 1)[0] in CLI_FLAGS for token in extra_tokens):
        errors["extra_args"] = "Known CLI options must use their dedicated controls."
    if errors:
        raise ConfigError("Fix the highlighted parameters.", errors)
    return normalized, included, extra_tokens


def build_teleop_args(raw: object) -> list[str]:
    values, included, extra_tokens = normalize_config(raw)
    ordered_names = list(BASELINE_ORDER) + [spec["dest"] for spec in CLI_SCHEMA if spec["dest"] not in BASELINE_ORDER]
    args = []
    for name in ordered_names:
        spec = SCHEMA_BY_DEST[name]
        value = values[name]
        if spec["action"] == "store_true":
            if value:
                args.append(spec["flag"])
        elif name in included or spec["required"] or not _same_value(value, spec["default"], spec["type"]):
            args += [spec["flag"], value]
    return args + extra_tokens


def format_command(args: list[str]) -> str:
    parts = []
    index = 0
    while index < len(args):
        flag = args[index]
        if flag in BOOLEAN_FLAGS or index + 1 == len(args) or args[index + 1].startswith("--"):
            parts.append(shlex.quote(flag))
            index += 1
        else:
            parts.append(f"{shlex.quote(flag)} {shlex.quote(args[index + 1])}")
            index += 2
    return "python teleop_hand_and_arm.py" + (" \\\n  " + " \\\n  ".join(parts) if parts else "")


class ProcessManager:
    def __init__(self, script: Path = TELEOP_SCRIPT):
        self.script = Path(script)
        self._pid: int | None = None
        self._master_fd: int | None = None
        self._exit_code: int | None = None
        self._logs: list[dict] = []
        self._sequence = 0
        self._lock = threading.RLock()

    def _append_output(self, data: bytes) -> None:
        with self._lock:
            self._sequence += 1
            self._logs.append({"id": self._sequence, "data": data})
            del self._logs[:-5000]

    def _read_pty(self, master_fd: int) -> None:
        try:
            while True:
                try:
                    data = os.read(master_fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                self._append_output(data)
        finally:
            try:
                os.close(master_fd)
            except OSError:
                pass

    def _poll_locked(self) -> None:
        if self._pid is None or self._exit_code is not None:
            return
        try:
            pid, status = os.waitpid(self._pid, os.WNOHANG)
        except ChildProcessError:
            pid, status = self._pid, 0
        if pid:
            self._exit_code = os.waitstatus_to_exitcode(status)

    def _wait(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                self._poll_locked()
                if self._exit_code is not None:
                    return True
            time.sleep(0.05)
        return False

    def start(self, args: list[str]) -> dict:
        with self._lock:
            self._poll_locked()
            if self._pid is not None and self._exit_code is None:
                raise RuntimeError("teleoperation is already running")
            command = [sys.executable, str(self.script), *args]
            pid, master_fd = pty.fork()
            if pid == 0:
                try:
                    os.chdir(self.script.parent)
                    os.execv(sys.executable, command)
                finally:
                    os._exit(127)
            self._pid = pid
            self._master_fd = master_fd
            self._exit_code = None
            self.resize(120, 40)
        threading.Thread(target=self._read_pty, args=(master_fd,), daemon=True).start()
        return self.status()

    def write_input(self, data: object) -> None:
        if not isinstance(data, str) or not data or len(data.encode()) > 4096:
            raise ConfigError("terminal input must contain 1 to 4096 bytes")
        with self._lock:
            self._poll_locked()
            if self._pid is None or self._exit_code is not None or self._master_fd is None:
                raise RuntimeError("teleoperation is not running")
            os.write(self._master_fd, data.encode())

    def resize(self, columns: object, rows: object) -> None:
        try:
            columns, rows = int(columns), int(rows)
        except (TypeError, ValueError) as exc:
            raise ConfigError("terminal size must be numeric") from exc
        if not 20 <= columns <= 400 or not 5 <= rows <= 200:
            raise ConfigError("terminal size is out of range")
        with self._lock:
            self._poll_locked()
            if self._pid is None or self._exit_code is not None or self._master_fd is None:
                raise RuntimeError("teleoperation is not running")
            fcntl.ioctl(self._master_fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, columns, 0, 0))

    def stop(self, timeout: float = 20.0) -> dict:
        with self._lock:
            self._poll_locked()
            if self._pid is None or self._exit_code is not None:
                return self.status()
            pid = self._pid
            try:
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        if not self._wait(timeout):
            try:
                os.killpg(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if not self._wait(5):
                try:
                    os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self._wait(5)
        return self.status()

    def status(self) -> dict:
        with self._lock:
            self._poll_locked()
            running = self._pid is not None and self._exit_code is None
            state = "running" if running else "error" if self._exit_code not in (None, 0) else "stopped"
            return {
                "state": state,
                "running": running,
                "pid": self._pid if running else None,
                "exit_code": self._exit_code,
            }

    def chunks(self, after: int = 0) -> list[dict]:
        with self._lock:
            return [entry.copy() for entry in self._logs if entry["id"] > after]

    def clear_output(self) -> None:
        with self._lock:
            self._logs.clear()


def _websocket_send(connection: socket.socket, payload: bytes, opcode: int = 2) -> None:
    length = len(payload)
    header = bytes((0x80 | opcode, length)) if length < 126 else (
        bytes((0x80 | opcode, 126)) + struct.pack("!H", length) if length < 65536
        else bytes((0x80 | opcode, 127)) + struct.pack("!Q", length)
    )
    connection.sendall(header + payload)


def _recv_exact(connection: socket.socket, length: int) -> bytes:
    data = bytearray()
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        if not chunk:
            raise ConnectionError("WebSocket closed")
        data.extend(chunk)
    return bytes(data)


def _websocket_receive(connection: socket.socket) -> tuple[int, bytes]:
    first, second = _recv_exact(connection, 2)
    opcode = first & 0x0F
    masked = second & 0x80
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _recv_exact(connection, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _recv_exact(connection, 8))[0]
    if not masked or length > 65536:
        raise ConnectionError("Invalid WebSocket frame")
    mask = _recv_exact(connection, 4)
    payload = _recv_exact(connection, length)
    return opcode, bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))


def probe_port(host: str, port: int) -> str:
    try:
        with socket.create_connection((host, port), timeout=0.2):
            return "connected"
    except OSError:
        return "disconnected"


def probe_image_server(host: str, transport: str) -> str:
    port = 60001 if transport == "webrtc" else 55555
    return probe_port(host, port)


class WebUIHandler(BaseHTTPRequestHandler):
    server_version = "XRUI/1.0"

    def log_message(self, _format: str, *_args) -> None:
        pass

    @property
    def manager(self) -> ProcessManager:
        return self.server.process_manager

    def _send(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, value: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        self._send(json.dumps(value).encode(), "application/json; charset=utf-8", status)

    def _read_json(self) -> object:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ConfigError("invalid content length") from exc
        if not 0 < length <= 65536:
            raise ConfigError("request body must be between 1 byte and 64 KiB")
        try:
            return json.loads(self.rfile.read(length))
        except json.JSONDecodeError as exc:
            raise ConfigError("invalid JSON") from exc

    def _serve_terminal_websocket(self) -> None:
        key = self.headers.get("Sec-WebSocket-Key")
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        if (
            self.headers.get("Upgrade", "").lower() != "websocket"
            or "upgrade" not in self.headers.get("Connection", "").lower()
            or self.headers.get("Sec-WebSocket-Version") != "13"
            or not key
            or origin not in {f"http://{host}", f"https://{host}"}
        ):
            self._json({"error": "invalid WebSocket upgrade"}, HTTPStatus.BAD_REQUEST)
            return
        accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
        self.send_response(HTTPStatus.SWITCHING_PROTOCOLS)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        last_id = 0
        try:
            while True:
                for chunk in self.manager.chunks(last_id):
                    _websocket_send(self.connection, chunk["data"])
                    last_id = chunk["id"]
                readable, _, _ = select.select((self.connection,), (), (), 0.02)
                if not readable:
                    continue
                opcode, payload = _websocket_receive(self.connection)
                if opcode == 8:
                    _websocket_send(self.connection, payload, 8)
                    break
                if opcode == 9:
                    _websocket_send(self.connection, payload, 10)
                    continue
                if opcode != 1:
                    raise ConnectionError("Unsupported WebSocket frame")
                try:
                    message = json.loads(payload)
                    if message.get("type") == "input":
                        self.manager.write_input(message.get("data"))
                    elif message.get("type") == "resize":
                        self.manager.resize(message.get("columns"), message.get("rows"))
                    else:
                        raise ConfigError("unknown terminal message")
                except (ConfigError, RuntimeError, json.JSONDecodeError) as exc:
                    _websocket_send(self.connection, json.dumps({"type": "error", "message": str(exc)}).encode(), 1)
        except (ConnectionError, OSError):
            pass
        finally:
            self.close_connection = True

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/ws/terminal":
            self._serve_terminal_websocket()
            return
        if parsed.path == "/api/status":
            status = self.manager.status()
            query = parse_qs(parsed.query)
            image_state = "idle"
            if "img_server_ip" in query:
                try:
                    host = _clean_text("img_server_ip", query["img_server_ip"][0], 253)
                    transport = query.get("image_transport", ["zmq"])[0]
                    if not host or transport not in SCHEMA_BY_DEST["image_transport"]["choices"]:
                        raise ValueError
                    image_state = probe_image_server(host, transport)
                except (ConfigError, ValueError):
                    image_state = "error"
            active_state = "checking" if status["running"] else "idle"
            ee = query.get("ee", [""])[0]
            self._json({
                **status,
                "components": {
                    "xr": probe_port("127.0.0.1", 8012) if status["running"] else "idle",
                    "image": image_state,
                    "g1_dds": active_state,
                    "inspire_dfx": active_state if ee == "inspire_dfx" else "idle",
                    "process": status["state"],
                },
            })
            return
        if parsed.path == "/api/schema":
            self._json({
                "parameters": CLI_SCHEMA,
                "defaults": CLI_DEFAULTS,
                "baseline": {"values": BASELINE_VALUES, "included": BASELINE_INCLUDED},
                "presets": PRESETS,
            })
            return
        files = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/static/style.css": ("style.css", "text/css; charset=utf-8"),
            "/static/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/static/vendor/xterm.js": ("vendor/xterm.js", "text/javascript; charset=utf-8"),
            "/static/vendor/xterm.css": ("vendor/xterm.css", "text/css; charset=utf-8"),
            "/static/vendor/addon-fit.js": ("vendor/addon-fit.js", "text/javascript; charset=utf-8"),
        }
        if parsed.path not in files:
            self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        name, content_type = files[parsed.path]
        try:
            self._send((STATIC_DIR / name).read_bytes(), content_type)
        except OSError:
            self._json({"error": "static asset unavailable"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_POST(self) -> None:
        try:
            payload = self._read_json()
            if self.path == "/api/preview":
                args = build_teleop_args(payload)
                self._json({"args": args, "command": format_command(args)})
            elif self.path == "/api/start":
                args = build_teleop_args(payload)
                self._json({**self.manager.start(args), "args": args, "command": format_command(args)})
            elif self.path == "/api/stop":
                self._json(self.manager.stop())
            elif self.path == "/api/terminal/clear":
                self.manager.clear_output()
                self._json({"ok": True})
            else:
                self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
        except ConfigError as exc:
            self._json({"error": str(exc), "field_errors": exc.field_errors}, HTTPStatus.BAD_REQUEST)
        except RuntimeError as exc:
            self._json({"error": str(exc)}, HTTPStatus.CONFLICT)
        except OSError as exc:
            self._json({"error": f"process error: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)


def make_server(host: str = "127.0.0.1", port: int = 8080, manager: ProcessManager | None = None):
    server = ThreadingHTTPServer((host, port), WebUIHandler)
    server.daemon_threads = True
    server.process_manager = manager or ProcessManager()
    return server


def main() -> None:
    parser = argparse.ArgumentParser(description="Local XR teleoperation control UI")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    server = make_server(port=args.port)

    def handle_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, handle_sigterm)
    print(f"XR Teleop Web UI: http://127.0.0.1:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.process_manager.stop()
        server.server_close()


if __name__ == "__main__":
    main()
