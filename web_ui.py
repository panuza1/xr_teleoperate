#!/usr/bin/env python3
"""Local, hardware-passive control UI for the existing teleop entry point."""

from __future__ import annotations

import argparse
import base64
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import pty
import re
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

DEFAULT_CONFIG = {
    "arm": "G1_29",
    "ee": "",
    "input_mode": "hand",
    "motion": True,
    "display_mode": "immersive",
    "img_server_ip": "192.168.123.164",
    "image_transport": "zmq",
    "frequency": 30.0,
    "network_interface": "",
    "headless": False,
    "sim": False,
    "ipc": False,
    "affinity": False,
    "record": False,
    "task_dir": "./utils/data/",
    "task_name": "pick cube",
    "task_goal": "pick up cube.",
    "task_desc": "task description",
    "task_steps": "step1: do this; step2: do that;",
    "extra_args": "",
}

CLI_FLAGS = frozenset({
    "--frequency", "--input-mode", "--display-mode", "--arm", "--ee",
    "--img-server-ip", "--image-transport", "--network-interface", "--motion",
    "--headless", "--sim", "--ipc", "--affinity", "--record", "--task-dir",
    "--task-name", "--task-goal", "--task-desc", "--task-steps",
})
BOOLEAN_FLAGS = frozenset({"--motion", "--headless", "--sim", "--ipc", "--affinity", "--record"})
CHOICES = {
    "arm": {"G1_29", "G1_23", "H1_2", "H1", "H2"},
    "ee": {"", "dex1", "dex3", "inspire_ftp", "inspire_dfx", "brainco"},
    "input_mode": {"hand", "controller"},
    "display_mode": {"immersive", "ego", "pass-through"},
    "image_transport": {"auto", "webrtc", "zmq"},
}
TEXT_LIMITS = {
    "task_dir": 512,
    "task_name": 160,
    "task_goal": 500,
    "task_desc": 1000,
    "task_steps": 2000,
}
BOOL_FIELDS = {"motion", "headless", "sim", "ipc", "affinity", "record"}
INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.:-]{1,64}$")


class ConfigError(ValueError):
    pass


def _clean_text(name: str, value: object, limit: int) -> str:
    if not isinstance(value, str):
        raise ConfigError(f"{name} must be text")
    if len(value) > limit or any(ord(char) < 32 and char not in "\t" for char in value):
        raise ConfigError(f"{name} contains invalid text")
    return value


def normalize_config(raw: object) -> dict:
    if not isinstance(raw, dict):
        raise ConfigError("configuration must be a JSON object")
    unknown = set(raw) - set(DEFAULT_CONFIG)
    if unknown:
        raise ConfigError(f"unknown configuration field: {sorted(unknown)[0]}")

    config = DEFAULT_CONFIG | raw
    for name, allowed in CHOICES.items():
        if config[name] not in allowed:
            raise ConfigError(f"invalid {name}")
    for name in BOOL_FIELDS:
        if not isinstance(config[name], bool):
            raise ConfigError(f"{name} must be true or false")

    try:
        frequency = float(config["frequency"])
    except (TypeError, ValueError) as exc:
        raise ConfigError("frequency must be a number") from exc
    if not 1 <= frequency <= 240:
        raise ConfigError("frequency must be between 1 and 240 Hz")
    config["frequency"] = frequency

    try:
        config["img_server_ip"] = str(ipaddress.ip_address(config["img_server_ip"]))
    except (TypeError, ValueError) as exc:
        raise ConfigError("image server IP must be a valid IPv4 or IPv6 address") from exc

    interface = config["network_interface"]
    if not isinstance(interface, str) or (interface and not INTERFACE_RE.fullmatch(interface)):
        raise ConfigError("network interface contains invalid characters")
    for name, limit in TEXT_LIMITS.items():
        config[name] = _clean_text(name, config[name], limit)

    extra = _clean_text("extra_args", config["extra_args"], 512).strip()
    try:
        extra_tokens = shlex.split(extra)
    except ValueError as exc:
        raise ConfigError(f"invalid extra CLI arguments: {exc}") from exc
    if len(extra_tokens) > 32 or (extra_tokens and not extra_tokens[0].startswith("--")):
        raise ConfigError("extra CLI arguments must start with an option and contain at most 32 tokens")
    if "--" in extra_tokens or CLI_FLAGS.intersection(extra_tokens):
        raise ConfigError("known CLI options must use their dedicated controls")
    config["extra_tokens"] = extra_tokens
    return config


def build_teleop_args(raw: object) -> list[str]:
    config = normalize_config(raw)
    args = ["--arm", config["arm"]]
    if config["ee"]:
        args += ["--ee", config["ee"]]
    args += ["--input-mode", config["input_mode"]]
    if config["display_mode"] != DEFAULT_CONFIG["display_mode"]:
        args += ["--display-mode", config["display_mode"]]
    if config["frequency"] != DEFAULT_CONFIG["frequency"]:
        args += ["--frequency", str(config["frequency"])]
    if config["motion"]:
        args.append("--motion")
    args += [
        "--img-server-ip", config["img_server_ip"],
        "--image-transport", config["image_transport"],
    ]
    if config["network_interface"]:
        args += ["--network-interface", config["network_interface"]]
    for name in ("headless", "sim", "ipc", "affinity", "record"):
        if config[name]:
            args.append(f"--{name}")
    for name in ("task_dir", "task_name", "task_goal", "task_desc", "task_steps"):
        if config[name] != DEFAULT_CONFIG[name]:
            args += [f"--{name.replace('_', '-')}", config[name]]
    return args + config["extra_tokens"]


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
                    host = str(ipaddress.ip_address(query["img_server_ip"][0]))
                    transport = query.get("image_transport", ["zmq"])[0]
                    if transport not in CHOICES["image_transport"]:
                        raise ValueError
                    image_state = probe_image_server(host, transport)
                except ValueError:
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
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
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
