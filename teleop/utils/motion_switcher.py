# for motion switcher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
# for loco client
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
import time
import math
import queue
import threading
import logging_mp

logger_mp = logging_mp.getLogger(__name__)
LOCO_RPC_TIMEOUT_DEFAULT = 0.1
LOCO_RPC_TIMEOUT_MAX = 0.5
LOCO_RPC_FAILURE_LIMIT = 3

# MotionSwitcher used to switch mode between debug mode and ai mode
class MotionSwitcher:
    def __init__(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(1.0)
        self.msc.Init()

    def Enter_Debug_Mode(self, max_attempts=10, retry_interval_s=1.0):
        try:
            for attempt in range(max_attempts + 1):
                status, result = self.msc.CheckMode()
                if status != 0 or not isinstance(result, dict):
                    return status, result
                if not result.get('name'):
                    return status, result
                if attempt == max_attempts:
                    return None, result
                status, _ = self.msc.ReleaseMode()
                if status != 0:
                    return status, result
                time.sleep(retry_interval_s)
        except Exception:
            return None, None
    
    def Exit_Debug_Mode(self, target_mode='ai', timeout_s=5.0, poll_interval_s=0.2):
        try:
            status, result = self.msc.SelectMode(nameOrAlias=target_mode)
            if status != 0:
                return status, result

            deadline = time.monotonic() + timeout_s
            while True:
                status, result = self.msc.CheckMode()
                if status == 0 and result and result.get('name') == target_mode:
                    return status, result
                if time.monotonic() >= deadline:
                    return (status if status != 0 else None), result
                time.sleep(min(poll_interval_s, max(0.0, deadline - time.monotonic())))
        except Exception:
            return None, None

class LocoClientWrapper:
    """Serialize bounded SDK RPCs off the XR control loop; only acked stops count."""
    def __init__(self, rpc_timeout_s=LOCO_RPC_TIMEOUT_DEFAULT, client=None):
        if not 0.01 <= rpc_timeout_s <= LOCO_RPC_TIMEOUT_MAX:
            raise ValueError(f"rpc_timeout_s must be between 0.01 and {LOCO_RPC_TIMEOUT_MAX}s")
        self.rpc_timeout_s = rpc_timeout_s
        self.client = client or LocoClient()
        self.client.SetTimeout(rpc_timeout_s)
        self.client.Init()
        self._commands = queue.Queue(maxsize=1)
        self._state_lock = threading.Lock()
        self._closed = False
        self._consecutive_failures = 0
        self._last_acknowledged = None
        self._worker = threading.Thread(target=self._run, name="g1-loco-rpc", daemon=True)
        self._worker.start()

    @property
    def last_acknowledged(self):
        with self._state_lock:
            return self._last_acknowledged

    @property
    def consecutive_failures(self):
        with self._state_lock:
            return self._consecutive_failures

    def _submit(self, kind, args, wait=False):
        task = {
            "kind": kind,
            "args": args,
            "event": threading.Event() if wait else None,
            "result": [False],
        }
        with self._state_lock:
            if self._closed:
                return False
            while True:
                try:
                    self._commands.put_nowait(task)
                    break
                except queue.Full:
                    try:
                        replaced = self._commands.get_nowait()
                    except queue.Empty:
                        continue
                    if replaced is not None and replaced["event"] is not None:
                        replaced["event"].set()
        if not wait:
            return True
        confirmed = task["event"].wait(self.rpc_timeout_s * 4 + 0.05)
        return confirmed and task["result"][0]

    def _run(self):
        while True:
            try:
                task = self._commands.get(timeout=0.05)
            except queue.Empty:
                with self._state_lock:
                    if self._closed:
                        return
                continue
            if task is None:
                return
            try:
                if task["kind"] == "velocity":
                    vx, vy, vyaw = task["args"]
                    status = self.client.SetVelocity(vx, vy, vyaw, duration=1.0)
                else:
                    status = self.client.SetFsmId(1)
                succeeded = status == 0
                error = None if succeeded else f"SDK status {status!r}"
            except Exception as exc:
                succeeded = False
                error = f"{type(exc).__name__}: {exc}"

            with self._state_lock:
                previous_failures = self._consecutive_failures
                self._consecutive_failures = 0 if succeeded else previous_failures + 1
                self._last_acknowledged = succeeded
                failure_count = self._consecutive_failures
            if succeeded and previous_failures:
                logger_mp.info("G1 locomotion RPC recovered.")
            elif not succeeded:
                is_stop = task["kind"] == "velocity" and task["args"] == (0.0, 0.0, 0.0)
                if failure_count == 1 or failure_count % LOCO_RPC_FAILURE_LIMIT == 0:
                    logger_mp.error(
                        f"G1 locomotion RPC failed ({error}); "
                        + ("zero velocity is not confirmed; use the physical E-stop."
                           if is_stop else "locomotion command is not confirmed.")
                    )
            task["result"][0] = succeeded
            if task["event"] is not None:
                task["event"].set()

    def Damp(self):
        # Damp is a locomotion mode request, not an emergency stop.
        return self._submit("damp", ())

    def Move(self, vx, vy, vyaw):
        values = tuple(float(value) for value in (vx, vy, vyaw))
        if not all(math.isfinite(value) for value in values):
            logger_mp.error("Rejected non-finite G1 locomotion command.")
            return False
        values = (
            max(-0.3, min(0.3, values[0])),
            max(-0.3, min(0.3, values[1])),
            max(-0.3, min(0.3, values[2])),
        )
        return self._submit("velocity", values)

    def stop_motion(self):
        """Request and await an acknowledged zero-velocity command, with a hard bound."""
        confirmed = self._submit("velocity", (0.0, 0.0, 0.0), wait=True)
        if not confirmed:
            logger_mp.error(
                "Could not confirm G1 locomotion stop; use the physical E-stop."
            )
        return confirmed

    def close(self):
        with self._state_lock:
            if self._closed:
                return not self._worker.is_alive()
            self._closed = True
            while True:
                try:
                    task = self._commands.get_nowait()
                except queue.Empty:
                    break
                if task is not None and task["event"] is not None:
                    task["event"].set()
            self._commands.put_nowait(None)
        self._worker.join(self.rpc_timeout_s * 2 + 0.1)
        return not self._worker.is_alive()

if __name__ == '__main__':
    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation
    ms = MotionSwitcher()
    status, result = ms.Enter_Debug_Mode()
    print("Enter debug mode:", status, result)
    time.sleep(5)
    status, result = ms.Exit_Debug_Mode()
    print("Exit debug mode:", status, result)
    time.sleep(2)
