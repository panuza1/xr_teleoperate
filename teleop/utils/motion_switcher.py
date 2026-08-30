# for motion switcher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
# for loco client
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
import time

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
    def __init__(self):
        self.client = LocoClient()
        self.client.SetTimeout(0.0001)
        self.client.Init()

    def Damp(self):
        self.client.Damp()
    
    def Move(self, vx, vy, vyaw):
        self.client.Move(vx, vy, vyaw, continous_move=False)

if __name__ == '__main__':
    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation
    ms = MotionSwitcher()
    status, result = ms.Enter_Debug_Mode()
    print("Enter debug mode:", status, result)
    time.sleep(5)
    status, result = ms.Exit_Debug_Mode()
    print("Exit debug mode:", status, result)
    time.sleep(2)
