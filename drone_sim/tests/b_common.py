"""b_common.py - shared pieces for workstream B tests: a ROS client for the bridge (status, link stats,
commands), a direct MAVLink listener on SITL's spare port, SITL stop/restart through tmux, results."""
import json
import os
import subprocess
import threading
import time

os.environ.setdefault('MAVLINK20', '1')
import rclpy  # noqa: E402
from drone_interfaces.msg import BridgeStatus  # noqa: E402
from drone_interfaces.srv import FlightCommand  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

SPARE_URL = 'tcp:127.0.0.1:5763'      # SITL SERIAL2: free in truth/slam profiles (no scripted flight)


class BridgeClient(Node):
    def __init__(self):
        super().__init__('b_test_client')
        self.lock = threading.Lock()
        self.status = []          # (t_wall, BridgeStatus)
        self.stats = []           # (t_wall, dict)
        self.create_subscription(BridgeStatus, '/bridge/status', self.cb_status, 50)
        self.create_subscription(String, '/bridge/link_stats', self.cb_stats, 10)
        self.cli = self.create_client(FlightCommand, '/bridge/command')

    def cb_status(self, m):
        with self.lock:
            self.status.append((time.time(), m))

    def cb_stats(self, m):
        with self.lock:
            self.stats.append((time.time(), json.loads(m.data)))

    def last(self):
        with self.lock:
            return self.status[-1][1] if self.status else None

    def command(self, name, value=0.0, timeout=90.0):
        """Calls /bridge/command; returns (success, message, seconds)."""
        if not self.cli.wait_for_service(timeout_sec=10.0):
            return False, 'service /bridge/command not available', 0.0
        req = FlightCommand.Request()
        req.command, req.value = name, float(value)
        t0 = time.time()
        fut = self.cli.call_async(req)
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        if not fut.done():
            return False, f'no service response within {timeout:.0f} s', time.time() - t0
        r = fut.result()
        return r.success, r.message, time.time() - t0


class Client:
    def __enter__(self):
        rclpy.init()
        self.node = BridgeClient()
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.th = threading.Thread(target=self.ex.spin, daemon=True)
        self.th.start()
        return self.node

    def __exit__(self, *exc):
        try:
            self.ex.shutdown(timeout_sec=2.0)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


def wait_for(cond, timeout, period=0.2):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(period)
    return False


def spare_heartbeat(timeout=60.0):
    """Connect to SITL's spare port; returns the connection once an autopilot HEARTBEAT is seen, else None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            c = mavutil.mavlink_connection(SPARE_URL, source_system=250, source_component=190, retries=0)
            m = c.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            while m is not None and m.get_srcComponent() != 1:
                m = c.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if m is not None:
                return c
            c.close()
        except Exception:  # noqa: BLE001
            time.sleep(0.5)
    return None


def sitl_stop():
    """Ctrl+C in the SITL window (as the guidebook does), then wait for the window to close."""
    subprocess.run(['tmux', 'send-keys', '-t', 'sim:sitl', 'C-c'], check=False)
    for _ in range(40):
        out = subprocess.run(['tmux', 'list-windows', '-t', 'sim', '-F', '#W'], capture_output=True, text=True).stdout
        if 'sitl' not in out.split():
            return True
        time.sleep(0.5)
    subprocess.run(['tmux', 'kill-window', '-t', 'sim:sitl'], check=False)
    time.sleep(2)
    return False


def sitl_start(out_dir):
    """Restart ArduPilot alone, with the command sim_up.sh saved."""
    cmd = os.path.join(out_dir, 'sitl_cmd.sh')
    subprocess.run(['tmux', 'new-window', '-d', '-t', 'sim', '-n', 'sitl', f"bash -ic 'source {cmd}'"], check=True)


def save(out_dir, test_id, result, ok, lines):
    result['test'], result['pass'] = test_id, ok
    with open(os.path.join(out_dir, 'result.json'), 'w') as f:
        json.dump(result, f, indent=1, default=str)
    with open(os.path.join(out_dir, 'report.txt'), 'w') as f:
        f.write('\n'.join(lines) + f'\n{test_id}: {"PASS" if ok else "FAIL"}\n')
    for ln in lines:
        print(ln)
    print(f'{test_id}: {"PASS" if ok else "FAIL"}', flush=True)
