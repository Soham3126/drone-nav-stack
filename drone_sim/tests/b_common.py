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
from nav_msgs.msg import Odometry  # noqa: E402
from pymavlink import mavutil  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import String  # noqa: E402

SPARE_URL = 'tcp:127.0.0.1:5763'      # SITL SERIAL2: free in truth/slam profiles (no scripted flight)


class BridgeClient(Node):
    def __init__(self):
        super().__init__('b_test_client')
        self.lock = threading.Lock()
        self.status = []          # (t_wall, BridgeStatus)
        self.stats = []           # (t_wall, dict)
        self.truth_z = float('nan')   # Gazebo ground-truth height of the body (m, world frame)
        self.truth_z0 = None          # the same at the first sample (on the ground)
        self.truth = []               # (t_wall, x, y, z) ground truth, world frame
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_truth, qos_profile_sensor_data)
        self.create_subscription(BridgeStatus, '/bridge/status', self.cb_status, 50)
        self.create_subscription(String, '/bridge/link_stats', self.cb_stats, 10)
        self.cli = self.create_client(FlightCommand, '/bridge/command')

    def cb_status(self, m):
        with self.lock:
            self.status.append((time.time(), m))

    def cb_truth(self, m):
        p = m.pose.pose.position
        z = p.z
        with self.lock:
            self.truth.append((time.time(), p.x, p.y, p.z))
            if len(self.truth) > 200000:
                del self.truth[:100000]
        if self.truth_z0 is None:
            self.truth_z0 = z
        self.truth_z = z

    def truth_height(self):
        """Ground-truth height above the take-off point (m)."""
        return self.truth_z - self.truth_z0 if self.truth_z0 is not None else float('nan')

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
            self.th.join(timeout=3.0)
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


def set_param(name, value, timeout=10.0):
    """Set an ArduPilot parameter through the spare port and read it back. Returns the value read."""
    c = spare_heartbeat(30)
    if c is None:
        return None
    try:
        for _ in range(3):
            c.mav.param_set_send(1, 1, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            m = c.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout / 3)
            while m is not None and m.param_id.rstrip('\x00') != name:
                m = c.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout / 3)
            if m is not None:
                return m.param_value
        return None
    finally:
        c.close()


def gcs_heartbeat(conn):
    """ArduPilot forwards messages only to links it has heard from: say hello as a ground station."""
    try:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    except Exception:  # noqa: BLE001
        pass


def spare_heartbeat(timeout=60.0):
    """Connect to SITL's spare port; returns the connection once an autopilot HEARTBEAT is seen, else None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            c = mavutil.mavlink_connection(SPARE_URL, source_system=250, source_component=190, retries=0)
            gcs_heartbeat(c)
            m = c.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            while m is not None and m.get_srcComponent() != 1:
                m = c.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if m is not None:
                return c
            c.close()
        except Exception:  # noqa: BLE001
            time.sleep(0.5)
    return None


def _sitl_running():
    return subprocess.run(['pgrep', '-f', 'arducopter'], capture_output=True).returncode == 0


def sitl_stop():
    """Stop ArduPilot alone: Ctrl+C in its window (as the guidebook does). MAVProxy ignores Ctrl+C and
    the arducopter process outlives its tmux window, so then stop the processes themselves, as
    sim_down.sh does. Returns True when arducopter is gone."""
    subprocess.run(['tmux', 'send-keys', '-t', 'sim:sitl', 'C-c'], check=False)
    time.sleep(1)
    for pat in ('arducopter', 'mavproxy', 'sim_vehicle'):
        subprocess.run(['pkill', '-f', pat], check=False)
    subprocess.run(['tmux', 'kill-window', '-t', 'sim:sitl'], check=False)
    for _ in range(40):
        if not _sitl_running():
            time.sleep(1)                       # let the TCP ports close
            return True
        time.sleep(0.25)
    subprocess.run(['pkill', '-9', '-f', 'arducopter'], check=False)
    time.sleep(2)
    return not _sitl_running()


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


def finish(code):
    """Leave without Python's slow, crash-prone teardown of ROS threads (results are already saved)."""
    import sys
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
