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
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
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
        self.pub_path = self.create_publisher(Path, '/bridge/follow_path', 10)

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


    # ---------------------------------------------------------------- flying helpers
    def take_off(self, height=1.1, arm_limit=60):
        """guided + arm (retried while ArduPilot's own checks settle) + takeoff. Returns (ok, message)."""
        if not wait_for(lambda: self.last() is not None and self.last().fc_connected
                        and self.last().ekf_position_ok and self.last().origin_set, 180):
            return False, 'EKF never used our pose'
        t0 = time.time()
        while True:
            self.command('guided')
            ok, msg, _ = self.command('arm')
            if ok:
                break
            if time.time() - t0 > arm_limit:
                return False, f'arm: {msg}'
            time.sleep(5)
        ok, msg, _ = self.command('takeoff', height)
        return ok, f'takeoff: {msg}'

    def follow_async(self, points_map):
        """Publish a path (map frame, list of (x, y)) without waiting."""
        self.pub_path.publish(self._path_msg(points_map))

    def _path_msg(self, points_map):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points_map:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def follow(self, points_map, timeout):
        """Publish a path (map frame, list of (x, y)) and wait until the bridge reports it arrived.
        Returns (arrived, seconds, t_start, t_end) in wall time."""
        msg = self._path_msg(points_map)
        t0 = time.time()
        self.pub_path.publish(msg)
        wait_for(lambda: self.last() is not None and not self.last().path_arrived, 3, period=0.05)
        ok = wait_for(lambda: self.last().path_arrived, timeout, period=0.05)
        return ok, time.time() - t0, t0, time.time()

    def truth_between(self, t0, t1):
        with self.lock:
            return [p for p in self.truth if t0 <= p[0] <= t1]


# ------------------------------------------------------------------ geometry (world frame)
def load_meta(world):
    import os as _os
    pkg = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
    with open(_os.path.join(pkg, 'maps', world, 'meta.json')) as f:
        return json.load(f)


def dist_to_rects(x, y, rects):
    """Distance from (x, y) to the nearest wall rectangle [x0, y0, x1, y1] (0 inside one)."""
    best = float('inf')
    for x0, y0, x1, y1 in rects:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        best = min(best, (dx * dx + dy * dy) ** 0.5)
    return best


def dist_to_polyline(x, y, pts):
    """Distance from (x, y) to the polyline pts, and the index of the nearest segment."""
    best, seg = float('inf'), 0
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / L2))
        d = ((x - a[0] - t * dx) ** 2 + (y - a[1] - t * dy) ** 2) ** 0.5
        if d < best:
            best, seg = d, i
    return best, seg


def near_vertex(x, y, pts, r):
    """True if (x, y) is within r of an interior vertex (a corner) of the polyline."""
    return any(((x - p[0]) ** 2 + (y - p[1]) ** 2) ** 0.5 <= r for p in pts[1:-1])


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


class FcRecorder:
    """Background recorder of the flight controller's LOCAL_POSITION_NED and ATTITUDE yaw from SITL's
    spare port: rows (t_wall, n, e, d, yaw_ned). Use as a context manager."""

    def __init__(self, rate_hz=20):
        self.rate = rate_hz
        self.rows = []
        self.stop = False
        self.yaw = float('nan')

    def __enter__(self):
        self.th = threading.Thread(target=self.run, daemon=True)
        self.th.start()
        return self

    def __exit__(self, *exc):
        self.stop = True
        self.th.join(3)

    def run(self):
        c = spare_heartbeat(15)
        if c is None:
            return
        for mid in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
            c.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 1e6 / self.rate,
                                    0, 0, 0, 0, 0)
        t_hb = 0.0
        while not self.stop:
            if time.time() - t_hb > 1.0:
                gcs_heartbeat(c)
                t_hb = time.time()
            m = c.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE'], blocking=True, timeout=0.5)
            if m is None or m.get_srcComponent() != 1:
                continue
            if m.get_type() == 'ATTITUDE':
                self.yaw = m.yaw
            else:
                self.rows.append((time.time(), m.x, m.y, m.z, self.yaw))
        c.close()

    def between(self, t0, t1):
        return [r for r in self.rows if t0 <= r[0] <= t1]


def call_trigger(node, name, timeout=5.0):
    from std_srvs.srv import Trigger
    cli = node.create_client(Trigger, name)
    if not cli.wait_for_service(timeout_sec=timeout):
        return False, f'{name} not available'
    fut = cli.call_async(Trigger.Request())
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    if not fut.done():
        return False, 'no response'
    return fut.result().success, fut.result().message


def call_setbool(node, name, value, timeout=5.0):
    from std_srvs.srv import SetBool
    cli = node.create_client(SetBool, name)
    if not cli.wait_for_service(timeout_sec=timeout):
        return False, f'{name} not available'
    req = SetBool.Request()
    req.data = bool(value)
    fut = cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    if not fut.done():
        return False, 'no response'
    return fut.result().success, fut.result().message


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
