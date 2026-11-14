"""MAVLink connection for the bridge: reader thread, reconnect, send lock (guidebook 2.9, test B1).
No ROS here, so it can be tested against a fake autopilot (test_mavlink_link.py).

It accepts messages from the autopilot only (system 1, component 1). If nothing arrives for `quiet_s`
seconds the connection is closed and reopened. While `capture` is set, every message is also copied
into `cmd_queue` for a running command (CommandRunner reads it through recv()).
"""
import os
import queue
import threading
import time

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402


class MavLink:
    """One MAVLink connection with a reader thread, reconnect and a send lock. Messages for a running
    command are copied into `cmd_queue` while `capture` is set."""

    def __init__(self, url, on_message, log, quiet_s=3.0):
        self.url = url
        self.on_message = on_message
        self.log = log
        self.quiet_s = quiet_s
        self.conn = None
        self.send_lock = threading.Lock()
        self.cmd_queue = queue.Queue()
        self.capture = False
        self.last_rx = 0.0
        self.connected = False          # True once a HEARTBEAT from the autopilot has been seen
        self.connect_count = 0
        self.stop = False
        self.fast_none = 0
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    # -- transport ---------------------------------------------------------------------------
    def _open(self):
        try:
            c = mavutil.mavlink_connection(self.url, source_system=1, source_component=191,
                                           autoreconnect=False, retries=0)
            c.WIRE_PROTOCOL_VERSION = '2.0'
            return c
        except Exception as e:  # noqa: BLE001
            self.log(f'connect {self.url} failed: {e}')
            return None

    def _close(self):
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:  # noqa: BLE001
                pass
        self.conn = None
        if self.connected:
            self.log('link lost')
        self.connected = False

    def _run(self):
        while not self.stop:
            if self.conn is None:
                self.conn = self._open()
                if self.conn is None:
                    time.sleep(0.5)
                    continue
                self.last_rx = time.monotonic()
            t_call = time.monotonic()
            try:
                m = self.conn.recv_match(blocking=True, timeout=0.2)
            except Exception as e:  # noqa: BLE001
                self.log(f'receive error: {e}')
                self._close()
                time.sleep(0.5)
                continue
            now = time.monotonic()
            if m is None:
                # a closed TCP socket returns "nothing" at once instead of after the timeout
                fast = now - t_call < 0.1
                self.fast_none = self.fast_none + 1 if fast else 0
                if now - self.last_rx > self.quiet_s or self.fast_none >= 5:
                    self._close()
                    time.sleep(0.5)
                continue
            self.fast_none = 0
            if m.get_type() == 'BAD_DATA':
                continue
            if m.get_srcSystem() != 1 or m.get_srcComponent() != 1:   # only the autopilot itself
                continue
            self.last_rx = now
            if m.get_type() == 'HEARTBEAT' and not self.connected:
                self.connected = True
                self.connect_count += 1
                self.log(f'autopilot heartbeat on {self.url} (connection {self.connect_count})')
            if self.capture:
                self.cmd_queue.put(m)
            self.on_message(m, now)
        self._close()

    def send(self, fn_name, *args):
        """Call conn.mav.<fn_name>(*args) safely; returns False when not connected."""
        c = self.conn
        if c is None:
            return False
        with self.send_lock:
            try:
                getattr(c.mav, fn_name)(*args)
                return True
            except Exception as e:  # noqa: BLE001
                self.log(f'send {fn_name} failed: {e}')
                return False

    # -- the interface CommandRunner expects -------------------------------------------------
    def send_command(self, command_id, params):
        self.send('command_long_send', 1, 1, command_id, 0, *[float(p) for p in params])

    def recv(self, timeout):
        try:
            return self.cmd_queue.get(timeout=max(0.0, timeout))
        except queue.Empty:
            return None
