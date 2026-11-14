"""MavLink connection against a fake autopilot over real TCP: heartbeat seen, our messages arrive with
system 1 / component 191, a command gets its ACK through CommandRunner, other senders are ignored, and the
link reconnects after the autopilot restarts (B1 step 4, here in under 5 s)."""
import os
import socket
import threading
import time

import pytest

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402

from drone_bridge.commands import CommandRunner  # noqa: E402
from drone_bridge.mavlink_link import MavLink  # noqa: E402

M = mavutil.mavlink


class FakeAutopilot:
    """TCP server speaking MAVLink 2 as system 1 / component 1. Sends HEARTBEAT at 10 Hz, answers
    COMMAND_LONG with an ACK (result from `results`), and records everything it receives."""

    def __init__(self, port=0, results=None):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('127.0.0.1', port))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.results = results or {}
        self.received = []
        self.stop = False
        self.conn = None
        self.th = threading.Thread(target=self.run, daemon=True)
        self.th.start()

    def run(self):
        self.srv.settimeout(0.2)
        while not self.stop:
            try:
                self.conn, _ = self.srv.accept()
                break
            except socket.timeout:
                continue
        if self.conn is None:
            return
        self.conn.settimeout(0.05)
        mav = M.MAVLink(self, srcSystem=1, srcComponent=1)
        other = M.MAVLink(self, srcSystem=1, srcComponent=50)     # someone else on the bus
        last_hb = 0
        while not self.stop:
            now = time.monotonic()
            if now - last_hb > 0.1:
                mav.heartbeat_send(M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 4, M.MAV_STATE_STANDBY)
                other.statustext_send(M.MAV_SEVERITY_INFO, b'not from the autopilot')
                last_hb = now
            try:
                data = self.conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            for m in mav.parse_buffer(data) or []:
                self.received.append(m)
                if m.get_type() == 'COMMAND_LONG':
                    res = self.results.get(m.command, 0)
                    if res is not None:
                        if res != 0:
                            mav.statustext_send(M.MAV_SEVERITY_CRITICAL, b'PreArm: test refusal')
                        mav.command_ack_send(m.command, res)
                    if res == 0 and m.command == M.MAV_CMD_COMPONENT_ARM_DISARM:
                        mav.heartbeat_send(M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_ARDUPILOTMEGA, 128, 4,
                                           M.MAV_STATE_ACTIVE)

    def write(self, buf):            # MAVLink(file=self) sends through here
        try:
            self.conn.sendall(buf)
        except OSError:
            pass

    def close(self):
        self.stop = True
        self.th.join(2)
        for s in (self.conn, self.srv):
            try:
                s.close()
            except Exception:  # noqa: BLE001
                pass


def wait(cond, t=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def ap():
    a = FakeAutopilot()
    yield a
    a.close()


def test_connect_send_and_filter(ap):
    seen = []
    link = MavLink(f'tcp:127.0.0.1:{ap.port}', lambda m, t: seen.append((m.get_type(), m.get_srcComponent())),
                   lambda s: None)
    try:
        assert wait(lambda: link.connected)
        assert all(c == 1 for _, c in seen)                     # component 50 is ignored
        assert link.send('heartbeat_send', M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, 4)
        link.send('vision_position_estimate_send', 123, 1.0, -2.0, -0.1, 0, 0, 0.5,
                  [float('nan')] + [0.0] * 20, 7)
        assert wait(lambda: any(m.get_type() == 'VISION_POSITION_ESTIMATE' for m in ap.received))
        hb = next(m for m in ap.received if m.get_type() == 'HEARTBEAT')
        assert (hb.get_srcSystem(), hb.get_srcComponent()) == (1, 191)
        vpe = next(m for m in ap.received if m.get_type() == 'VISION_POSITION_ESTIMATE')
        assert vpe.reset_counter == 7 and vpe.y == pytest.approx(-2.0)
    finally:
        link.stop = True


def test_command_through_runner(ap):
    link = MavLink(f'tcp:127.0.0.1:{ap.port}', lambda m, t: None, lambda s: None)
    try:
        assert wait(lambda: link.connected)
        link.capture = True
        ok, msg = CommandRunner(link).run('arm')
        assert ok, msg
    finally:
        link.stop = True


def test_refusal_text_through_runner():
    ap = FakeAutopilot(results={M.MAV_CMD_COMPONENT_ARM_DISARM: 4})
    link = MavLink(f'tcp:127.0.0.1:{ap.port}', lambda m, t: None, lambda s: None)
    try:
        assert wait(lambda: link.connected)
        link.capture = True
        ok, msg = CommandRunner(link).run('arm')
        assert not ok and 'PreArm: test refusal' in msg
    finally:
        link.stop = True
        ap.close()


def test_reconnect_after_autopilot_restart():
    ap = FakeAutopilot()
    port = ap.port
    link = MavLink(f'tcp:127.0.0.1:{port}', lambda m, t: None, lambda s: None, quiet_s=1.0)
    try:
        assert wait(lambda: link.connected)
        ap.close()                                              # autopilot goes away
        assert wait(lambda: not link.connected, 4)
        ap = FakeAutopilot(port=port)                           # and comes back on the same port
        t0 = time.monotonic()
        assert wait(lambda: link.connected and link.connect_count == 2, 8)
        assert time.monotonic() - t0 < 5.0
    finally:
        link.stop = True
        ap.close()


def test_silent_autopilot_counts_as_lost():
    """Socket still open but no HEARTBEAT: the link is reported lost within quiet_s and reopened."""
    ap = FakeAutopilot()
    link = MavLink(f'tcp:127.0.0.1:{ap.port}', lambda m, t: None, lambda s: None, quiet_s=1.0)
    try:
        assert wait(lambda: link.connected)
        ap.stop = True                      # stops sending (and serving) but does not close the socket at once
        t0 = time.monotonic()
        assert wait(lambda: not link.connected, 4)
        assert time.monotonic() - t0 < 2.5
    finally:
        link.stop = True
        ap.close()
