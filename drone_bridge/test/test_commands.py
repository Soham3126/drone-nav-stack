"""B0.4 command logic with a fake MAVLink peer: ACK accepted; ACK refused (returns STATUSTEXT);
no ACK (3 tries, then failure in <= 9 s); confirmation by HEARTBEAT."""
import pytest

from drone_bridge.commands import (COPTER_GUIDED, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_MODE,
                                   MAV_CMD_NAV_TAKEOFF, CommandRunner, command_spec)


class Msg:
    def __init__(self, type_, **kw):
        self._t = type_
        self.__dict__.update(kw)

    def get_type(self):
        return self._t


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakePeer:
    """Scripted flight controller. `script(cmd, params, attempt)` returns a list of (delay_s, Msg)."""

    def __init__(self, clock, script):
        self.clock = clock
        self.script = script
        self.sent = []
        self.inbox = []           # (time, Msg)

    def send_command(self, cmd, params):
        self.sent.append((self.clock(), cmd, list(params)))
        for delay, m in self.script(cmd, params, len(self.sent)):
            self.inbox.append((self.clock() + delay, m))
        self.inbox.sort(key=lambda x: x[0])

    def recv(self, timeout):
        if self.inbox and self.inbox[0][0] <= self.clock() + timeout:
            t, m = self.inbox.pop(0)
            self.clock.t = max(self.clock.t, t)
            return m
        self.clock.t += timeout            # nothing arrives: the wait times out
        return None


def ack(cmd, result=0):
    return Msg('COMMAND_ACK', command=cmd, result=result)


def hb(mode=0, armed=False):
    return Msg('HEARTBEAT', custom_mode=mode, base_mode=128 if armed else 0)


def make(script):
    clk = FakeClock()
    peer = FakePeer(clk, script)
    return CommandRunner(peer, clock=clk), peer, clk


def test_guided_accepted_and_confirmed():
    r, peer, _ = make(lambda c, p, n: [(0.1, ack(c)), (0.5, hb(COPTER_GUIDED))])
    ok, msg = r.run('guided')
    assert ok and 'confirmed' in msg
    assert peer.sent[0][1] == MAV_CMD_DO_SET_MODE and peer.sent[0][2][:2] == [1, 4]


def test_arm_refused_returns_statustext():
    def script(c, p, n):
        return [(0.05, Msg('STATUSTEXT', text='PreArm: Need Position Estimate')), (0.1, ack(c, result=4))]
    r, peer, _ = make(script)
    ok, msg = r.run('arm')
    assert not ok
    assert 'refused: FAILED' in msg and 'PreArm: Need Position Estimate' in msg
    assert len(peer.sent) == 1                       # a refusal is not retried
    assert peer.sent[0][1] == MAV_CMD_COMPONENT_ARM_DISARM


def test_no_ack_three_tries_then_failure_within_9s():
    r, peer, clk = make(lambda c, p, n: [])
    ok, msg = r.run('arm')
    assert not ok and 'no ACK' in msg
    assert len(peer.sent) == 3
    assert clk.t <= 9.0 + 1e-9
    assert [round(s[0], 6) for s in peer.sent] == [0.0, 3.0, 6.0]


def test_ack_on_second_try():
    r, peer, _ = make(lambda c, p, n: [] if n == 1 else [(0.2, ack(c))])
    ok, msg = r.run('takeoff', 1.1)
    assert ok and len(peer.sent) == 2
    assert peer.sent[1][1] == MAV_CMD_NAV_TAKEOFF and peer.sent[1][2][6] == pytest.approx(1.1)


def test_acks_for_other_commands_are_ignored():
    r, _, _ = make(lambda c, p, n: [(0.1, ack(999)), (0.2, ack(c)), (0.3, hb(armed=True))])
    ok, _ = r.run('arm')
    assert ok


def test_accepted_but_not_confirmed():
    r, _, clk = make(lambda c, p, n: [(0.1, ack(c)), (0.2, hb(mode=0))])
    ok, msg = r.run('guided')
    assert not ok and 'not confirmed' in msg


def test_statustext_during_confirmation_is_reported():
    r, _, _ = make(lambda c, p, n: [(0.1, ack(c)), (0.2, Msg('STATUSTEXT', text='Mode change')),
                                    (0.3, hb(9))])
    ok, msg = r.run('land')
    assert ok and 'Mode change' in msg


def test_on_message_sees_everything():
    seen = []
    clk = FakeClock()
    peer = FakePeer(clk, lambda c, p, n: [(0.1, hb()), (0.2, ack(c))])
    r = CommandRunner(peer, clock=clk, on_message=lambda m: seen.append(m.get_type()))
    r.run('takeoff', 1.0)
    assert seen == ['HEARTBEAT', 'COMMAND_ACK']


def test_bad_commands():
    with pytest.raises(ValueError):
        command_spec('fly_to_moon')
    with pytest.raises(ValueError):
        command_spec('takeoff', 0)
