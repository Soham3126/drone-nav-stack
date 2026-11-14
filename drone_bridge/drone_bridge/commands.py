"""Flight-command logic (guidebook 2.9), independent of the transport. Tested by test_commands.py (B0)
with a fake MAVLink peer.

Each command waits up to 3 s for its COMMAND_ACK, retries twice (3 tries, so failure within 9 s), and
returns the result plus every STATUSTEXT received meanwhile. After an accepted ACK an optional
confirmation (e.g. HEARTBEAT custom mode 4 for 'guided') is waited for.

The link object must provide:
  send_command(command_id, params)   -> None     (params: 7 floats)
  recv(timeout)                      -> message or None
Messages look like pymavlink messages: msg.get_type(), and fields as attributes.
"""
import time

MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_TAKEOFF = 22
MAV_RESULT_ACCEPTED = 0
COPTER_GUIDED = 4
COPTER_LAND = 9
MAV_MODE_FLAG_SAFETY_ARMED = 128

RESULT_NAMES = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED', 4: 'FAILED',
                5: 'IN_PROGRESS', 6: 'CANCELLED'}


def command_spec(name, value=0.0):
    """name -> (command id, params, confirm predicate or None, confirm timeout s)."""
    if name == 'guided':
        return (MAV_CMD_DO_SET_MODE, [1, COPTER_GUIDED, 0, 0, 0, 0, 0],
                lambda m: m.get_type() == 'HEARTBEAT' and m.custom_mode == COPTER_GUIDED, 3.0)
    if name == 'arm':
        return (MAV_CMD_COMPONENT_ARM_DISARM, [1, 0, 0, 0, 0, 0, 0],
                lambda m: m.get_type() == 'HEARTBEAT' and bool(m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED), 3.0)
    if name == 'takeoff':
        if not value or value <= 0:
            raise ValueError('takeoff needs a height > 0')
        return (MAV_CMD_NAV_TAKEOFF, [0, 0, 0, 0, 0, 0, float(value)], None, 0.0)
    if name == 'land':
        return (MAV_CMD_DO_SET_MODE, [1, COPTER_LAND, 0, 0, 0, 0, 0],
                lambda m: m.get_type() == 'HEARTBEAT' and m.custom_mode == COPTER_LAND, 3.0)
    raise ValueError(f'unknown command {name!r}')


class CommandRunner:
    def __init__(self, link, ack_timeout=3.0, tries=3, clock=time.monotonic, on_message=None):
        self.link = link
        self.ack_timeout = ack_timeout
        self.tries = tries
        self.clock = clock
        self.on_message = on_message      # every message seen while waiting is also passed on

    def _recv(self, deadline):
        left = deadline - self.clock()
        if left <= 0:
            return None
        m = self.link.recv(left)
        if m is not None and self.on_message:
            self.on_message(m)
        return m

    def run(self, name, value=0.0):
        """Returns (success: bool, message: str)."""
        cmd, params, confirm, confirm_timeout = command_spec(name, value)
        texts = []
        last = 'no ACK'
        for attempt in range(1, self.tries + 1):
            self.link.send_command(cmd, params)
            deadline = self.clock() + self.ack_timeout
            ack = None
            while self.clock() < deadline:
                m = self._recv(deadline)
                if m is None:
                    continue
                t = m.get_type()
                if t == 'STATUSTEXT':
                    texts.append(str(m.text).strip())
                elif t == 'COMMAND_ACK' and m.command == cmd:
                    ack = m
                    break
            if ack is None:
                last = f'no ACK (try {attempt}/{self.tries})'
                continue
            if ack.result != MAV_RESULT_ACCEPTED:
                why = RESULT_NAMES.get(ack.result, str(ack.result))
                return False, self._msg(f'{name} refused: {why}', texts)
            if confirm is None:
                return True, self._msg(f'{name} accepted', texts)
            deadline = self.clock() + confirm_timeout
            while self.clock() < deadline:
                m = self._recv(deadline)
                if m is None:
                    continue
                if m.get_type() == 'STATUSTEXT':
                    texts.append(str(m.text).strip())
                elif confirm(m):
                    return True, self._msg(f'{name} accepted and confirmed', texts)
            return False, self._msg(f'{name} accepted but not confirmed within {confirm_timeout:.0f} s', texts)
        return False, self._msg(f'{name} failed: {last}', texts)

    @staticmethod
    def _msg(head, texts):
        return head + (' | ' + ' | '.join(texts) if texts else '')
