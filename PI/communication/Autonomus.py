import math
import threading
import time
import struct
import queue
from pyPS4Controller.controller import Controller

# ─────────────────────────────────────────────────────────────
# INJECTED BY main.py AT STARTUP
# ─────────────────────────────────────────────────────────────
BASE_PORT_INJECTED = None
ARM_PORT_INJECTED  = None   # NEW: needed so this file can drive the arm too
stm_ack_queue      = None
mode_confirm_queue = None

# ─────────────────────────────────────────────────────────────
# TUNING CONSTANTS
# ─────────────────────────────────────────────────────────────
DEADZONE      = 5000
DIAGONAL_BIAS = 0.20

CMD_TRANSLATE = 0x10
CMD_TWIST     = 0x20

ACK_TIMEOUT_S = 30.0

# ── ARM STATE ──
# The arm STM has no "leave this servo alone" sentinel — every 'W' packet
# moves BOTH servos to the angles it contains (confirmed from RobotController
# ._move_to(), which resends the unchanged servo's last known angle rather
# than omitting it). So this file must track the last commanded angle for
# each servo itself, exactly like RobotController does with
# self.last_servo1 / self.last_servo2, and reuse that value whenever a map
# step's move flag for a servo is False.
HOME_SERVO1 = 40.0
HOME_SERVO2 = 130.0

_last_servo1 = HOME_SERVO1
_last_servo2 = HOME_SERVO2

# ─────────────────────────────────────────────────────────────
# MAP SEQUENCE
#
# Four step shapes, executed strictly in the order written:
#
#   ('translate', fwd_cm, strafe_cm)
#   ('twist', degrees)
#   ('arm', move_servo1, move_servo2, angle1, angle2, step_deg, step_sleep_s, delay_s)
#   ('gripper', action, delay_s)
#
# Arm step fields:
#   move_servo1 / move_servo2  -> bool, whether that servo moves this step
#                                  (mirrors the R1 "servo1 true / servo2 false" logic)
#   angle1 / angle2            -> target angle for that servo, ignored (use None)
#                                  if its move_servoX is False
#   step_deg                   -> degrees per intermediate packet, same idea as
#                                  AUTO_STEP_DEG in the manual file's _move_to().
#                                  Smaller = smoother sweep, more packets sent.
#                                  Set per call — every arm step can use a
#                                  different sweep granularity.
#   step_sleep_s                -> seconds to wait BETWEEN each intermediate
#                                  step packet (like AUTO_SLEEP_SEC). Set per
#                                  call too — different moves can sweep at
#                                  different speeds.
#   delay_s                     -> extra settle wait AFTER the sweep finishes
#                                  and reaches the final target, before the
#                                  runner moves to the next map step. The base
#                                  sends nothing during the whole arm step
#                                  (sweep + this delay), so it stays stopped
#                                  the entire time the arm is moving.
#
# Gripper step fields:
#   action   -> 'close' or 'open' (mirrors on_x_press(): send_arm('1') closes,
#                send_arm('2') opens — same single-char protocol, no IK angles)
#   delay_s  -> wait AFTER sending the gripper command, before the runner
#                moves to the next map step. Set per call.
#
# Example: sweep servo1 to 90° in 5° steps, 0.3s between steps, then wait
# an extra 1.0s once it arrives:
#   ('arm', True, False, 90.0, None, 5.0, 0.3, 1.0)
#
# Example: sweep both servos in fine 2° steps, fast 0.1s between steps,
# no extra settle wait:
#   ('arm', True, True, 60.0, 120.0, 2.0, 0.1, 0.0)
#
# Example: close the gripper, then wait 0.5s before continuing:
#   ('gripper', 'close', 0.5)
#
# Example: open the gripper, no extra wait:
#   ('gripper', 'open', 0.0)
# ─────────────────────────────────────────────────────────────
MAP_SEQUENCE = [
    ('translate',  -57, 0 ),
    ('twist',     -91),
    ('translate',  -110, 0 ),
    ('gripper', 'open', 0.5),
    ('arm', False, True, None ,155, 3.0, 0.3, 1.0),
    ('arm', True, False, 60 ,None, 3.0, 0.3, 1.0),
    ('arm', False, True, None ,180, 3.0, 0.3, 1.0),
    ('gripper', 'close', 0.5),
    ('arm', True, False, 20 ,None, 3.0, 0.3, 1.0),
    ('arm', False, True, None ,110, 3.0, 0.3, 1.0),
    ('arm', True, False, 70 ,None, 3.0, 0.3, 1.0),
    ('gripper', 'open', 0.5),
    ('arm', True, True, 40 ,130, 3.0, 0.3, 1.0),
    ###############
    ('translate',  0, 153 ),
    ('translate',  0, -8 ),
    ('twist',     90),
    ('arm', False, True, None ,150, 3.0, 0.3, 1.0),
    ('arm', True, False, 70 ,None, 3.0, 0.3, 1.0),
    ('gripper', 'close', 0.5),
    ('arm', True, False, 20 ,None, 3.0, 0.3, 1.0),
    ('arm', False, True, None ,110, 3.0, 0.3, 1.0),
    ('arm', True, False, 70 ,None, 3.0, 0.3, 1.0),
    ('gripper', 'open', 0.5),
    ('arm', True, True, 40 ,130, 3.0, 0.3, 1.0),
    ('twist',     90),
    #################
    ('translate',  -150, 0),
    ('translate',  -50, 50 ),
    ('arm', False, True, None ,140, 3.0, 0.3, 1.0),
    ('arm', True, False, 60 ,None, 3.0, 0.3, 1.0),
    ('arm', True, False, 100 ,None, 3.0, 0.3, 1.0),
    ('gripper', 'close', 0.5),
    ('arm', True, False, 20 ,None, 3.0, 0.3, 1.0),
    ('arm', False, True, None ,110, 3.0, 0.3, 1.0),
    ('arm', True, False, 70 ,None, 3.0, 0.3, 1.0),
    ('gripper', 'open', 0.5),
    ('arm', True, True, 40 ,130, 3.0, 0.3, 1.0),

]

# ─────────────────────────────────────────────────────────────
# PACKET BUILDERS — BASE
# ─────────────────────────────────────────────────────────────
def _to_signed16_bytes(value: int):
    raw = struct.pack('>h', value)
    return raw[0], raw[1]

def build_translate_packet(fwd_cm: int, strafe_cm: int) -> bytes:
    cmd  = CMD_TRANSLATE
    b1, b2 = _to_signed16_bytes(fwd_cm)
    b3, b4 = _to_signed16_bytes(strafe_cm)
    checksum = (cmd + b1 + b2 + b3 + b4) % 256
    return bytes([0xFF, 0xAA, cmd, b1, b2, b3, b4, checksum])

def build_twist_packet(degrees: int) -> bytes:
    cmd  = CMD_TWIST
    b1, b2 = _to_signed16_bytes(degrees)
    checksum = (cmd + b1 + b2) % 256
    return bytes([0xFF, 0xAA, cmd, b1, b2, 0x00, 0x00, checksum])

def build_manual_packet(cmd_char: str) -> bytes:
    cmd      = ord(cmd_char)
    checksum = cmd % 256
    return bytes([0xFF, 0xAA, cmd, 0, 0, 0, 0, checksum])

# ─────────────────────────────────────────────────────────────
# PACKET BUILDER — ARM
# Mirrors build_arm_ik_packet() from the manual controller file so the
# wire format the arm STM expects stays identical between manual ('W')
# IK moves and these automatic map-sequence arm moves.
# ─────────────────────────────────────────────────────────────
def build_arm_ik_packet(cmd_char: str, servo1_deg: float, servo2_deg: float) -> bytes:
    a1 = int(servo1_deg * 100)
    a2 = int(servo2_deg * 100)

    a1_high = (a1 >> 8) & 0xFF
    a1_low  =  a1 & 0xFF
    a2_high = (a2 >> 8) & 0xFF
    a2_low  =  a2 & 0xFF

    cmd      = ord(cmd_char)
    checksum = (cmd + a1_high + a1_low + a2_high + a2_low) % 256
    return bytes([0xFF, 0xAA, cmd, a1_high, a1_low, a2_high, a2_low, checksum])

# ─────────────────────────────────────────────────────────────
# SEND HELPERS
# ─────────────────────────────────────────────────────────────
def send_raw(packet: bytes, label: str):
    port = BASE_PORT_INJECTED
    if port is not None:
        port.write(packet)
    print(f"[AUTO-BASE] {label}  packet: {list(packet)}", flush=True)

def send_manual_cmd(cmd_char: str):
    packet = build_manual_packet(cmd_char)
    send_raw(packet, f"→ '{cmd_char}'")

def send_gripper(action: str, label: str):
    """
    Mirrors RobotController.on_x_press(): send_arm('1') closes the gripper,
    send_arm('2') opens it. Same single-char packet shape as the base's
    manual commands (build_manual_packet), but written to ARM_PORT_INJECTED
    since the gripper lives on the arm STM, not the base STM.
    """
    cmd_char = '1' if action == 'close' else '2'
    packet   = build_manual_packet(cmd_char)

    port = ARM_PORT_INJECTED
    if port is not None:
        port.write(packet)
    print(f"[AUTO-ARM] {label}  packet: {list(packet)}", flush=True)

def send_arm_step(move_servo1: bool, move_servo2: bool, angle1, angle2,
                   step_deg: float, step_sleep_s: float, label: str):
    """
    Sweeps the arm toward (angle1, angle2) in incremental packets, exactly
    like RobotController._move_to() does for the manual R1 sequence —
    instead of jumping straight to the target in a single packet.

    step_deg     -> degrees per intermediate packet (per-call, like AUTO_STEP_DEG)
    step_sleep_s -> seconds between intermediate packets (per-call, like AUTO_SLEEP_SEC)

    Confirmed arm STM behavior: every 'W' packet moves BOTH servos to
    whatever angles it contains — there's no per-servo "ignore" flag.
    To hold a servo still, resend its LAST KNOWN angle instead of a fresh
    target. _last_servo1 / _last_servo2 track that, mirroring
    RobotController.last_servo1 / last_servo2.
    """
    global _last_servo1, _last_servo2

    s1 = _last_servo1
    s2 = _last_servo2
    t1 = angle1 if move_servo1 else s1
    t2 = angle2 if move_servo2 else s2

    step_count = 0
    while True:
        diff1 = t1 - s1
        diff2 = t2 - s2

        if abs(diff1) < step_deg and abs(diff2) < step_deg:
            s1 = t1
            s2 = t2
            _send_arm_packet(s1, s2, label, step_count, final=True)
            break

        if abs(diff1) >= step_deg:
            s1 += step_deg if diff1 > 0 else -step_deg
            s1  = max(0.0, min(180.0, s1))
        else:
            s1 = t1

        if abs(diff2) >= step_deg:
            s2 += step_deg if diff2 > 0 else -step_deg
            s2  = max(0.0, min(180.0, s2))
        else:
            s2 = t2

        _send_arm_packet(s1, s2, label, step_count, final=False)
        step_count += 1
        time.sleep(step_sleep_s)

    _last_servo1 = s1
    _last_servo2 = s2


def _send_arm_packet(s1: float, s2: float, label: str, step_count: int, final: bool):
    packet = build_arm_ik_packet('W', s1, s2)
    port   = ARM_PORT_INJECTED
    if port is not None:
        port.write(packet)
    tag = "FINAL" if final else f"step {step_count}"
    print(f"[AUTO-ARM] {label} [{tag}]  servo1={s1:.1f}° servo2={s2:.1f}°  "
          f"packet: {list(packet)}", flush=True)

# ─────────────────────────────────────────────────────────────
# ACK READER
# ─────────────────────────────────────────────────────────────
def wait_for_ack(stop_flag: threading.Event, timeout: float = ACK_TIMEOUT_S):
    """
    Waits for a genuine SUCCESS ack (status 0x00) for the CURRENT step.
    A 0x02 rejection (STM still busy with the previous move) is NOT treated
    as completion — we keep waiting for the real 0x00 within the timeout,
    so the Pi only advances once the STM has actually finished the step.
    """
    if BASE_PORT_INJECTED is None:
        print("[AUTO-BASE] (no port) simulating ACK", flush=True)
        time.sleep(0.05)
        return 0x00, 0x00

    deadline = time.time() + timeout
    while time.time() < deadline:
        if stop_flag.is_set():
            print("[AUTO-BASE] ACK wait aborted by stop flag", flush=True)
            return None, None
        try:
            echo_cmd, status = stm_ack_queue.get(timeout=0.1)

            if status == 0x02:
                print(f"[AUTO-BASE] STM busy (rejected) — cmd=0x{echo_cmd:02X}, "
                      f"still waiting for real completion …", flush=True)
                continue

            if status == 0x00:
                print(f"[AUTO-BASE] ACK received — cmd=0x{echo_cmd:02X} status=0x{status:02X}", flush=True)
                return echo_cmd, status

            print(f"[AUTO-BASE] Unexpected ack status=0x{status:02X} for "
                  f"cmd=0x{echo_cmd:02X} — ignoring, still waiting …", flush=True)

        except queue.Empty:
            continue

    print("[AUTO-BASE] ACK TIMEOUT — continuing anyway", flush=True)
    return None, None

# ─────────────────────────────────────────────────────────────
# MAP RUNNER
# ─────────────────────────────────────────────────────────────
def run_map(stop_flag: threading.Event):
    print(f"[AUTO-BASE] Starting map — {len(MAP_SEQUENCE)} steps", flush=True)

    for idx, step in enumerate(MAP_SEQUENCE):
        if stop_flag.is_set():
            print("[AUTO-BASE] Map aborted (stop requested)", flush=True)
            return

        kind = step[0]

        if kind == 'translate':
            fwd_cm, strafe_cm = step[1], step[2]
            packet = build_translate_packet(fwd_cm, strafe_cm)
            label  = f"Step {idx+1}/{len(MAP_SEQUENCE)}: TRANSLATE fwd={fwd_cm}cm strafe={strafe_cm}cm"

            print(f"[AUTO-BASE] {label}", flush=True)
            send_raw(packet, label)
            wait_for_ack(stop_flag)

        elif kind == 'twist':
            degrees = step[1]
            packet  = build_twist_packet(degrees)
            label   = f"Step {idx+1}/{len(MAP_SEQUENCE)}: TWIST {degrees}° ({'CW' if degrees>0 else 'CCW'})"

            print(f"[AUTO-BASE] {label}", flush=True)
            send_raw(packet, label)
            wait_for_ack(stop_flag)

        elif kind == 'arm':
            # ('arm', move_servo1, move_servo2, angle1, angle2, step_deg, step_sleep_s, delay_s)
            _, move_s1, move_s2, ang1, ang2, step_deg, step_sleep_s, delay_s = step
            label = (f"Step {idx+1}/{len(MAP_SEQUENCE)}: ARM "
                     f"servo1={'%.1f' % ang1 if move_s1 else 'hold'} "
                     f"servo2={'%.1f' % ang2 if move_s2 else 'hold'} "
                     f"step={step_deg}° step_sleep={step_sleep_s}s delay={delay_s}s")

            print(f"[AUTO-BASE] {label}", flush=True)

            # Base sends nothing during an arm step — it is already stopped
            # from the previous base step's completion ack, so it stays put
            # while the arm sweeps toward its target. After the sweep
            # finishes we wait this step's extra delay_s before letting the
            # loop continue to the next map entry.
            send_arm_step(move_s1, move_s2, ang1, ang2, step_deg, step_sleep_s, label)
            time.sleep(delay_s)

        elif kind == 'gripper':
            # ('gripper', action, delay_s)
            _, action, delay_s = step
            label = f"Step {idx+1}/{len(MAP_SEQUENCE)}: GRIPPER {action.upper()}"

            print(f"[AUTO-BASE] {label}", flush=True)
            send_gripper(action, label)
            time.sleep(delay_s)

        else:
            print(f"[AUTO-BASE] Unknown step type '{kind}' — skipping", flush=True)
            continue

    print("[AUTO-BASE] Map complete ✓", flush=True)


# ─────────────────────────────────────────────────────────────
# CONTROLLER
# ─────────────────────────────────────────────────────────────
class AutoBaseController(Controller):

    MODE_MANUAL_CTRL = 'manual_ctrl'
    MODE_AUTO_MAP    = 'auto_map'

    def __init__(self, base_port, switch_event, **kwargs):
        super().__init__(**kwargs)
        self._switch_event = switch_event

        self.left_x   = 0
        self.left_y   = 0
        self._l2_held = False
        self._r2_held = False
        self._rot_lock = threading.Lock()

        self._mode      = self.MODE_MANUAL_CTRL
        self._mode_lock = threading.Lock()
        self._map_stop  = threading.Event()

        self._start_rotation_loop()

    def on_options_press(self):
        print("[AUTO-BASE] Options pressed — stopping map, signalling main to request MANUAL …", flush=True)
        self._map_stop.set()
        if self._switch_event is not None:
            self._switch_event.set()

    def on_x_press(self):
        with self._mode_lock:
            if self._mode == self.MODE_AUTO_MAP:
                print("[AUTO-BASE] Map already running", flush=True)
                return
            self._mode = self.MODE_AUTO_MAP

        print("[AUTO-BASE] X pressed — starting map sequence", flush=True)
        self._map_stop.clear()

        def _run():
            send_manual_cmd('N')
            print("[AUTO-BASE] Sent 'N' → STM auto_is_running activated", flush=True)
            time.sleep(0.1)
            run_map(self._map_stop)
            with self._mode_lock:
                self._mode = self.MODE_MANUAL_CTRL
            print("[AUTO-BASE] Returned to manual-ctrl sub-mode", flush=True)

        threading.Thread(target=_run, daemon=True).start()

    def on_x_release(self): pass

    def on_circle_press(self):
        print("[AUTO-BASE] O pressed — FULL STOP", flush=True)
        self._map_stop.set()
        with self._mode_lock:
            self._mode = self.MODE_MANUAL_CTRL
        send_manual_cmd('X')

    def on_circle_release(self): pass

    def _start_rotation_loop(self):
        def loop():
            while True:
                with self._mode_lock:
                    in_manual = (self._mode == self.MODE_MANUAL_CTRL)
                with self._rot_lock:
                    l2 = self._l2_held
                    r2 = self._r2_held
                if in_manual:
                    if l2:
                        send_manual_cmd('Q')
                    elif r2:
                        send_manual_cmd('E')
                time.sleep(0.05)
        threading.Thread(target=loop, daemon=True).start()

    def _resolve_base(self):
        with self._mode_lock:
            if self._mode != self.MODE_MANUAL_CTRL:
                return
        with self._rot_lock:
            rotating = self._l2_held or self._r2_held
        if rotating:
            return

        x, y      = self.left_x, self.left_y
        magnitude = math.sqrt(x**2 + y**2)

        if magnitude < DEADZONE:
            send_manual_cmd('S')
            return

        angle   = math.degrees(math.atan2(-y, x)) % 360
        pure_hw = 22.5 * (1.0 + DIAGONAL_BIAS)

        if   90  - pure_hw <= angle <  90  + pure_hw:   send_manual_cmd('B')
        elif 270 - pure_hw <= angle < 270  + pure_hw:   send_manual_cmd('F')
        elif angle < pure_hw or angle >= 360 - pure_hw: send_manual_cmd('L')
        elif 180 - pure_hw <= angle < 180  + pure_hw:   send_manual_cmd('R')
        elif  45 - 22.5    <= angle <  45  + 22.5:      send_manual_cmd('H')
        elif 135 - 22.5    <= angle < 135  + 22.5:      send_manual_cmd('J')
        elif 225 - 22.5    <= angle < 225  + 22.5:      send_manual_cmd('I')
        elif 315 - 22.5    <= angle < 315  + 22.5:      send_manual_cmd('G')

    def on_L3_up(self, value):    self.left_y = value;  self._resolve_base()
    def on_L3_down(self, value):  self.left_y = value;  self._resolve_base()
    def on_L3_left(self, value):  self.left_x = value;  self._resolve_base()
    def on_L3_right(self, value): self.left_x = value;  self._resolve_base()
    def on_L3_x_at_rest(self):    self.left_x = 0;      self._resolve_base()
    def on_L3_y_at_rest(self):    self.left_y = 0;      self._resolve_base()

    def on_L2_press(self, value):
        with self._rot_lock: self._l2_held = True
    def on_L2_release(self):
        with self._rot_lock: self._l2_held = False
        with self._mode_lock:
            if self._mode == self.MODE_MANUAL_CTRL:
                send_manual_cmd('S')

    def on_R2_press(self, value):
        with self._rot_lock: self._r2_held = True
    def on_R2_release(self):
        with self._rot_lock: self._r2_held = False
        with self._mode_lock:
            if self._mode == self.MODE_MANUAL_CTRL:
                send_manual_cmd('S')