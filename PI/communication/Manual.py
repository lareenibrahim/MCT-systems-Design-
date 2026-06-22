import glob
import math
import threading
import time
from pyPS4Controller.controller import Controller

import qr_scanner   # shared color_event + COLOR_MAP live here

# ─────────────────────────────────────────────────────────────
# SERIAL PORTS  (injected by main.py — do NOT open here)
# ─────────────────────────────────────────────────────────────
BASE_PORT = None
ARM_PORT  = None

# ─────────────────────────────────────────────────────────────
# TUNING CONSTANTS
# ─────────────────────────────────────────────────────────────
DEADZONE        = 5000
DIAGONAL_BIAS   = 0.20
STEP_COARSE_DEG = 5.0    # fast movement
STEP_FINE_DEG   = 1.0    # precise alignment
AUTO_STEP_DEG   = 3.0    # degrees per step in auto/homing sequences
AUTO_SLEEP_SEC  = 0.3    # delay between steps in auto/homing sequences

# ─────────────────────────────────────────────────────────────
# SAVED POSITION  (fill servo angles from recording session)
# R1 press drives the arm to this position then grips
# ─────────────────────────────────────────────────────────────
SAVED_SERVO1 = 40
SAVED_SERVO2 = 130
Placing_servo1 = 108
Placing_servo2 = 48

HOME_SERVO1 = 40.0
HOME_SERVO2 = 130.0

# ─────────────────────────────────────────────────────────────
# DS4 LIGHT BAR  (via kernel LED sysfs interface)
# ─────────────────────────────────────────────────────────────
def _find_led_prefix():
    """Return the sysfs LED prefix for the first connected DS4, or None."""
    candidates = glob.glob('/sys/class/leds/*:red')
    if candidates:
        return candidates[0][:-4]
    return None

_LED_PREFIX = _find_led_prefix()

def set_controller_color(r: int, g: int, b: int):
    """Write R/G/B (0-255) to the DS4 light bar via sysfs."""
    if _LED_PREFIX is None:
        print(f"[LED] Light bar not found via sysfs — would set ({r},{g},{b})")
        return
    for channel, value in (('red', r), ('green', g), ('blue', b)):
        path = f"{_LED_PREFIX}:{channel}/brightness"
        try:
            with open(path, 'w') as f:
                f.write(str(value))
        except Exception as e:
            print(f"[LED] Could not write {path}: {e}")
    print(f"[LED] Light bar → R={r} G={g} B={b}")

def _start_color_watcher():
    """Background thread: waits for qr_scanner to signal a new color."""
    def loop():
        while True:
            qr_scanner.color_event.wait()
            qr_scanner.color_event.clear()
            with qr_scanner.color_lock:
                color_name = qr_scanner.detected_color
            if color_name and color_name in qr_scanner.COLOR_MAP:
                r, g, b = qr_scanner.COLOR_MAP[color_name]
                set_controller_color(r, g, b)
    threading.Thread(target=loop, daemon=True).start()

_start_color_watcher()

# ─────────────────────────────────────────────────────────────
# PACKET BUILDER
# ─────────────────────────────────────────────────────────────
def build_packet(cmd_char, b1=0, b2=0, b3=0, b4=0):
    cmd      = ord(cmd_char)
    checksum = (cmd + b1 + b2 + b3 + b4) % 256
    return bytes([0xFF, 0xAA, cmd, b1, b2, b3, b4, checksum])

def build_arm_ik_packet(cmd_char, servo1_deg, servo2_deg):
    a1 = int(servo1_deg * 100)
    a2 = int(servo2_deg * 100)

    a1_high = (a1 >> 8) & 0xFF
    a1_low  =  a1 & 0xFF
    a2_high = (a2 >> 8) & 0xFF
    a2_low  =  a2 & 0xFF

    return build_packet(cmd_char, a1_high, a1_low, a2_high, a2_low)

# ─────────────────────────────────────────────────────────────
# SEND HELPERS
# ─────────────────────────────────────────────────────────────
def send_base(cmd_char):
    packet = build_packet(cmd_char)
    if BASE_PORT is not None:
        BASE_PORT.write(packet)
    if cmd_char != 'S':
        print(f"[BASE] → '{cmd_char}'")

def send_arm(cmd_char):
    packet = build_packet(cmd_char)
    if ARM_PORT is not None:
        ARM_PORT.write(packet)
    print(f"[ARM ] → '{cmd_char}'")

def send_arm_ik(servo1_deg, servo2_deg):
    packet = build_arm_ik_packet('W', servo1_deg, servo2_deg)
    if ARM_PORT is not None:
        ARM_PORT.write(packet)

# ─────────────────────────────────────────────────────────────
# POSITION RECORDER
# ─────────────────────────────────────────────────────────────
def print_position(servo1, servo2):
    print(f"\n{'='*40}")
    print(f"[RECORD] Current arm position")
    print(f"  Servo1 : {servo1:.2f} deg")
    print(f"  Servo2 : {servo2:.2f} deg")
    print(f"{'='*40}\n")


# ─────────────────────────────────────────────────────────────
# CONTROLLER
# ─────────────────────────────────────────────────────────────
class RobotController(Controller):
       
    def __init__(self, switch_event=None, **kwargs):
        super().__init__(**kwargs)
        self._switch_event = switch_event

        self.left_x  = 0
        self.left_y  = 0

        self.use_coarse_step  = True   # start with coarse step
        self.current_step_deg = STEP_COARSE_DEG

        self.last_servo1 = HOME_SERVO1
        self.last_servo2 = HOME_SERVO2

        self.tilt_active    = False
        self.gripper_closed = False

        self._l2_held  = False
        self._r2_held  = False
        self._rot_lock = threading.Lock()

        # ── Homing ───────────────────────────────────────────
        print(f"[INIT] Homing to servo1={HOME_SERVO1}°  servo2={HOME_SERVO2}°")
        self._move_to(target1=HOME_SERVO1, target2=HOME_SERVO2)
        print(f"[INIT] Homing complete.")

        self._start_rotation_loop()
           
    # ─────────────────────────────────────────
    # JOINT CONTROL — one-shot helper
    # Called directly by arrow press callbacks.
    # Sends one angle increment and updates state.
    # ─────────────────────────────────────────
    def _move_joint(self, joint, delta_deg):
        if joint == 1:
            new_angle        = max(0.0, min(180.0, self.last_servo1 + delta_deg))
            self.last_servo1 = new_angle
        else:
            new_angle        = max(0.0, min(180.0, self.last_servo2 + delta_deg))
            self.last_servo2 = new_angle

        send_arm_ik(self.last_servo1, self.last_servo2)
        print(f"[JOINT] servo1={self.last_servo1:.1f}°  servo2={self.last_servo2:.1f}°")
           
    # ─────────────────────────────────────────
    # INCREMENTAL MOVE — used by homing and R1
    # Pass only the servo(s) you want to move:
    #   self._move_to(target1=90)
    #   self._move_to(target2=45)
    #   self._move_to(target1=90, target2=45)
    # ─────────────────────────────────────────
    def _move_to(self, target1=None, target2=None):
        s1 = self.last_servo1
        s2 = self.last_servo2
        t1 = target1 if target1 is not None else s1
        t2 = target2 if target2 is not None else s2

        while True:
            diff1 = t1 - s1
            diff2 = t2 - s2

            if abs(diff1) < AUTO_STEP_DEG and abs(diff2) < AUTO_STEP_DEG:
                s1 = t1
                s2 = t2
                send_arm_ik(s1, s2)
                self.last_servo1 = s1
                self.last_servo2 = s2
                break

            if abs(diff1) >= AUTO_STEP_DEG:
                s1 += AUTO_STEP_DEG if diff1 > 0 else -AUTO_STEP_DEG
                s1  = max(0.0, min(180.0, s1))
            else:
                s1 = t1

            if abs(diff2) >= AUTO_STEP_DEG:
                s2 += AUTO_STEP_DEG if diff2 > 0 else -AUTO_STEP_DEG
                s2  = max(0.0, min(180.0, s2))
            else:
                s2 = t2

            send_arm_ik(s1, s2)
            self.last_servo1 = s1
            self.last_servo2 = s2
            time.sleep(AUTO_SLEEP_SEC)

    def _toggle_step_size(self):
        self.use_coarse_step  = not self.use_coarse_step
        self.current_step_deg = STEP_COARSE_DEG if self.use_coarse_step else STEP_FINE_DEG
        mode = "COARSE" if self.use_coarse_step else "FINE"
        print(f"[STEP] Switched to {mode} step: {self.current_step_deg}° per arrow press")

    # ─────────────────────────────────────────
    # OPTIONS → signal main to request AUTO mode
    # main.py sends 'M' and waits for STM confirmation
    # before actually switching controllers.
    # ─────────────────────────────────────────
    def on_options_press(self):
        print("[MANUAL] Options pressed — signalling main to request AUTO mode …")
        if self._switch_event is not None:
            self._switch_event.set()
        if self._switch_event is not None:
            self._switch_event.set()

    # ─────────────────────────────────────────
    # ROTATION LOOP (L2/R2 for base rotation)
    # ─────────────────────────────────────────
    def _start_rotation_loop(self):
        def loop():
            while True:
                with self._rot_lock:
                    l2 = self._l2_held
                    r2 = self._r2_held
                if l2:
                    send_base('E')
                elif r2:
                    send_base('Q')
                time.sleep(0.05)
        threading.Thread(target=loop, daemon=True).start()

    # ─────────────────────────────────────────
    # BASE DIRECTION RESOLVER (left stick)
    # ─────────────────────────────────────────
    def _resolve_base(self):
        with self._rot_lock:
            rotating = self._l2_held or self._r2_held
        if rotating:
            return

        x, y      = self.left_x, self.left_y
        magnitude = math.sqrt(x**2 + y**2)

        if magnitude < DEADZONE:
            send_base('S')
            return

        angle   = math.degrees(math.atan2(-y, x)) % 360
        pure_hw = 22.5 * (1.0 + DIAGONAL_BIAS)

        if   90  - pure_hw <= angle <  90  + pure_hw:   send_base('B')
        elif 270 - pure_hw <= angle < 270  + pure_hw:   send_base('F')
        elif angle < pure_hw or angle >= 360 - pure_hw: send_base('L')
        elif 180 - pure_hw <= angle < 180  + pure_hw:   send_base('R')
        elif  45 - 22.5    <= angle <  45  + 22.5:      send_base('H')
        elif 135 - 22.5    <= angle < 135  + 22.5:      send_base('J')
        elif 225 - 22.5    <= angle < 225  + 22.5:      send_base('I')
        elif 315 - 22.5    <= angle < 315  + 22.5:      send_base('G')

    # ─────────────────────────────────────────
    # LEFT STICK → Base movement
    # ─────────────────────────────────────────
    def on_L3_up(self, value):    self.left_y = value;  self._resolve_base()
    def on_L3_down(self, value):  self.left_y = value;  self._resolve_base()
    def on_L3_left(self, value):  self.left_x = value;  self._resolve_base()
    def on_L3_right(self, value): self.left_x = value;  self._resolve_base()
    def on_L3_x_at_rest(self):    self.left_x = 0;      self._resolve_base()
    def on_L3_y_at_rest(self):    self.left_y = 0;      self._resolve_base()

    # ─────────────────────────────────────────
    # L2 / R2 → Base rotation
    # ─────────────────────────────────────────
    def on_L2_press(self, value):
        with self._rot_lock: self._l2_held = True

    def on_L2_release(self):
        with self._rot_lock: self._l2_held = False
        send_base('S')

    def on_R2_press(self, value):
        with self._rot_lock: self._r2_held = True

    def on_R2_release(self):
        with self._rot_lock: self._r2_held = False
        send_base('S')

    # ─────────────────────────────────────────
    # L1 → toggle step size (coarse / fine)
    # ─────────────────────────────────────────
    def on_L1_press(self):
        self._toggle_step_size()

    def on_L1_release(self):
        pass   # nothing needed on release

    # ─────────────────────────────────────────
    # R1 → go to saved position then grip
    # ─────────────────────────────────────────
    def on_R1_press(self):


        # ── STEP 1 ──────────────────────────────────────────────
        self._move_to(target2=SAVED_SERVO2)
        print(f"[R1] Servo2 at {SAVED_SERVO2}°")

        # ── STEP 2 ──────────────────────────────────────────────
        self._move_to(target1=SAVED_SERVO1)
        print(f"[R1] Servo1 at {SAVED_SERVO1}°")
   


        
        

        # ── Open gripper — release cube ────────────────────────
        if self.gripper_closed:
            send_arm('2')
            self.gripper_closed = False
            print("[R1] Gripper opened")

        print("[R1] Sequence complete")


    def on_R1_release(self): pass

    # ─────────────────────────────────────────
    # D-PAD → Joint control (direct one-shot angle)
    # Up/Down   → Servo1 (shoulder) +/−
    # Right/Left → Servo2 (elbow)   +/−
    # ─────────────────────────────────────────
    def on_up_arrow_press(self):
        self._move_joint(1, +self.current_step_deg)

    def on_down_arrow_press(self):
        self._move_joint(1, -self.current_step_deg)

    def on_right_arrow_press(self):
        self._move_joint(2, +self.current_step_deg)

    def on_left_arrow_press(self):
        self._move_joint(2, -self.current_step_deg)

    # ─────────────────────────────────────────
    # TOUCHPAD → Record current position
    # ─────────────────────────────────────────
    def on_playstation_button_press(self):
        print_position(self.last_servo1, self.last_servo2)

    # ─────────────────────────────────────────
    # CROSS → Gripper toggle
    # ─────────────────────────────────────────
    def on_x_press(self):
        self.gripper_closed = not self.gripper_closed
        if self.gripper_closed:
            send_arm('1')
            print('[GRIP] Closed')
        else:
            send_arm('2')
            print('[GRIP] Opened')

    def on_x_release(self): pass

    # ─────────────────────────────────────────
    # TRIANGLE → Tilt toggle
    # ─────────────────────────────────────────
    def on_triangle_press(self):
        self.tilt_active = not self.tilt_active
        send_arm('T' if self.tilt_active else 'Y')

    # ─────────────────────────────────────────
    # UNUSED
    # ─────────────────────────────────────────
    def on_circle_press(self): pass
    def on_square_press(self): pass
    # ─────────────────────────────────────────
    # RIGHT STICK — unused (no IK joystick mode)
    # Override to silence library's default debug prints
    # ─────────────────────────────────────────
    def on_R3_up(self, value):    pass
    def on_R3_down(self, value):  pass
    def on_R3_left(self, value):  pass
    def on_R3_right(self, value): pass
    def on_R3_x_at_rest(self):    pass
    def on_R3_y_at_rest(self):    pass
    def on_R3_press(self):        pass
    def on_R3_release(self):      pass


# ─────────────────────────────────────────────────────────────
# ENTRY POINT  — only when run directly
# ─────────────────────────────────────────────────────────────
def main():
    controller = RobotController(
        interface="/dev/input/js0",
        connecting_using_ds4drv=False
    )
    controller.listen()

if __name__ == "__main__":
    main()