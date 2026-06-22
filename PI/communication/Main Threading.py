"""
main.py  –  Orchestrator
"""

import threading
import time
import serial
import queue
from pyPS4Controller.controller import Controller

import qr_scanner
import PStoPIdebugged
import auto_base
from PStoPIdebugged import set_controller_color

# ─────────────────────────────────────────────────────────────
# SERIAL PORTS  (opened once, injected into both modules)
# ─────────────────────────────────────────────────────────────
try:
    BASE_PORT = serial.Serial('/dev/ttySTM_BASE', 115200, timeout=1)
    print("[SERIAL] Base STM connected on /dev/ttySTM_BASE", flush=True)
except Exception as e:
    BASE_PORT = None
    print(f"[SERIAL] Base STM not connected ({e}) — prints only", flush=True)

try:
    ARM_PORT = serial.Serial('/dev/ttySTM_ARM', 115200, timeout=1)
    print("[SERIAL] Arm  STM connected on /dev/ttySTM_ARM", flush=True)
except Exception as e:
    ARM_PORT = None
    print(f"[SERIAL] Arm  STM not connected ({e}) — prints only", flush=True)

# ─────────────────────────────────────────────────────────────
# INJECT PORTS INTO MODULES
# ─────────────────────────────────────────────────────────────
PStoPIdebugged.BASE_PORT = BASE_PORT
PStoPIdebugged.ARM_PORT  = ARM_PORT
auto_base.BASE_PORT_INJECTED = BASE_PORT
auto_base.ARM_PORT_INJECTED  = ARM_PORT 
# ─────────────────────────────────────────────────────────────
# QUEUES
# stm_ack_queue      → map step ACKs  (0xFF 0xBB packets)
# mode_confirm_queue → mode switch confirmations (0xFF 0xCC packets)
# ─────────────────────────────────────────────────────────────
stm_ack_queue      = queue.Queue()
mode_confirm_queue = queue.Queue()

auto_base.stm_ack_queue      = stm_ack_queue
auto_base.mode_confirm_queue = mode_confirm_queue

# ─────────────────────────────────────────────────────────────
# MODE CONFIRMATION PROTOCOL
# Packet sent by STM after every 'M' or 'P' command:
#   [0xFF, 0xCC, confirmed_mode, 0x00, 0x00, 0x00, 0x00, checksum]
#   confirmed_mode: 0x00 = Manual, 0x01 = Auto
#
# Pi sends the command, then waits up to MODE_CONFIRM_TIMEOUT_S.
# If confirmed mode matches what was requested → switch.
# If timeout or wrong mode → stay in current mode, print warning.
# ─────────────────────────────────────────────────────────────
MODE_CONFIRM_TIMEOUT_S = 3.0
MODE_MANUAL_BYTE       = 0x00
MODE_AUTO_BYTE         = 0x01

def _build_cmd_packet(cmd_char: str) -> bytes:
    cmd      = ord(cmd_char)
    checksum = cmd % 256
    return bytes([0xFF, 0xAA, cmd, 0, 0, 0, 0, checksum])

def send_mode_command_and_wait(cmd_char: str, expected_mode_byte: int) -> bool:
    """
    Send a mode command ('M' or 'P') to the STM and wait for the STM to
    confirm it has entered the requested mode via a 0xFF 0xCC packet.

    Returns True if the STM confirmed the correct mode within the timeout,
    False otherwise (Pi does NOT switch modes on False).
    """
    if BASE_PORT is None:
        # No hardware — simulate success so development still works
        print(f"[MODE ] No port — simulating STM confirmation for '{cmd_char}'", flush=True)
        return True

    # Flush any stale confirmations sitting in the queue before sending
    while not mode_confirm_queue.empty():
        try:
            mode_confirm_queue.get_nowait()
        except queue.Empty:
            break

    # Send the command
    packet = _build_cmd_packet(cmd_char)
    BASE_PORT.write(packet)
    print(f"[MODE ] Sent '{cmd_char}' to STM — waiting for confirmation …", flush=True)

    # Wait for STM to confirm
    deadline = time.time() + MODE_CONFIRM_TIMEOUT_S
    while time.time() < deadline:
        try:
            confirmed_mode = mode_confirm_queue.get(timeout=0.1)
            if confirmed_mode == expected_mode_byte:
                mode_name = "MANUAL" if expected_mode_byte == MODE_MANUAL_BYTE else "AUTO"
                print(f"[MODE ] STM confirmed → {mode_name} ✓", flush=True)
                return True
            else:
                # STM confirmed, but the wrong mode — could be a queued stale packet
                print(f"[MODE ] Got confirmation 0x{confirmed_mode:02X} but expected "
                      f"0x{expected_mode_byte:02X} — ignoring, still waiting …", flush=True)
        except queue.Empty:
            continue

    print(f"[MODE ] WARNING: STM did not confirm mode switch for '{cmd_char}' "
          f"within {MODE_CONFIRM_TIMEOUT_S}s — staying in current mode.", flush=True)
    return False

# ─────────────────────────────────────────────────────────────
# CAMERA THREAD
# ─────────────────────────────────────────────────────────────
def run_camera():
    try:
        qr_scanner.main()
    except Exception as e:
        import traceback
        print(f"[CAMERA] Thread crashed: {e}", flush=True)
        traceback.print_exc()

# ─────────────────────────────────────────────────────────────
# BACKGROUND SERIAL READER
# Single owner of BASE_PORT — routes packets to the right queue.
#
#   0xFF 0xBB → map step ACK        → stm_ack_queue
#   0xFF 0xCC → mode confirmation   → mode_confirm_queue
#   anything else → text telemetry  → log file + console
# ─────────────────────────────────────────────────────────────
LOG_FILE = open("/tmp/stm_telemetry.log", "w", buffering=1)

def read_serial_background():
    if BASE_PORT is None:
        return

    while True:
        try:
            raw = BASE_PORT.read(1)   # blocking single-byte read
            if not raw:
                continue

            if raw[0] == 0xFF:
                # Read the second byte to decide packet type
                second = BASE_PORT.read(1)
                if not second:
                    continue

                if second[0] == 0xBB:
                    # ── Map step ACK packet ──
                    rest = BASE_PORT.read(6)   # 6 more bytes → total 8
                    if len(rest) == 6:
                        echo_cmd = rest[0]     # byte index 2 of full packet
                        status   = rest[1]     # byte index 3
                        checksum = rest[5]     # byte index 7
                        calc     = (echo_cmd + status) % 256
                        # Validate: echo_cmd must be a known map command (0x10 or 0x20)
                        # and checksum must match. This rejects garbled text bytes that
                        # get bundled with the ACK by the CDC driver.
                        if echo_cmd in (0x10, 0x20, 0x30) and checksum == calc:
                            stm_ack_queue.put((echo_cmd, status))
                            print(f"[SERIAL] ACK queued — cmd=0x{echo_cmd:02X} "
                                  f"status=0x{status:02X}", flush=True)
                        else:
                            print(f"[SERIAL] ACK REJECTED (garbled) — "
                                  f"cmd=0x{echo_cmd:02X} chk=0x{checksum:02X} "
                                  f"calc=0x{calc:02X}", flush=True)

                elif second[0] == 0xCC:
                    # ── Mode confirmation packet ──
                    rest = BASE_PORT.read(6)   # 6 more bytes → total 8
                    if len(rest) == 6:
                        confirmed_mode = rest[0]   # byte index 2: 0x00=Manual 0x01=Auto
                        checksum       = rest[5]   # byte index 7
                        calc           = confirmed_mode % 256
                        if checksum == calc:
                            mode_confirm_queue.put(confirmed_mode)
                            mode_name = "MANUAL" if confirmed_mode == 0x00 else "AUTO"
                            print(f"[SERIAL] Mode confirm queued → {mode_name} "
                                  f"(0x{confirmed_mode:02X})", flush=True)
                        else:
                            print(f"[SERIAL] Mode confirm checksum FAIL — "
                                  f"got 0x{checksum:02X} expected 0x{calc:02X}", flush=True)

                else:
                    # Unknown binary packet — treat as text telemetry
                    line = bytes([0xFF, second[0]]) + BASE_PORT.readline()
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        print(text, flush=True)
                        LOG_FILE.write(text + "\n")

            else:
                # ── Text / telemetry line ──
                line = raw + BASE_PORT.readline()
                text = line.decode('utf-8', errors='ignore').strip()
                if text:
                    print(text, flush=True)
                    LOG_FILE.write(text + "\n")

        except Exception as e:
            print(f"[SERIAL] Read error: {e}", flush=True)
            time.sleep(1)

# ─────────────────────────────────────────────────────────────
# THE SAFE IDLE CONTROLLER
# ─────────────────────────────────────────────────────────────
class IdleController(Controller):
    """Ignores all inputs until Options is pressed."""
    def __init__(self, switch_event, **kwargs):
        super().__init__(**kwargs)
        self._switch_event = switch_event

    def on_options_press(self):
        print("[IDLE ] Options pressed — requesting MANUAL mode from STM …", flush=True)
        self._switch_event.set()

# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":

    threading.Thread(target=run_camera, daemon=True).start()
    print("[CAMERA] Camera thread started", flush=True)

    threading.Thread(target=read_serial_background, daemon=True).start()
    print("[MAIN ] Serial reader thread started", flush=True)

    MODE_IDLE      = 'idle'
    MODE_MANUAL    = 'manual'
    MODE_AUTOMATIC = 'automatic'

    current_mode = MODE_IDLE
    set_controller_color(255, 0, 0)   # Red = Idle/Safe
    print("[MAIN ] Default mode: IDLE (Safe State)", flush=True)

    while True:
        switch_event = threading.Event()

        # ── BUILD the correct controller ──────────────────────
        if current_mode == MODE_IDLE:
            print("[MAIN ] Starting IDLE controller (Press Options to wake) …", flush=True)
            ctrl = IdleController(
                interface="/dev/input/js0",
                connecting_using_ds4drv=False,
                switch_event=switch_event
            )
        elif current_mode == MODE_MANUAL:
            print("[MAIN ] Starting MANUAL controller …", flush=True)
            ctrl = PStoPIdebugged.RobotController(
                interface="/dev/input/js0",
                connecting_using_ds4drv=False,
                switch_event=switch_event
            )
        else:
            print("[MAIN ] Starting AUTOMATIC controller …", flush=True)
            ctrl = auto_base.AutoBaseController(
                base_port=BASE_PORT,
                interface="/dev/input/js0",
                connecting_using_ds4drv=False,
                switch_event=switch_event
            )

        # ── START controller in a daemon thread ───────────────
        t = threading.Thread(target=ctrl.listen, daemon=True)
        t.start()

        # ── WAIT for Options press ─────────────────────────────
        switch_event.wait()
        print("[MAIN ] Switch event received — asking STM to change mode …", flush=True)

        # ── DETERMINE what mode we WANT to switch to ──────────
        if current_mode == MODE_IDLE:
            next_mode     = MODE_MANUAL
            cmd_char      = 'P'              # 'P' = go to Manual
            expected_byte = MODE_MANUAL_BYTE
            next_color    = (0, 0, 255)      # Blue
        elif current_mode == MODE_MANUAL:
            next_mode     = MODE_AUTOMATIC
            cmd_char      = 'M'              # 'M' = go to Auto
            expected_byte = MODE_AUTO_BYTE
            next_color    = (0, 255, 0)      # Green
        else:
            next_mode     = MODE_MANUAL
            cmd_char      = 'P'              # 'P' = go to Manual
            expected_byte = MODE_MANUAL_BYTE
            next_color    = (0, 0, 255)      # Blue

        # ── STOP THE ACTIVE CONTROLLER FIRST ──────────────────
        # Critical: stop before sending the mode command so the controller's
        # background threads (rotation loop, stick loop) cannot keep spamming
        # 'S' packets that overwrite valid_cmd on the STM before it reads 'M'/'P'.
        print("[MAIN ] Stopping active controller before mode command …", flush=True)
        try:
            ctrl.stop = True
        except Exception:
            pass
        t.join(timeout=3.0)
        time.sleep(0.3)   # Let the serial line go quiet before sending mode cmd

        # ── NOW ASK STM AND WAIT FOR CONFIRMATION ─────────────
        confirmed = send_mode_command_and_wait(cmd_char, expected_byte)

        if not confirmed:
            # STM did not confirm — do NOT switch, restart the same controller
            print(f"[MAIN ] Mode switch ABORTED — STM did not confirm. "
                  f"Restarting {current_mode.upper()} controller.", flush=True)
            # Loop continues with current_mode unchanged → restarts same controller
            continue

        # ── CONFIRMED: apply new mode ─────────────────────────
        current_mode = next_mode
        set_controller_color(*next_color)
        print(f"[MAIN ] Switched → {current_mode.upper()} MODE ✓", flush=True)