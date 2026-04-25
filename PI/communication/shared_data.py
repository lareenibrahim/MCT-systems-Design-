# =============================================================================
# shared_data.py — Thread-safe shared data store
# MCT333/MCT344 — Mechatronic Systems Design, Spring 2026, Ain Shams University
# Modular Omni-Wheel Mobile Manipulator
# =============================================================================
#
# SharedData is the single object all four threads read from and write to.
#
# Thread map — who writes what:
#   Thread 2 (Base UART listener) writes:
#       base heartbeat, wheel_velocities, imu_yaw,
#       tof_nav, box_counts, base_fault, base_arrived_flag
#
#   Thread 3 (Arm UART listener) writes:
#       arm heartbeat, pick_status, retrieve_status,
#       place_status, arm_fault, arm_ready_flag
#
#   Thread 4 (Perception) writes:
#       qr_color, qr_coordinate, qr_ready_flag,
#       obstacle_detected, tof_depth
#
#   Thread 1 (State machine) reads everything above.
#       Clears flags after consuming them.
#
#   esp_receiver.py (called from WiFi receive thread) writes:
#       operator command flags (cmd_manual, cmd_autonomous,
#       cmd_auto_pick, cmd_estop)
#
# Lock groups — one lock per group of related variables:
#   _heartbeat_lock : base/arm heartbeat ok flags + last heartbeat timestamps
#   _base_lock      : wheel_velocities, imu_yaw, tof_nav,
#                     box_counts, base_fault, base_arrived_flag
#                     (everything the Base MCU sends lives here)
#   _arm_lock       : pick_status, retrieve_status, place_status,
#                     arm_fault, arm_ready_flag
#                     (everything the Arm MCU sends lives here)
#   _perception_lock: qr_color, qr_coordinate, qr_ready_flag,
#                     obstacle_detected, tof_depth
#   _operator_lock  : cmd_manual, cmd_autonomous, cmd_auto_pick, cmd_estop
#
# Usage:
#   shared = SharedData()
#
#   # Write (from UART listener thread):
#   shared.set_wheel_velocities(0.5, -0.3, 0.1, 0.0)
#
#   # Read (from state machine thread):
#   vels = shared.get_wheel_velocities()
#
# =============================================================================

import threading
import time
from config import (
    COLOR_NONE, COLOR_RED, COLOR_GREEN, COLOR_BLUE,
)


class SharedData:
    """
    Thread-safe shared data store for the RPi controller.

    All public methods acquire and release their lock internally.
    Callers never need to touch locks directly.

    Sections:
        1.  Heartbeat
        2.  Base MCU data  (sensors, box counts, faults, events)
        3.  Arm MCU status (pick, retrieve, place, ready, fault)
        4.  Perception     (QR, obstacle, depth ToF)
        5.  Operator commands  (from ESP controller over WiFi)
        6.  System control (running flag for clean shutdown)
    """

    def __init__(self):

        # ── 1. Heartbeat ──────────────────────────────────────────────────────
        self._heartbeat_lock           = threading.Lock()
        self._base_heartbeat_ok        = False  # True while base MCU is responding
        self._arm_heartbeat_ok         = False  # True while arm MCU is responding
        self._last_base_heartbeat_time = 0.0    # time.time() of last base pong received
        self._last_arm_heartbeat_time  = 0.0    # time.time() of last arm pong received

        # ── 2. Base MCU data ──────────────────────────────────────────────────
        # One lock covers everything the Base MCU sends to the Pi.
        self._base_lock         = threading.Lock()

        # Sensor data
        # Wheel velocities (m/s) — order: front-left, front-right, rear-left, rear-right
        self._wheel_velocities  = [0.0, 0.0, 0.0, 0.0]
        # IMU yaw (degrees) — heading of the robot, 0 = forward
        self._imu_yaw           = 0.0
        # Navigation ToF distances (meters) — front, left, right
        self._tof_nav           = [0.0, 0.0, 0.0]

        # Box counts — per-color, reported by Base MCU storage ToF sensors
        # Three storage compartments, one ToF each, one counter per color
        self._box_counts        = {
            COLOR_RED:   0,
            COLOR_GREEN: 0,
            COLOR_BLUE:  0,
        }

        # Base fault
        self._base_fault_flag   = False  # True when base MCU reported a fault
        self._base_fault_code   = None   # FAULT_* constant from config.py

        # Base event flag — set when MSG_BASE_ARRIVED received
        # Cleared by state machine after consuming (AUTONOMOUS -> RETRIEVING)
        self._base_arrived_flag = False

        # ── 3. Arm MCU status ─────────────────────────────────────────────────
        # One lock covers everything the Arm MCU sends to the Pi.
        self._arm_lock          = threading.Lock()

        # Operation statuses — None means operation still in progress
        # Set by Arm UART listener, cleared by state machine after consuming
        # STATUS_* constants from config.py
        self._pick_status       = None
        self._retrieve_status   = None
        self._place_status      = None

        # Arm fault
        self._arm_fault_flag    = False  # True when arm MCU reported a fault
        self._arm_fault_code    = None   # FAULT_* constant from config.py

        # Arm ready flag — set when MSG_ARM_READY received (homing complete)
        # Reused for re-homing confirmation at start of AUTONOMOUS
        self._arm_ready_flag    = False

        # ── 4. Perception ─────────────────────────────────────────────────────
        self._perception_lock   = threading.Lock()
        self._qr_color          = COLOR_NONE        # color decoded from QR scan
        self._qr_coordinate     = (0.0, 0.0, 0.0)  # (x, y, z) target for arm pick
        self._qr_ready_flag     = False             # True when fresh QR result available
        self._obstacle_detected = False             # True when camera detects obstacle
        self._tof_depth         = 0.0              # Pi ToF depth reading (meters)

        # ── 5. Operator commands ──────────────────────────────────────────────
        # Set by esp_receiver.py when ESP controller sends a command over WiFi.
        # Cleared by state machine after consuming each command.
        self._operator_lock     = threading.Lock()
        self._cmd_manual        = False  # operator requested MANUAL mode
        self._cmd_autonomous    = False  # operator requested AUTONOMOUS mode
        self._cmd_auto_pick     = False  # operator triggered auto-pick
        self._cmd_estop         = False  # operator triggered E-STOP

        # ── 6. System control ─────────────────────────────────────────────────
        # Set to False to signal all threads to exit cleanly on shutdown.
        self._running_lock      = threading.Lock()
        self._running           = True


    # =========================================================================
    # 1. Heartbeat
    # =========================================================================

    def update_base_heartbeat(self):
        """
        Call every time MSG_BASE_HEARTBEAT is received.
        Records current timestamp and marks base as alive.
        Called by: Base UART listener thread.
        """
        with self._heartbeat_lock:
            self._base_heartbeat_ok        = True
            self._last_base_heartbeat_time = time.time()

    def update_arm_heartbeat(self):
        """
        Call every time MSG_ARM_HEARTBEAT is received.
        Records current timestamp and marks arm as alive.
        Called by: Arm UART listener thread.
        """
        with self._heartbeat_lock:
            self._arm_heartbeat_ok        = True
            self._last_arm_heartbeat_time = time.time()

    def get_heartbeat_status(self) -> dict:
        """
        Return current heartbeat status for both MCUs.
        Called by: State machine thread (heartbeat monitor).

        Returns:
            {
                'base_ok'             : bool,
                'arm_ok'              : bool,
                'last_base_heartbeat' : float,  # time.time() timestamp
                'last_arm_heartbeat'  : float,
            }
        """
        with self._heartbeat_lock:
            return {
                'base_ok'             : self._base_heartbeat_ok,
                'arm_ok'              : self._arm_heartbeat_ok,
                'last_base_heartbeat' : self._last_base_heartbeat_time,
                'last_arm_heartbeat'  : self._last_arm_heartbeat_time,
            }

    def set_base_heartbeat_lost(self):
        """
        Mark base MCU as not responding.
        Called by: heartbeat monitor in state machine thread after timeout.
        """
        with self._heartbeat_lock:
            self._base_heartbeat_ok = False

    def set_arm_heartbeat_lost(self):
        """
        Mark arm MCU as not responding.
        Called by: heartbeat monitor in state machine thread after timeout.
        """
        with self._heartbeat_lock:
            self._arm_heartbeat_ok = False


    # =========================================================================
    # 2. Base MCU data
    # =========================================================================

    # ── Sensor data ───────────────────────────────────────────────────────────

    def set_wheel_velocities(self, fl: float, fr: float,
                              rl: float, rr: float):
        """
        Store four wheel velocities from MSG_ENCODER_DATA.

        Args:
            fl : front-left  wheel velocity (m/s)
            fr : front-right wheel velocity (m/s)
            rl : rear-left   wheel velocity (m/s)
            rr : rear-right  wheel velocity (m/s)

        Called by: Base UART listener thread.
        """
        with self._base_lock:
            self._wheel_velocities = [fl, fr, rl, rr]

    def get_wheel_velocities(self) -> list[float]:
        """
        Return latest wheel velocities [fl, fr, rl, rr] in m/s.
        Called by: State machine / logger.
        """
        with self._base_lock:
            return list(self._wheel_velocities)

    def set_imu_yaw(self, yaw: float):
        """
        Store yaw angle from MSG_IMU_DATA.

        Args:
            yaw : robot heading in degrees (0 = forward)

        Called by: Base UART listener thread.
        """
        with self._base_lock:
            self._imu_yaw = yaw

    def get_imu_yaw(self) -> float:
        """
        Return latest yaw angle in degrees.
        Called by: State machine (navigation, alignment).
        """
        with self._base_lock:
            return self._imu_yaw

    def set_tof_nav(self, front: float, left: float, right: float):
        """
        Store three navigation ToF distances from MSG_TOF_NAV_DATA.

        Args:
            front : distance to obstacle ahead (meters)
            left  : distance to left wall/obstacle (meters)
            right : distance to right wall/obstacle (meters)

        Called by: Base UART listener thread.
        """
        with self._base_lock:
            self._tof_nav = [front, left, right]

    def get_tof_nav(self) -> list[float]:
        """
        Return latest navigation ToF distances [front, left, right] in meters.
        Called by: State machine (obstacle avoidance, wall alignment).
        """
        with self._base_lock:
            return list(self._tof_nav)

    # ── Box counts ────────────────────────────────────────────────────────────

    def set_box_counts(self, red: int, green: int, blue: int):
        """
        Store per-color box counts from MSG_BOX_COUNT.

        The Base MCU maintains three storage compartments, one ToF per compartment.
        Each ToF detects distance increases (by one box width) and increments
        its counter. The Pi receives the final counts here.

        Args:
            red, green, blue : current box count for each color

        Called by: Base UART listener thread.
        """
        with self._base_lock:
            self._box_counts[COLOR_RED]   = red
            self._box_counts[COLOR_GREEN] = green
            self._box_counts[COLOR_BLUE]  = blue

    def get_box_counts(self) -> dict:
        """
        Return all box counts as {COLOR_RED: n, COLOR_GREEN: n, COLOR_BLUE: n}.
        Called by: State machine (RETRIEVING entry).
        """
        with self._base_lock:
            return dict(self._box_counts)

    def get_box_count_for_color(self, color: int) -> int:
        """
        Return box count for one specific color.

        Args:
            color : COLOR_RED / COLOR_GREEN / COLOR_BLUE

        Returns:
            Count as int, or 0 if color not recognised.

        Called by: State machine thread.
        """
        with self._base_lock:
            return self._box_counts.get(color, 0)

    # ── Base fault ────────────────────────────────────────────────────────────

    def set_base_fault(self, fault_code: int):
        """
        Record a fault reported by the Base MCU (MSG_BASE_FAULT).

        Args:
            fault_code : FAULT_* constant from config.py

        Called by: Base UART listener thread.
        """
        with self._base_lock:
            self._base_fault_flag = True
            self._base_fault_code = fault_code

    def get_and_clear_base_fault(self) -> dict:
        """
        Return base fault state and clear the flag.

        Returns:
            {
                'fault' : bool,
                'code'  : int or None,
            }

        Called by: State machine thread every poll cycle.
        """
        with self._base_lock:
            result = {
                'fault' : self._base_fault_flag,
                'code'  : self._base_fault_code,
            }
            self._base_fault_flag = False
            self._base_fault_code = None
            return result

    # ── Base event flag ───────────────────────────────────────────────────────

    def set_base_arrived(self):
        """
        Signal that the base reached its target position (MSG_BASE_ARRIVED).
        Called by: Base UART listener thread.
        """
        with self._base_lock:
            self._base_arrived_flag = True

    def get_and_clear_base_arrived(self) -> bool:
        """
        Return base arrived flag and clear it.
        Called by: State machine thread (AUTONOMOUS -> RETRIEVING transition).
        """
        with self._base_lock:
            flag                    = self._base_arrived_flag
            self._base_arrived_flag = False
            return flag


    # =========================================================================
    # 3. Arm MCU status
    # =========================================================================

    # ── Operation statuses ────────────────────────────────────────────────────

    def set_pick_status(self, status: int):
        """
        Store pick result from MSG_PICK_STATUS.

        Args:
            status : STATUS_SUCCESS / STATUS_FAIL / STATUS_RETRY / STATUS_TIMEOUT

        Called by: Arm UART listener thread.
        """
        with self._arm_lock:
            self._pick_status = status

    def get_and_clear_pick_status(self) -> int | None:
        """
        Return pick status and immediately clear it.

        Returns None if no result has arrived yet (arm still operating).
        Clearing prevents the same result being consumed on the next poll cycle.

        Called by: State machine thread (PICKING state).
        """
        with self._arm_lock:
            status            = self._pick_status
            self._pick_status = None
            return status

    def set_retrieve_status(self, status: int):
        """
        Store retrieve result from MSG_RETRIEVE_STATUS.
        Called by: Arm UART listener thread.
        """
        with self._arm_lock:
            self._retrieve_status = status

    def get_and_clear_retrieve_status(self) -> int | None:
        """
        Return retrieve status and clear it.
        Called by: State machine thread (RETRIEVING state).
        """
        with self._arm_lock:
            status                = self._retrieve_status
            self._retrieve_status = None
            return status

    def set_place_status(self, status: int):
        """
        Store place result from MSG_PLACE_STATUS.
        Called by: Arm UART listener thread.
        """
        with self._arm_lock:
            self._place_status = status

    def get_and_clear_place_status(self) -> int | None:
        """
        Return place status and clear it.
        Called by: State machine thread (PLACING state).
        """
        with self._arm_lock:
            status             = self._place_status
            self._place_status = None
            return status

    # ── Arm fault ─────────────────────────────────────────────────────────────

    def set_arm_fault(self, fault_code: int):
        """
        Record a fault reported by the Arm MCU (MSG_ARM_FAULT).

        Args:
            fault_code : FAULT_* constant from config.py

        Called by: Arm UART listener thread.
        """
        with self._arm_lock:
            self._arm_fault_flag = True
            self._arm_fault_code = fault_code

    def get_and_clear_arm_fault(self) -> dict:
        """
        Return arm fault state and clear the flag.

        Returns:
            {
                'fault' : bool,
                'code'  : int or None,
            }

        Called by: State machine thread every poll cycle.
        """
        with self._arm_lock:
            result = {
                'fault' : self._arm_fault_flag,
                'code'  : self._arm_fault_code,
            }
            self._arm_fault_flag = False
            self._arm_fault_code = None
            return result

    # ── Arm ready flag ────────────────────────────────────────────────────────

    def set_arm_ready(self):
        """
        Mark arm as homed and ready (MSG_ARM_READY received).
        Reused for both initial homing in INIT and re-homing at AUTONOMOUS entry.
        Called by: Arm UART listener thread.
        """
        with self._arm_lock:
            self._arm_ready_flag = True

    def get_and_clear_arm_ready(self) -> bool:
        """
        Return arm ready flag and clear it.
        Called by: State machine thread (INIT state, AUTONOMOUS entry).
        """
        with self._arm_lock:
            flag                 = self._arm_ready_flag
            self._arm_ready_flag = False
            return flag


    # =========================================================================
    # 4. Perception
    # =========================================================================

    def set_qr_result(self, color: int, x: float, y: float, z: float):
        """
        Store a decoded QR scan result from the camera pipeline.

        Args:
            color    : COLOR_RED / COLOR_GREEN / COLOR_BLUE
            x, y, z  : target coordinate for the arm pick operation (meters)

        Called by: Perception thread (qr_detection.py).
        """
        with self._perception_lock:
            self._qr_color      = color
            self._qr_coordinate = (x, y, z)
            self._qr_ready_flag = True

    def get_and_clear_qr_result(self) -> dict:
        """
        Return QR scan result and clear the ready flag.

        Returns:
            {
                'ready'      : bool,
                'color'      : int  (COLOR_* constant),
                'coordinate' : (x, y, z) tuple in meters,
            }

        Called by: State machine thread (PICKING state entry).
        """
        with self._perception_lock:
            result = {
                'ready'      : self._qr_ready_flag,
                'color'      : self._qr_color,
                'coordinate' : self._qr_coordinate,
            }
            self._qr_ready_flag = False
            return result

    def set_obstacle_detected(self, detected: bool):
        """
        Update obstacle detection state from camera pipeline.

        Args:
            detected : True if obstacle is in the path, False when clear

        Called by: Perception thread (obstacle_detection.py).
        """
        with self._perception_lock:
            self._obstacle_detected = detected

    def get_obstacle_detected(self) -> bool:
        """
        Return current obstacle detection state.
        Called by: State machine thread (AUTONOMOUS — path replanning).
        """
        with self._perception_lock:
            return self._obstacle_detected

    def set_tof_depth(self, depth: float):
        """
        Store latest depth reading from the Pi-side ToF sensor.

        Args:
            depth : distance to target surface in meters

        Called by: Perception thread (tof_sensor.py).
        """
        with self._perception_lock:
            self._tof_depth = depth

    def get_tof_depth(self) -> float:
        """
        Return latest Pi ToF depth in meters.
        Called by: State machine thread (PICKING, PLACING — arm alignment).
        """
        with self._perception_lock:
            return self._tof_depth


    # =========================================================================
    # 5. Operator commands  (from ESP controller over WiFi)
    # =========================================================================
    # set_cmd_* methods are called by esp_receiver.py when the ESP sends a command.
    # get_and_clear_cmd_* methods are called by the state machine thread.
    # Clearing after reading prevents the same command firing on the next poll.

    def set_cmd_manual(self):
        """Operator requested switch to MANUAL mode."""
        with self._operator_lock:
            self._cmd_manual = True

    def get_and_clear_cmd_manual(self) -> bool:
        with self._operator_lock:
            flag             = self._cmd_manual
            self._cmd_manual = False
            return flag

    def set_cmd_autonomous(self):
        """Operator requested switch to AUTONOMOUS mode."""
        with self._operator_lock:
            self._cmd_autonomous = True

    def get_and_clear_cmd_autonomous(self) -> bool:
        with self._operator_lock:
            flag                 = self._cmd_autonomous
            self._cmd_autonomous = False
            return flag

    def set_cmd_auto_pick(self):
        """Operator triggered auto-pick — cube is in position, start picking sequence."""
        with self._operator_lock:
            self._cmd_auto_pick = True

    def get_and_clear_cmd_auto_pick(self) -> bool:
        with self._operator_lock:
            flag                = self._cmd_auto_pick
            self._cmd_auto_pick = False
            return flag

    def set_cmd_estop(self):
        """Operator triggered E-STOP."""
        with self._operator_lock:
            self._cmd_estop = True

    def get_and_clear_cmd_estop(self) -> bool:
        with self._operator_lock:
            flag            = self._cmd_estop
            self._cmd_estop = False
            return flag


    # =========================================================================
    # 6. System control
    # =========================================================================

    def is_running(self) -> bool:
        """
        Return True while the system should keep running.
        All threads check this in their loop condition.
        Called by: all threads every cycle.
        """
        with self._running_lock:
            return self._running

    def request_shutdown(self):
        """
        Signal all threads to exit their loops cleanly.
        Called by: main.py on keyboard interrupt or fatal error.
        """
        with self._running_lock:
            self._running = False