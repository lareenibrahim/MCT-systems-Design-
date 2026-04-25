# =============================================================================
# base_uart.py — Base MCU UART communication
# MCT333/MCT344 — Mechatronic Systems Design, Spring 2026, Ain Shams University
# Modular Omni-Wheel Mobile Manipulator
# =============================================================================
#
# Owns two internal threads:
#   - Listener thread      : blocks on serial.read(1), feeds PacketReader,
#                            parses complete packets, writes results to SharedData
#   - Heartbeat sender     : sleeps HEARTBEAT_INTERVAL_MS, sends ping, repeats
#
# Public interface (called from thread_manager.py or state machine):
#   base = BaseUART(shared_data)
#   base.connect()                          # open serial port, start threads
#   base.send_idle()                        # send MSG_SET_IDLE
#   base.send_manual()                      # send MSG_SET_MANUAL
#   base.send_autonomous()                  # send MSG_SET_AUTONOMOUS
#   base.send_charging()                    # send MSG_SET_CHARGING
#   base.send_estop()                       # send MSG_ESTOP
#   base.send_velocity(vx, vy, omega)       # send MSG_VELOCITY_CMD
#   base.send_query_box_count()             # send MSG_QUERY_BOX_COUNT
#   base.disconnect()                       # stop threads, close port
#
# Logging:
#   Two levels controlled by verbose flag in connect():
#     Normal  — port open/close, faults, errors, timeouts, arrivals
#     Verbose — every packet sent and received, all sensor values
#
# =============================================================================

import threading
import time
import serial
import logging

from config import (
    BASE_MCU_PORT, BAUD_RATE, UART_TIMEOUT_S,
    HEARTBEAT_INTERVAL_MS,
    START_BYTE,
    MSG_HEARTBEAT_PING,
    MSG_SET_IDLE, MSG_SET_MANUAL, MSG_SET_AUTONOMOUS,
    MSG_SET_CHARGING, MSG_ESTOP, MSG_VELOCITY_CMD, MSG_QUERY_BOX_COUNT,
    MSG_BASE_HEARTBEAT, MSG_ENCODER_DATA, MSG_IMU_DATA,
    MSG_TOF_NAV_DATA, MSG_BOX_COUNT, MSG_BASE_FAULT, MSG_BASE_ARRIVED,
    FAULT_COMM_ERROR,
)
from communication.protocol import (
    build_packet, PacketReader, parse_packet,
    encode_signed, decode_signed, decode_unsigned,
)
from communication.shared_data import SharedData

# Module-level logger — name appears in every log line as [base_uart]
logger = logging.getLogger('base_uart')


# =============================================================================
# MSG_ID → human-readable name  (used in verbose logging)
# =============================================================================

_MSG_NAMES = {
    MSG_HEARTBEAT_PING  : 'MSG_HEARTBEAT_PING',
    MSG_SET_IDLE        : 'MSG_SET_IDLE',
    MSG_SET_MANUAL      : 'MSG_SET_MANUAL',
    MSG_VELOCITY_CMD    : 'MSG_VELOCITY_CMD',
    MSG_SET_AUTONOMOUS  : 'MSG_SET_AUTONOMOUS',
    MSG_SET_CHARGING    : 'MSG_SET_CHARGING',
    MSG_ESTOP           : 'MSG_ESTOP',
    MSG_QUERY_BOX_COUNT : 'MSG_QUERY_BOX_COUNT',
    MSG_BASE_HEARTBEAT  : 'MSG_BASE_HEARTBEAT',
    MSG_ENCODER_DATA    : 'MSG_ENCODER_DATA',
    MSG_IMU_DATA        : 'MSG_IMU_DATA',
    MSG_TOF_NAV_DATA    : 'MSG_TOF_NAV_DATA',
    MSG_BOX_COUNT       : 'MSG_BOX_COUNT',
    MSG_BASE_FAULT      : 'MSG_BASE_FAULT',
    MSG_BASE_ARRIVED    : 'MSG_BASE_ARRIVED',
}

def _msg_name(msg_id: int) -> str:
    """Return human-readable name for a MSG_ID, or hex string if unknown."""
    return _MSG_NAMES.get(msg_id, f'UNKNOWN(0x{msg_id:02X})')


# =============================================================================
# BaseUART class
# =============================================================================

class BaseUART:
    """
    Manages all UART communication with the Base MCU.

    Owns two threads internally:
        _listener_thread       : receives and parses incoming packets
        _heartbeat_thread      : sends periodic heartbeat pings

    Both threads share the same serial port object. serial.write() is
    protected by _write_lock so the heartbeat sender and any send_*()
    call from the state machine cannot write simultaneously.

    Args:
        shared : SharedData instance — all parsed results are written here
        verbose: if True, log every packet sent and received (debug mode)
                 if False, log only errors, faults, and important events
    """

    def __init__(self, shared: SharedData, verbose: bool = False):
        self._shared        = shared
        self._verbose       = verbose
        self._port          = None          # serial.Serial object, set in connect()
        self._write_lock    = threading.Lock()
        self._reader        = PacketReader()

        self._listener_thread   = None
        self._heartbeat_thread  = None

        # _running controls both internal threads.
        # Set to False in disconnect() to signal clean exit.
        self._running = False


    # =========================================================================
    # Public interface
    # =========================================================================

    def connect(self) -> bool:
        """
        Open the serial port and start both internal threads.

        Returns:
            True  if port opened successfully and threads started
            False if the port could not be opened (logs the error)

        Called by: thread_manager.py during system startup.
        """
        try:
            self._port = serial.Serial(
                port      = BASE_MCU_PORT,
                baudrate  = BAUD_RATE,
                timeout   = UART_TIMEOUT_S,   # read() returns after this if no byte arrives
            )
            logger.info(f'[BASE] Port opened: {BASE_MCU_PORT} at {BAUD_RATE} baud')
        except serial.SerialException as e:
            logger.critical(f'[BASE] Failed to open port {BASE_MCU_PORT}: {e}')
            return False

        self._running = True

        self._listener_thread = threading.Thread(
            target = self._listen_loop,
            name   = 'base_listener',
            daemon = True,    # exits automatically if main program exits
        )
        self._heartbeat_thread = threading.Thread(
            target = self._heartbeat_loop,
            name   = 'base_heartbeat',
            daemon = True,
        )

        self._listener_thread.start()
        self._heartbeat_thread.start()
        logger.info('[BASE] Listener and heartbeat threads started')
        return True

    def disconnect(self):
        """
        Signal both threads to stop, wait for them to finish, close the port.
        Called by: thread_manager.py during system shutdown.
        """
        logger.info('[BASE] Disconnecting...')
        self._running = False

        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=2.0)

        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=2.0)

        if self._port and self._port.is_open:
            self._port.close()
            logger.info(f'[BASE] Port {BASE_MCU_PORT} closed')


    # =========================================================================
    # Packet send methods  (Pi → Base MCU)
    # =========================================================================
    # All send_*() methods follow the same pattern:
    #   1. Build the packet bytes using build_packet() + encode helpers
    #   2. Call _write() which acquires the write lock and calls serial.write()
    #   3. Log at verbose level if enabled

    def send_idle(self):
        """Tell Base MCU to enter IDLE and stop all motors."""
        self._write(build_packet(MSG_SET_IDLE), MSG_SET_IDLE)

    def send_manual(self):
        """Tell Base MCU to enter MANUAL and enable motor drivers."""
        self._write(build_packet(MSG_SET_MANUAL), MSG_SET_MANUAL)

    def send_autonomous(self):
        """Tell Base MCU to enter AUTONOMOUS mode."""
        self._write(build_packet(MSG_SET_AUTONOMOUS), MSG_SET_AUTONOMOUS)

    def send_charging(self):
        """Tell Base MCU to enter CHARGING mode."""
        self._write(build_packet(MSG_SET_CHARGING), MSG_SET_CHARGING)

    def send_estop(self):
        """Send emergency stop to Base MCU — highest priority command."""
        self._write(build_packet(MSG_ESTOP), MSG_ESTOP)

    def send_query_box_count(self):
        """Request current per-color box counts from Base MCU."""
        self._write(build_packet(MSG_QUERY_BOX_COUNT), MSG_QUERY_BOX_COUNT)

    def send_velocity(self, vx: float, vy: float, omega: float):
        """
        Send velocity command to Base MCU.

        Args:
            vx    : forward velocity (m/s), signed — positive = forward
            vy    : lateral velocity (m/s), signed — positive = left
            omega : rotational velocity (rad/s), signed — positive = CCW

        Encoding: each float scaled x1000, split into 2 signed bytes → 6 bytes payload.
        Called every 50ms from state machine during MANUAL and AUTONOMOUS.
        """
        vx_h,  vx_l  = encode_signed(vx)
        vy_h,  vy_l  = encode_signed(vy)
        om_h,  om_l  = encode_signed(omega)
        data   = bytes([vx_h, vx_l, vy_h, vy_l, om_h, om_l])
        packet = build_packet(MSG_VELOCITY_CMD, data)
        self._write(packet, MSG_VELOCITY_CMD)

        if self._verbose:
            logger.debug(
                f'[BASE] TX {_msg_name(MSG_VELOCITY_CMD)} '
                f'vx={vx:.3f} vy={vy:.3f} omega={omega:.3f}'
            )


    # =========================================================================
    # Internal — write
    # =========================================================================

    def _write(self, packet: bytes, msg_id: int):
        """
        Write a packet to the serial port under the write lock.

        The write lock prevents the heartbeat sender and a state machine
        send_*() call from writing at the same time, which would corrupt
        both packets on the wire.

        On SerialException: triggers a FAULT_COMM_ERROR in SharedData
        immediately — the state machine will detect this on its next poll.

        Args:
            packet : complete packet bytes from build_packet()
            msg_id : MSG_ID of the packet (used for logging only)
        """
        try:
            with self._write_lock:
                self._port.write(packet)
            if self._verbose and msg_id != MSG_HEARTBEAT_PING:
                # Heartbeat pings are excluded from verbose TX log to avoid spam
                logger.debug(f'[BASE] TX {_msg_name(msg_id)} '
                             f'({len(packet)} bytes) [{packet.hex(" ")}]')
        except serial.SerialException as e:
            logger.error(f'[BASE] TX FAILED for {_msg_name(msg_id)}: {e}')
            self._shared.set_base_fault(FAULT_COMM_ERROR)


    # =========================================================================
    # Internal — listener thread
    # =========================================================================

    def _listen_loop(self):
        """
        Listener thread entry point.

        Blocks on serial.read(1) continuously. Each byte is fed into
        PacketReader. When PacketReader returns a complete raw packet,
        parse_packet() validates it and _handle_message() processes it.

        Exits cleanly when self._running is set to False and the port closes.
        """
        logger.info('[BASE] Listener thread running')

        while self._running:
            try:
                raw_byte = self._port.read(1)   # blocks up to UART_TIMEOUT_S
                if not raw_byte:
                    # Timeout elapsed with no byte — normal, just loop again
                    continue

                raw_packet = self._reader.feed(raw_byte[0])
                if raw_packet is None:
                    # Packet not yet complete — keep reading
                    continue

                # Complete packet assembled — parse and handle
                result = parse_packet(raw_packet)
                if result.success:
                    self._handle_message(result.msg_id, result.data)
                else:
                    logger.warning(f'[BASE] RX bad packet: {result.error}')
                    # PacketReader already reset itself inside parse_packet call

            except serial.SerialException as e:
                if self._running:
                    # Only log and fault if we didn't intentionally disconnect
                    logger.error(f'[BASE] Serial read error: {e}')
                    self._shared.set_base_fault(FAULT_COMM_ERROR)
                break

        logger.info('[BASE] Listener thread exited')


    # =========================================================================
    # Internal — message handler
    # =========================================================================

    def _handle_message(self, msg_id: int, data: bytes):
        """
        Route a validated incoming packet to the correct handler.

        Called from the listener thread. Writes results to SharedData.
        Each handler is responsible for decoding its specific payload.

        Args:
            msg_id : validated MSG_ID from parse_packet()
            data   : validated payload bytes (may be empty)
        """
        if self._verbose:
            logger.debug(f'[BASE] RX {_msg_name(msg_id)} '
                        f'data=[{data.hex(" ")}]')

        if msg_id == MSG_BASE_HEARTBEAT:
            self._handle_heartbeat()

        elif msg_id == MSG_ENCODER_DATA:
            self._handle_encoder_data(data)

        elif msg_id == MSG_IMU_DATA:
            self._handle_imu_data(data)

        elif msg_id == MSG_TOF_NAV_DATA:
            self._handle_tof_nav_data(data)

        elif msg_id == MSG_BOX_COUNT:
            self._handle_box_count(data)

        elif msg_id == MSG_BASE_FAULT:
            self._handle_fault(data)

        elif msg_id == MSG_BASE_ARRIVED:
            self._handle_base_arrived()

        else:
            logger.warning(f'[BASE] RX unknown MSG_ID: 0x{msg_id:02X}')


    # =========================================================================
    # Internal — individual message handlers
    # =========================================================================

    def _handle_heartbeat(self):
        """
        MSG_BASE_HEARTBEAT received — base MCU is alive.
        Updates SharedData timestamp. No payload.
        """
        self._shared.update_base_heartbeat()
        if self._verbose:
            logger.debug('[BASE] RX MSG_BASE_HEARTBEAT — base alive')

    def _handle_encoder_data(self, data: bytes):
        """
        MSG_ENCODER_DATA received.

        Payload: 8 bytes — 4 wheel velocities, each 2 signed bytes (scaled x1000)
            [FL_H | FL_L | FR_H | FR_L | RL_H | RL_L | RR_H | RR_L]

        Args:
            data : 8-byte payload
        """
        if len(data) != 8:
            logger.warning(f'[BASE] MSG_ENCODER_DATA bad length: {len(data)} (expected 8)')
            return

        fl = decode_signed(data[0], data[1])
        fr = decode_signed(data[2], data[3])
        rl = decode_signed(data[4], data[5])
        rr = decode_signed(data[6], data[7])

        self._shared.set_wheel_velocities(fl, fr, rl, rr)

        if self._verbose:
            logger.debug(
                f'[BASE] RX MSG_ENCODER_DATA '
                f'FL={fl:.3f} FR={fr:.3f} RL={rl:.3f} RR={rr:.3f} m/s'
            )

    def _handle_imu_data(self, data: bytes):
        """
        MSG_IMU_DATA received.

        Payload: 2 bytes — yaw angle in degrees, signed (scaled x1000)
            [YAW_H | YAW_L]

        Args:
            data : 2-byte payload
        """
        if len(data) != 2:
            logger.warning(f'[BASE] MSG_IMU_DATA bad length: {len(data)} (expected 2)')
            return

        yaw = decode_signed(data[0], data[1])
        self._shared.set_imu_yaw(yaw)

        if self._verbose:
            logger.debug(f'[BASE] RX MSG_IMU_DATA yaw={yaw:.3f} deg')

    def _handle_tof_nav_data(self, data: bytes):
        """
        MSG_TOF_NAV_DATA received.

        Payload: 6 bytes — 3 navigation ToF distances (front, left, right)
                           each 2 unsigned bytes (scaled x1000, always positive)
            [FRONT_H | FRONT_L | LEFT_H | LEFT_L | RIGHT_H | RIGHT_L]

        Args:
            data : 6-byte payload
        """
        if len(data) != 6:
            logger.warning(f'[BASE] MSG_TOF_NAV_DATA bad length: {len(data)} (expected 6)')
            return

        front = decode_unsigned(data[0], data[1])
        left  = decode_unsigned(data[2], data[3])
        right = decode_unsigned(data[4], data[5])

        self._shared.set_tof_nav(front, left, right)

        if self._verbose:
            logger.debug(
                f'[BASE] RX MSG_TOF_NAV_DATA '
                f'front={front:.3f} left={left:.3f} right={right:.3f} m'
            )

    def _handle_box_count(self, data: bytes):
        """
        MSG_BOX_COUNT received.

        Payload: 3 bytes — one byte per color, raw integer counts (no scaling)
            [RED_COUNT | GREEN_COUNT | BLUE_COUNT]

        Args:
            data : 3-byte payload
        """
        if len(data) != 3:
            logger.warning(f'[BASE] MSG_BOX_COUNT bad length: {len(data)} (expected 3)')
            return

        red   = data[0]
        green = data[1]
        blue  = data[2]

        self._shared.set_box_counts(red, green, blue)

        if self._verbose:
            logger.debug(
                f'[BASE] RX MSG_BOX_COUNT '
                f'red={red} green={green} blue={blue}'
            )

    def _handle_fault(self, data: bytes):
        """
        MSG_BASE_FAULT received — Base MCU is reporting a hardware fault.

        Payload: 1 byte — fault code (FAULT_* constant from config.py)
            [FAULT_CODE]

        Always logged at ERROR level regardless of verbose setting.

        Args:
            data : 1-byte payload
        """
        if len(data) != 1:
            logger.warning(f'[BASE] MSG_BASE_FAULT bad length: {len(data)} (expected 1)')
            return

        fault_code = data[0]
        self._shared.set_base_fault(fault_code)
        logger.error(f'[BASE] RX MSG_BASE_FAULT code=0x{fault_code:02X}')

    def _handle_base_arrived(self):
        """
        MSG_BASE_ARRIVED received — base reached its navigation target.

        No payload. Sets base_arrived_flag in SharedData.
        State machine will detect this on next poll and transition
        AUTONOMOUS -> RETRIEVING.

        Always logged at INFO level regardless of verbose setting.
        """
        self._shared.set_base_arrived()
        logger.info('[BASE] RX MSG_BASE_ARRIVED — base reached target position')


    # =========================================================================
    # Internal — heartbeat sender thread
    # =========================================================================

    def _heartbeat_loop(self):
        """
        Heartbeat sender thread entry point.

        Sends MSG_HEARTBEAT_PING to Base MCU every HEARTBEAT_INTERVAL_MS.
        Sleeps between sends — does not touch incoming data at all.

        Exits cleanly when self._running is set to False.
        """
        logger.info('[BASE] Heartbeat sender thread running')
        interval_s = HEARTBEAT_INTERVAL_MS / 1000.0

        while self._running:
            self._write(build_packet(MSG_HEARTBEAT_PING), MSG_HEARTBEAT_PING)
            if self._verbose:
                logger.debug('[BASE] TX MSG_HEARTBEAT_PING')
            time.sleep(interval_s)

        logger.info('[BASE] Heartbeat sender thread exited')