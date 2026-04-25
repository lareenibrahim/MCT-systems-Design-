# =============================================================================
# config.py — System-wide constants and message IDs
# =============================================================================
 
# ── UART Ports ────────────────────────────────────────────────────────────────
BASE_MCU_PORT           = "/dev/ttyAMA2"
ARM_MCU_PORT            = "/dev/ttyAMA3"
BAUD_RATE               = 115200
 
# ── Heartbeat ─────────────────────────────────────────────────────────────────
HEARTBEAT_INTERVAL_MS   = 100      # Pi pings both MCUs every 100ms
HEARTBEAT_TIMEOUT_MS    = 300      # Fault triggered if no response within 300ms
 
# ── Protocol ──────────────────────────────────────────────────────────────────
START_BYTE              = 0xFF     # Every packet begins with this byte
 
# ── Timeouts ──────────────────────────────────────────────────────────────────
UART_TIMEOUT_S          = 0.1     # Max time pyserial waits for next byte before returning — prevents read() blocking forever
PICK_TIMEOUT_S          = 10.0    # Max time Pi waits for MSG_PICK_STATUS after sending MSG_PICK_CMD
RETRIEVE_TIMEOUT_S      = 10.0    # Max time Pi waits for MSG_RETRIEVE_STATUS after sending MSG_RETRIEVE_CMD
PLACE_TIMEOUT_S         = 10.0    # Max time Pi waits for MSG_PLACE_STATUS after sending MSG_PLACE_CMD
 
# ── State Machine ─────────────────────────────────────────────────────────────
STATE_MACHINE_POLL_MS   = 50       # State machine loop sleep interval — checks SharedData flags every 50ms for transition conditions
 
 
# =============================================================================
# Pi → Base MCU  (sent over BASE_MCU_PORT)
# =============================================================================
MSG_HEARTBEAT_PING      = 0x01    # Are you alive? (ping)
MSG_SET_IDLE            = 0x02    # Enter IDLE, stop all motors
MSG_SET_MANUAL          = 0x03    # Enter MANUAL, enable motor drivers
MSG_VELOCITY_CMD        = 0x04    # Velocity command: vx, vy, omega (sent every 50ms in MANUAL/AUTONOMOUS)
MSG_SET_AUTONOMOUS      = 0x05    # Enter AUTONOMOUS, wait for velocity cmds from Pi
MSG_SET_CHARGING        = 0x06    # Enter CHARGING, disable all outputs
MSG_ESTOP               = 0x07    # Emergency stop — cut everything immediately
MSG_QUERY_BOX_COUNT     = 0x08    # Request current per-color box counts from storage ToF
 
 
# =============================================================================
# Base MCU → Pi  (received over BASE_MCU_PORT)
# =============================================================================
MSG_BASE_HEARTBEAT      = 0x10    # Heartbeat response to MSG_HEARTBEAT_PING
MSG_ENCODER_DATA        = 0x11    # 4 wheel velocities (scaled integers)
MSG_IMU_DATA            = 0x12    # Roll, pitch, yaw (scaled integers)
MSG_TOF_NAV_DATA        = 0x13    # 3 navigation ToF distances (scaled integers)
MSG_BOX_COUNT           = 0x14    # Per-color box counts: red, green, blue
MSG_BASE_FAULT          = 0x15    # Fault code from Base MCU
MSG_BASE_ARRIVED        = 0x16    # Base reached target position during AUTONOMOUS
 
 
# =============================================================================
# Pi → Arm MCU  (sent over ARM_MCU_PORT)
# =============================================================================
MSG_ARM_HEARTBEAT_PING  = 0x01    # Are you alive? (same ID as base ping — different port)
MSG_ARM_SET_IDLE        = 0x20    # Enter IDLE, hold home position
MSG_ARM_SET_MANUAL      = 0x21    # Enter MANUAL, accept NRF joint commands
MSG_ARM_SET_CHARGING    = 0x22    # Enter CHARGING, fold arm to safe position
MSG_PICK_CMD            = 0x23    # Pick cube — includes target coordinate
MSG_RETRIEVE_CMD        = 0x24    # Retrieve box — includes color + coordinate
MSG_PLACE_CMD           = 0x25    # Place cube — includes target coordinate
MSG_ARM_ESTOP           = 0x07    # Emergency stop — same ID as base estop, different port
# NOTE: MSG_ARM_SET_IDLE (0x20) is used both for first entry into IDLE and
#       for returning to IDLE from any active state — no separate ID needed
 
 
# =============================================================================
# Arm MCU → Pi  (received over ARM_MCU_PORT)
# =============================================================================
MSG_ARM_HEARTBEAT       = 0x30    # Heartbeat response to MSG_ARM_HEARTBEAT_PING
MSG_PICK_STATUS         = 0x31    # Pick result: SUCCESS / FAIL / RETRY / TIMEOUT
MSG_RETRIEVE_STATUS     = 0x32    # Retrieve result: SUCCESS / FAIL / RETRY / TIMEOUT
MSG_PLACE_STATUS        = 0x33    # Place result: SUCCESS / FAIL / RETRY / TIMEOUT
MSG_ARM_FAULT           = 0x34    # Fault code from Arm MCU
MSG_ARM_READY           = 0x35    # Arm homing complete, ready to accept commands
#                                   also reused as re-homing confirmation (e.g. start of AUTONOMOUS)
 
 
# =============================================================================
# Status Codes  (used in MSG_PICK_STATUS, MSG_RETRIEVE_STATUS, MSG_PLACE_STATUS)
# =============================================================================
STATUS_SUCCESS          = 0x00    # Operation completed successfully
STATUS_FAIL             = 0x01    # Operation failed, do not retry
STATUS_RETRY            = 0x02    # Operation failed, retry recommended
STATUS_TIMEOUT          = 0x03    # Operation did not complete within timeout
 
 
# =============================================================================
# Fault Codes  (used in MSG_BASE_FAULT, MSG_ARM_FAULT)
# =============================================================================
FAULT_HEARTBEAT_TIMEOUT = 0x01    # No heartbeat response within HEARTBEAT_TIMEOUT_MS
FAULT_SENSOR_ERROR      = 0x02    # Sensor read failure (IMU, ToF, encoder)
FAULT_MOTOR_ERROR       = 0x03    # Motor driver fault (overcurrent, stall, etc.)
FAULT_COMM_ERROR        = 0x04    # UART communication error


# ── Box color codes ────────────────────────────────────────────────────────────
COLOR_NONE  = 0x00    # No color / unscanned
COLOR_RED   = 0x01    # Red box
COLOR_GREEN = 0x02    # Green box
COLOR_BLUE  = 0x03    # Blue box
