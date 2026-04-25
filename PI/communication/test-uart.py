import serial
import time
from config import BASE_MCU_PORT, BAUD_RATE, UART_TIMEOUT_S, MSG_HEARTBEAT_PING, MSG_BASE_HEARTBEAT
from communication.protocol import build_packet, PacketReader, parse_packet

# Open port
port   = serial.Serial(BASE_MCU_PORT, BAUD_RATE, timeout=UART_TIMEOUT_S)
reader = PacketReader()

print(f"Port opened: {BASE_MCU_PORT}")

# Send a heartbeat ping
packet = build_packet(MSG_HEARTBEAT_PING)
port.write(packet)
print(f"Sent: {packet.hex(' ')}")

# Listen for response for 2 seconds
start = time.time()
while time.time() - start < 2.0:
    byte = port.read(1)
    if not byte:
        continue
    raw_packet = reader.feed(byte[0])
    if raw_packet is None:
        continue
    result = parse_packet(raw_packet)
    if result.success:
        print(f"Received MSG_ID=0x{result.msg_id:02X} data={result.data.hex()}")
    else:
        print(f"Bad packet: {result.error}")

port.close()