import time
import board
import busio
import digitalio
from adafruit_mcp2515.canio import Message
from adafruit_mcp2515 import MCP2515
import adafruit_vl53l1x

# ==========================================
# CONFIGURATION
# ==========================================

# FRC standard CAN baudrate is 1 Mbps
CAN_BAUDRATE = 1000000

# FRC Team Use CAN ID (Device Type 10, Manufacturer 8 - Team Use)
# Extended ID format is recommended on the FRC CAN bus to avoid collisions.
CAN_ID_DATA = 0x1E040000  # ID for sending distance data
CAN_ID_CONFIG = 0x1E040001  # ID for receiving configuration
USE_EXTENDED_ID = True

# Map integer IDs sent from RoboRIO to CircuitPython board pin objects
PIN_MAP = {
    5: board.GP5,
    6: board.GP6,
    9: board.GP9,
    10: board.GP10,
    11: board.GP11,
    12: board.GP12,
    13: board.GP13,
    24: board.GP24,
    25: board.GP25,
}

# ==========================================
# INITIALIZATION
# ==========================================

print("Initializing RP2040 CAN ToF Node...")

# 1. Initialize I2C Bus
i2c = busio.I2C(board.SCL, board.SDA)

# Store initialized sensors and their XSHUT pins
# Format: { index: {'tof': obj, 'pin': digitalio} }
sensors = {}

# 4. Initialize CAN Bus (MCP2515)
# The Adafruit RP2040 CAN Bus Feather uses specific pins for the onboard CAN controller
cs = digitalio.DigitalInOut(board.CAN_CS)
cs.switch_to_output()
# The onboard MCP2515 is wired to the default SPI bus on the RP2040 CAN Bus Feather
spi = board.SPI()

try:
    mcp = MCP2515(spi, cs, baudrate=CAN_BAUDRATE)
    print("CAN Bus initialized successfully.")
except Exception as e:
    print(f"Failed to initialize CAN: {e}")
    # Handle error or reset

# ==========================================
# MAIN LOOP
# ==========================================

print("Starting main loop...")

while True:
    # 1. Check for Configuration Messages
    # We use a short timeout to not block the loop significantly
    message = mcp.read_message(timeout=0.01)

    if message is not None and message.id == CAN_ID_CONFIG:
        try:
            # Payload Format: [Index, Pin_ID, Address, 0, 0, 0, 0, 0]
            idx = message.data[0]
            pin_id = message.data[1]
            addr = message.data[2]

            if pin_id in PIN_MAP:
                print(
                    f"Configuring Sensor {idx} on Pin D{pin_id} to Address {hex(addr)}"
                )

                # Setup XSHUT pin
                xshut = digitalio.DigitalInOut(PIN_MAP[pin_id])
                xshut.direction = digitalio.Direction.OUTPUT
                xshut.value = False  # Reset
                time.sleep(0.1)
                xshut.value = True  # Enable
                time.sleep(0.1)

                # Initialize Sensor
                tof = adafruit_vl53l1x.VL53L1X(i2c)
                tof.set_address(addr)
                tof.distance_mode = 1
                tof.timing_budget = 50
                tof.start_ranging()

                # Store in dictionary
                sensors[idx] = {"tof": tof, "pin": xshut}
                print(f"Sensor {idx} configured successfully.")

        except Exception as e:
            print(f"Config Error: {e}")

    # 2. Read Distances
    distances_mm = []
    # We iterate up to 4 potential sensors for the standard packet
    for i in range(4):
        if i in sensors:
            tof = sensors[i]["tof"]
            try:
                if tof.data_ready:
                    dist_cm = tof.distance
                    if dist_cm is not None:
                        distances_mm.append(int(dist_cm * 10))
                    else:
                        distances_mm.append(0xFFFF)
                    tof.clear_interrupt()
                else:
                    distances_mm.append(0xFFFF)  # Not ready
            except:
                distances_mm.append(0xFFFF)  # I2C Error
        else:
            distances_mm.append(0xFFFF)  # Not configured

    # Prepare CAN Message Payload
    # A standard CAN frame holds up to 8 bytes.
    # We will send distance as 16-bit integers (2 bytes per sensor).
    # This allows up to 4 sensors per CAN frame.
    payload = bytearray(8)

    # Pack up to 4 sensors into the frame
    for i in range(min(4, len(distances_mm))):
        d = distances_mm[i]
        # Pack as little-endian 16-bit integer
        payload[i * 2] = d & 0xFF
        payload[i * 2 + 1] = (d >> 8) & 0xFF

    # Send the CAN message
    # If you have more than 4 sensors, you will need to send a second message
    # with a different CAN ID or use a multiplexing scheme in the payload.
    try:
        msg = Message(id=CAN_ID_DATA, data=payload, extended=USE_EXTENDED_ID)
        mcp.send(msg)
    except Exception as e:
        # Fails silently if CAN bus is not connected/acked (unless MCP2515 loopback is on)
        pass

    # Loop delay (e.g., 50ms = 20Hz update rate)
    time.sleep(0.05)
