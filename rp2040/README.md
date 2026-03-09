# RP2040 CAN Bus Feather - ToF Sensor Node

This folder contains the CircuitPython code for the Adafruit RP2040 CAN Bus Feather.
This script configures the Feather to read distances from multiple Adafruit VL53L1X Time-of-Flight (ToF) sensors over I2C, and broadcast those distances over the FRC CAN bus.

## Prerequisites

1.  **Install CircuitPython:**
    *   Download the latest `.uf2` file for the **Adafruit Feather RP2040 CAN Bus** from [circuitpython.org](https://circuitpython.org/board/adafruit_feather_rp2040_can/).
    *   Hold the `BOOT` button on the Feather while plugging it into your computer via USB.
    *   Drag and drop the downloaded `.uf2` file onto the `RPI-RP2` drive that appears.

2.  **Install Required Libraries:**
    Download the CircuitPython library bundle for your version of CircuitPython from [circuitpython.org/libraries](https://circuitpython.org/libraries). Extract it and copy the following folders/files into the `lib` folder on your `CIRCUITPY` drive:
    *   `adafruit_mcp2515` (Folder)
    *   `adafruit_vl53l1x.mpy` (File)
    *   `adafruit_bus_device` (Folder)

## Wiring & Hardware Setup

### ToF Sensors (VL53L1X)
Since all VL53L1X sensors share the same default I2C address (`0x29`), you must use their `XSHUT` pins to assign them unique addresses during startup.

1.  Connect **VIN** to 3.3V on the Feather.
2.  Connect **GND** to GND.
3.  Connect **SCL** to SCL.
4.  Connect **SDA** to SDA.
5.  Connect the **XSHUT** pin of *each* sensor to a separate digital pin on the Feather (e.g., D5, D6, D9, D10). 
    *   *Note: The RP2040 code waits for configuration over CAN. You do not need to hardcode active pins in `code.py`, but ensure your wired pins are present in the `PIN_MAP` dictionary.*

### CAN Bus
1.  Connect the **CAN H** and **CAN L** terminals on the Feather to your FRC robot's CAN network (green and yellow wires).
2.  The FRC CAN bus runs at 1 Mbps, which is already configured in `code.py`.
3.  Ensure the CAN network is properly terminated with 120-ohm resistors at both ends (usually handled by the RoboRIO and PDP/PDH).

## RoboRIO Integration

The RP2040 waits for configuration messages to initialize sensors, then broadcasts data.

### Configuration (RoboRIO -> RP2040)
*   **CAN ID:** `0x1E040001`
*   **Payload:** `[Index, Pin, Address, 0, 0, 0, 0, 0]`
    *   `Index`: 0-3 (The slot in the data packet)
    *   `Pin`: The RP2040 GPIO pin number (e.g., 5 for D5)
    *   `Address`: The desired I2C address (e.g., 0x30)

### Data Broadcast (RP2040 -> RoboRIO)
*   **CAN ID:** `0x1E040000`
*   **Payload Format:**
    *   Byte 0, 1: Sensor 1 Distance (mm) - Little Endian UInt16
    *   Byte 2, 3: Sensor 2 Distance (mm) - Little Endian UInt16
    *   Byte 4, 5: Sensor 3 Distance (mm) - Little Endian UInt16
    *   Byte 6, 7: Sensor 4 Distance (mm) - Little Endian UInt16
*   **Invalid Value:** If a sensor fails to read or is disconnected, it will send `0xFFFF` (65535).

### Example RoboRIO RobotPy Code

To read this on your RoboRIO, you can use the `FROGRP2040` and `FROGToF` classes from your `FROGlib`.

```python
# In your subsystem or robot container's __init__
from FROGlib.can import FROGRP2040
from FROGlib.sensors import FROGToF

# Create the RP2040 node object
# The device_id corresponds to the CAN ID (0 -> 0x1E040000)
self.rp2040 = FROGRP2040(device_id=0)

# Create and configure each ToF sensor.
# Passing the rp2040 object allows the sensor to register itself and send config over CAN.
self.tof_sensor_1 = FROGToF(self.rp2040, index=0, pin=5, address=0x30)
self.tof_sensor_2 = FROGToF(self.rp2040, index=1, pin=6, address=0x31)
# ... add more sensors as needed

# In your subsystem's periodic() or a command's execute()
# The FROGRP2040 object handles reading the CAN data packet automatically
# when you ask a sensor for its distance.
distance_1 = self.tof_sensor_1.distance # in millimeters
distance_2 = self.tof_sensor_2.distance # in millimeters

if distance_1 != -1.0:
    print(f"Sensor 1 distance: {distance_1} mm")

```

## Running the Code
Copy the `code.py` file from this directory and place it directly on the root of your `CIRCUITPY` drive. The board will automatically reboot and start running the script. You can use a serial terminal (like PuTTY or the Mu Editor's serial console) to view print statements and debug.