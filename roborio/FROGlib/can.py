import struct
from wpilib import CAN, CANData


class FROGCan(CAN):
    def __init__(self, device_id):
        # Initialize with Team Use Manufacturer (8) and Miscellaneous Device Type (10)
        super().__init__(
            device_id,
            CAN.kTeamManufacturer.kTeamUse,
            CAN.kTeamDeviceType.kMiscellaneous,
        )


class FROGRP2040(FROGCan):
    """
    Represents an RP2040 CAN Node that manages multiple sensors.
    """

    def __init__(self, device_id=0):
        super().__init__(device_id)
        self._shared_data = [0xFFFF] * 4

    def configure_i2c(self, sensor_index: int, disable_pin: int, i2c_address):
        data = bytearray(8)
        data[0] = sensor_index
        data[1] = disable_pin
        data[2] = i2c_address
        self.writePacket(data, 1)  # API ID 1 for configuration
        print(
            f"FROGRP2040: Sent config for Sensor {sensor_index} (Pin {disable_pin}, Addr {hex(i2c_address)})"
        )

    def update(self):
        try:
            received_data = CANData()
            if self.readPacketNew(0, received_data):  # API ID 0 for data
                self._shared_data = list(struct.unpack("<HHHH", received_data.data))
        except Exception:
            pass

    def get_distance(self, index):
        self.update()
        if 0 <= index < len(self._shared_data):
            return self._shared_data[index]
        return 0xFFFF
