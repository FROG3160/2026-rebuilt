from .can import FROGRP2040


class FROGToF:
    """
    Represents a single VL53L1X Time-of-Flight sensor managed by a FROGRP2040.
    """

    def __init__(self, rp2040: FROGRP2040, index: int, pin: int, address: int):
        self.rp2040 = rp2040
        self.index = index
        self.rp2040.configure_i2c(index, pin, address)

    @property
    def distance(self) -> float:
        """Returns the distance in millimeters. Returns -1.0 if invalid/error."""
        dist = self.rp2040.get_distance(self.index)
        return dist if dist != 0xFFFF else -1.0
