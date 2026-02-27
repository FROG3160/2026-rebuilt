import pytest
from unittest.mock import MagicMock
from wpimath.geometry import Pose2d, Rotation2d
from subsystems.feedback import FieldZones
import constants

def test_field_zones_in_restricted_zone():
    # Provide a simple pose supplier that defaults to (0,0) if not overridden
    pose = Pose2d()
    def pose_supplier():
        return pose
        
    zones = FieldZones(pose_supplier)

    # Inside zone 1: {"x_min": 0.0, "x_max": 3.5, "y_min": 0.0, "y_max": 1.8}
    pose_in_zone1 = Pose2d(2.0, 1.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_zone1) == True

    # Inside zone 3: {"x_min": 7.0, "x_max": 9.5, "y_min": 3.0, "y_max": 5.2}
    pose_in_zone3 = Pose2d(8.0, 4.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_zone3) == True

    # Outside zones
    pose_out = Pose2d(5.0, 5.0, Rotation2d())
    assert zones.in_restricted_zone(pose_out) == False

def test_field_zones_periodic_restricted():
    def pose_supplier():
        # Return a pose that is INSIDE a restricted zone
        return Pose2d(2.0, 1.0, Rotation2d())
    
    zones = FieldZones(pose_supplier)
    zones.periodic()
    
    assert zones.status == "Restricted Zone!"
    assert zones.get_max_speed_scalar() < 1.0

def test_field_zones_periodic_clear():
    def pose_supplier():
        # Return a pose that is OUTSIDE restricted zones
        return Pose2d(5.0, 5.0, Rotation2d())
    
    zones = FieldZones(pose_supplier)
    zones.periodic()
    
    assert zones.status == "Clear"
    assert zones.get_max_speed_scalar() == 1.0

