import pytest
from unittest.mock import MagicMock
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import Field2d
from subsystems.feedback import FieldZones
import constants

def test_field_zones_in_restricted_zone():
    # Provide a simple pose supplier that defaults to (0,0) if not overridden
    pose = Pose2d()
    def pose_supplier():
        return pose
        
    zones = FieldZones(pose_supplier, Field2d())

    # Inside Blue Right Trench: {"x_min": 4.62, "x_max": 8.25, "y_min": 0.0, "y_max": 1.8}
    pose_in_blue_right = Pose2d(5.0, 1.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_blue_right) == True

    # Inside Blue Left Trench: {"x_min": 4.62, "x_max": 8.25, "y_min": 6.4, "y_max": 8.2}
    pose_in_blue_left = Pose2d(6.0, 7.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_blue_left) == True

    # Inside Red Right Trench: {"x_min": 8.25, "x_max": 11.92, "y_min": 0.0, "y_max": 1.8}
    pose_in_red_right = Pose2d(10.0, 1.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_red_right) == True

    # Inside Red Left Trench: {"x_min": 8.25, "x_max": 11.92, "y_min": 6.4, "y_max": 8.2}
    pose_in_red_left = Pose2d(11.0, 7.5, Rotation2d())
    assert zones.in_restricted_zone(pose_in_red_left) == True

    # Outside zones (e.g. Center of field X=8.25 Y=4.1)
    pose_center = Pose2d(8.25, 4.1, Rotation2d())
    assert zones.in_restricted_zone(pose_center) == False

def test_field_zones_periodic_restricted():
    def pose_supplier():
        # Return a pose that is INSIDE a restricted zone
        return Pose2d(5.0, 1.0, Rotation2d())
    
    zones = FieldZones(pose_supplier, Field2d())
    zones.periodic()
    
    assert zones.status == "Restricted Zone!"
    assert zones.get_max_speed_scalar() < 1.0

def test_field_zones_periodic_clear():
    def pose_supplier():
        # Return a pose that is OUTSIDE restricted zones
        return Pose2d(5.0, 5.0, Rotation2d())
    
    zones = FieldZones(pose_supplier, Field2d())
    zones.periodic()
    
    assert zones.status == "Clear"
    assert zones.get_max_speed_scalar() == 1.0


import wpilib.simulation
import hal

def test_field_zones_get_aim_target():
    # Setup mutable pose
    current_pose = [Pose2d(3.0, 3.0, Rotation2d())]
    def pose_supplier():
        return current_pose[0]
        
    zones = FieldZones(pose_supplier, Field2d())
    
    # Simulate Blue Alliance
    wpilib.simulation.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue1)
    wpilib.simulation.DriverStationSim.notifyNewData()
    
    # In Blue alliance zone (X=3.0 < 5.5) -> should aim at kBlueHub
    current_pose[0] = Pose2d(3.0, 3.0, Rotation2d())
    assert zones.get_aim_target() == constants.kBlueHub
    
    # In middle of field (X=8.0)
    current_pose[0] = Pose2d(8.0, 3.0, Rotation2d()) # y < 4.1 -> closest corner (0.0, 0.0)
    assert zones.get_aim_target() == constants.kBlueRightCorner
    
    current_pose[0] = Pose2d(8.0, 5.0, Rotation2d()) # y >= 4.1 -> closest corner (0.0, 8.2)
    assert zones.get_aim_target() == constants.kBlueLeftCorner

    # Simulate Red Alliance
    wpilib.simulation.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kRed1)
    wpilib.simulation.DriverStationSim.notifyNewData()
    
    # In Red alliance zone (X=13.0 > 11.0) -> should aim at kRedHub
    current_pose[0] = Pose2d(13.0, 3.0, Rotation2d())
    assert zones.get_aim_target() == constants.kRedHub
    
    # In middle of field (X=8.0)
    current_pose[0] = Pose2d(8.0, 3.0, Rotation2d()) # y < 4.1 -> closest corner (16.5, 0.0)
    assert zones.get_aim_target() == constants.kRedRightCorner

    current_pose[0] = Pose2d(8.0, 5.0, Rotation2d()) # y >= 4.1 -> closest corner (16.5, 8.2)
    assert zones.get_aim_target() == constants.kRedLeftCorner

