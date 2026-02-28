import pytest
from unittest.mock import MagicMock
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import Field2d
from subsystems.feedback import FieldZones
import constants


def test_field_zones_in_restricted_zone():
    # Provide a simple pose supplier that defaults to (0,0) if not overridden
    pose = Pose2d()

    def pose_supplier():
        return pose

    zones = FieldZones(pose_supplier, lambda: False, Field2d())

    # Inside Blue Right Trench: {"x_min": 4.05, "x_max": 5.23, "y_min": 0.0, "y_max": 1.265}
    pose_in_blue_right = Pose2d(5.0, 1.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_blue_right) == True

    # Inside Blue Left Trench: {"x_min": 4.05, "x_max": 5.24, "y_min": 6.75, "y_max": 8.0}
    pose_in_blue_left = Pose2d(5.0, 7.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_blue_left) == True

    # Inside Red Left Trench: {"x_min": 11.33, "x_max": 12.53, "y_min": 0.0, "y_max": 1.265}
    pose_in_red_right = Pose2d(12.0, 1.0, Rotation2d())
    assert zones.in_restricted_zone(pose_in_red_right) == True

    # Inside Red Right Trench: {"x_min": 11.33, "x_max": 12.53, "y_min": 6.75, "y_max": 8.0}
    pose_in_red_left = Pose2d(12.0, 7.5, Rotation2d())
    assert zones.in_restricted_zone(pose_in_red_left) == True

    # Outside zones (e.g. Center of field X=8.25 Y=4.1)
    pose_center = Pose2d(8.25, 4.1, Rotation2d())
    assert zones.in_restricted_zone(pose_center) == False


def test_field_zones_periodic_restricted():
    def pose_supplier():
        # Return a pose that is INSIDE a restricted zone
        return Pose2d(5.0, 1.0, Rotation2d())

    zones = FieldZones(pose_supplier, lambda: False, Field2d())
    zones.periodic()

    assert zones.status == "Restricted Zone!"
    assert zones.get_max_speed_scalar() < 1.0


def test_field_zones_periodic_clear():
    def pose_supplier():
        # Return a pose that is OUTSIDE restricted zones
        return Pose2d(5.0, 5.0, Rotation2d())

    zones = FieldZones(pose_supplier, lambda: False, Field2d())
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

    zones = FieldZones(pose_supplier, lambda: False, Field2d())

    # Simulate Blue Alliance
    wpilib.simulation.DriverStationSim.setAllianceStationId(
        hal.AllianceStationID.kBlue1
    )
    wpilib.simulation.DriverStationSim.notifyNewData()

    # In Blue alliance zone (X=3.0 < 5.5) -> should aim at kBlueHub
    current_pose[0] = Pose2d(3.0, 3.0, Rotation2d())
    assert zones.get_aim_target() == constants.kBlueHub

    # In middle of field (X=8.0)
    current_pose[0] = Pose2d(
        8.0, 3.0, Rotation2d()
    )  # y < 4.1 -> closest corner (0.0, 0.0)
    assert zones.get_aim_target() == constants.kBlueRightCorner

    current_pose[0] = Pose2d(
        8.0, 5.0, Rotation2d()
    )  # y >= 4.1 -> closest corner (0.0, 8.2)
    assert zones.get_aim_target() == constants.kBlueLeftCorner

    # Simulate Red Alliance
    wpilib.simulation.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kRed1)
    wpilib.simulation.DriverStationSim.notifyNewData()

    # In Red alliance zone (X=13.0 > 11.0) -> should aim at kRedHub
    current_pose[0] = Pose2d(13.0, 3.0, Rotation2d())
    assert zones.get_aim_target() == constants.kRedHub

    # In middle of field (X=8.0)
    current_pose[0] = Pose2d(
        8.0, 3.0, Rotation2d()
    )  # y < 4.1 -> closest corner (16.5, 0.0)
    assert zones.get_aim_target() == constants.kRedRightCorner

    current_pose[0] = Pose2d(
        8.0, 5.0, Rotation2d()
    )  # y >= 4.1 -> closest corner (16.5, 8.2)
    assert zones.get_aim_target() == constants.kRedLeftCorner


def test_field_zones_repel_vector():
    current_pose = [Pose2d(0.0, 0.0, Rotation2d())]
    hood_deployed = [False]

    def pose_supplier():
        return current_pose[0]

    def hood_supplier():
        return hood_deployed[0]

    zones = FieldZones(pose_supplier, hood_supplier, Field2d())

    intended_velocity = Translation2d(6.0, 0.0)  # Moving in +X at 6 m/s

    # Not near a trench tag, so velocity should be unmodified
    assert zones.get_trench_velocity_limit(intended_velocity) == intended_velocity

    # Near a trench tag (Tag ID 1: X=11.878, Y=7.425)
    # Robot is at X=10.5, Y=7.425. Distance = 1.378 (Inside 1.5m outer radius)
    current_pose[0] = Pose2d(10.5, 7.425, Rotation2d())

    # Hood is NOT deployed, velocity should be unmodified
    assert zones.get_trench_velocity_limit(intended_velocity) == intended_velocity

    # Hood IS deployed, moving TOWARDS tag (positive X)
    hood_deployed[0] = True
    limited_vel = zones.get_trench_velocity_limit(intended_velocity)
    # Distance is 1.378. Inner radius is 0.75. Range is 0.75.
    # Scalar should be (1.378 - 0.75) / 0.75 = 0.8373
    # Velocity should be reduced!
    assert limited_vel.x < 6.0
    assert limited_vel.x > 0.0

    # Driving AWAY from the tag (negative X) while inside radius
    reverse_velocity = Translation2d(-6.0, 0.0)
    assert zones.get_trench_velocity_limit(reverse_velocity) == reverse_velocity

    # Extremely close to tag (inside inner 0.75m radius)
    # Robot at X=11.5, Y=7.425. Distance = 0.378
    current_pose[0] = Pose2d(11.5, 7.425, Rotation2d())
    halted_vel = zones.get_trench_velocity_limit(intended_velocity)
    # Should be completely clamped to 0 forward velocity
    assert halted_vel.x == pytest.approx(0.0, abs=1e-5)

    # But reverse should still work at 100%
    assert zones.get_trench_velocity_limit(reverse_velocity) == reverse_velocity
