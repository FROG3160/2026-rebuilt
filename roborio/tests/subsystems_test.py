import pytest
from unittest.mock import MagicMock
from commands2 import CommandScheduler
from subsystems.shooter import Shooter
from subsystems.feeder import Feeder
from subsystems.drive import Drive
from wpilib.simulation import DriverStationSim


# Fixture to reset the CommandScheduler between tests
@pytest.fixture(autouse=True)
def scheduler():
    """Reset the CommandScheduler between tests."""
    CommandScheduler.getInstance().cancelAll()
    CommandScheduler.getInstance().clearComposedCommands()
    CommandScheduler.getInstance().unregisterAllSubsystems()
    # IMPORTANT: Enable the robot simulation.
    # Otherwise, CommandScheduler.run() cancels commands because the robot is "disabled".
    DriverStationSim.setDsAttached(True)
    DriverStationSim.setEnabled(True)
    DriverStationSim.notifyNewData()
    yield


def test_shooter_fire_command_logic():
    """Test that fire_command calculates speed based on distance."""
    # Mock the Drive subsystem to control the distance returned
    mock_drive = MagicMock(spec=Drive)

    # Set a specific distance: 4 meters
    # Formula in Shooter: (23 * distance - 2) / 3
    # Calculation: (23 * 4 - 2) / 3 = 90 / 3 = 30.0
    mock_drive.get_distance_to_target.return_value = 4.0

    # Instantiate Shooter with the mock drive
    shooter = Shooter(mock_drive)

    # Get the command
    fire_cmd = shooter.cmd_fire_with_distance()

    # Initialize the command
    fire_cmd.initialize()
    # Execute the command (RunEndCommand runs the logic in execute)
    fire_cmd.execute()

    # Check if the speed was set correctly on the shooter
    assert shooter.get_commanded_speed() == 30.0

    # Check if is_at_speed returns False initially (simulated motor at 0)
    assert not shooter.is_at_speed()


def test_shooter_fire_command_no_target():
    """Test behavior when no target is found (None returned)."""
    mock_drive = MagicMock(spec=Drive)
    mock_drive.get_distance_to_target.return_value = None

    shooter = Shooter(mock_drive)
    fire_cmd = shooter.cmd_fire_with_distance()

    fire_cmd.initialize()
    fire_cmd.execute()

    # Should default to 0.0 if no target
    assert shooter.get_commanded_speed() == 0.0


def test_feeder_run_forward():
    """Test that the feeder command schedules correctly."""
    feeder = Feeder()
    run_cmd = feeder.runForward()

    # Schedule the command
    CommandScheduler.getInstance().schedule(run_cmd)

    # Verify it is scheduled
    assert CommandScheduler.getInstance().isScheduled(run_cmd)

    # Run scheduler once
    CommandScheduler.getInstance().run()

    # Cancel command
    run_cmd.cancel()

    # Verify it is no longer scheduled
    assert not CommandScheduler.getInstance().isScheduled(run_cmd)


def test_robot_container_init():
    """Smoke test to ensure RobotContainer initializes without dependency errors."""
    from robotcontainer import RobotContainer

    # This might fail if hardware dependencies (NavX, etc) are not mockable
    # or handled in simulationInit. If so, you may need to mock Drive inside RobotContainer.
    try:
        container = RobotContainer()
        assert container.shooter is not None
        assert container.feeder is not None
    except Exception as e:
        pytest.fail(f"RobotContainer failed to initialize: {e}")
