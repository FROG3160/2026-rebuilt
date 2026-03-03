import pytest
from commands2 import CommandScheduler
from commands2.sysid import SysIdRoutine
from wpilib.simulation import DriverStationSim
from subsystems.feeder import Feeder, kBackOffRotations, kBackOffTolerance


@pytest.fixture(autouse=True)
def scheduler():
    """Reset the CommandScheduler and enable the simulated DriverStation between tests."""
    CommandScheduler.getInstance().cancelAll()
    CommandScheduler.getInstance().clearComposedCommands()
    CommandScheduler.getInstance().unregisterAllSubsystems()
    DriverStationSim.setDsAttached(True)
    DriverStationSim.setEnabled(True)
    DriverStationSim.notifyNewData()
    yield


@pytest.fixture
def feeder():
    return Feeder()


# ── Instantiation ────────────────────────────────────────────────────────────


def test_feeder_instantiates(feeder):
    """Feeder initializes without raising."""
    assert feeder is not None


def test_feeder_default_velocity(feeder):
    """Default feed velocity is positive (forward-is-positive convention)."""
    assert feeder._feed_velocity > 0


# ── runForward ───────────────────────────────────────────────────────────────


def test_run_forward_schedules(feeder):
    cmd = feeder.runForward()
    CommandScheduler.getInstance().schedule(cmd)
    assert CommandScheduler.getInstance().isScheduled(cmd)


def test_run_forward_executes_without_error(feeder):
    cmd = feeder.runForward()
    CommandScheduler.getInstance().schedule(cmd)
    CommandScheduler.getInstance().run()  # initialize + execute
    cmd.cancel()
    assert not CommandScheduler.getInstance().isScheduled(cmd)


# ── runBackward ──────────────────────────────────────────────────────────────


def test_run_backward_schedules(feeder):
    cmd = feeder.runBackward()
    CommandScheduler.getInstance().schedule(cmd)
    assert CommandScheduler.getInstance().isScheduled(cmd)


def test_run_backward_executes_without_error(feeder):
    cmd = feeder.runBackward()
    CommandScheduler.getInstance().schedule(cmd)
    CommandScheduler.getInstance().run()
    cmd.cancel()
    assert not CommandScheduler.getInstance().isScheduled(cmd)


# ── backOffCmd ───────────────────────────────────────────────────────────────


def test_back_off_cmd_schedules(feeder):
    cmd = feeder.backOffCmd()
    CommandScheduler.getInstance().schedule(cmd)
    assert CommandScheduler.getInstance().isScheduled(cmd)


def test_back_off_sets_target_from_current_position(feeder):
    """_startBackOff captures position and subtracts kBackOffRotations."""
    # Motor starts at position 0 in simulation
    feeder._startBackOff()
    assert feeder._back_off_target == pytest.approx(
        feeder.motor.get_position().value - kBackOffRotations
    )


def test_back_off_not_at_target_initially(feeder):
    """_atBackOffTarget is False right after initiating the back-off."""
    feeder._startBackOff()
    assert not feeder._atBackOffTarget()


def test_back_off_at_target_when_within_tolerance(feeder):
    """_atBackOffTarget is True when motor position is within tolerance of target."""
    # Set target to be just within tolerance of the starting position (0)
    feeder._back_off_target = kBackOffTolerance / 2
    assert feeder._atBackOffTarget()


def test_back_off_not_at_target_when_outside_tolerance(feeder):
    """_atBackOffTarget is False when motor position is outside tolerance."""
    feeder._back_off_target = -(kBackOffRotations)
    assert not feeder._atBackOffTarget()


# ── SysId ────────────────────────────────────────────────────────────────────


def test_sysid_quasistatic_forward_schedules(feeder):
    cmd = feeder.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
    CommandScheduler.getInstance().schedule(cmd)
    assert CommandScheduler.getInstance().isScheduled(cmd)
    cmd.cancel()


def test_sysid_quasistatic_reverse_schedules(feeder):
    cmd = feeder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
    CommandScheduler.getInstance().schedule(cmd)
    assert CommandScheduler.getInstance().isScheduled(cmd)
    cmd.cancel()


def test_sysid_dynamic_forward_schedules(feeder):
    cmd = feeder.sysIdDynamic(SysIdRoutine.Direction.kForward)
    CommandScheduler.getInstance().schedule(cmd)
    assert CommandScheduler.getInstance().isScheduled(cmd)
    cmd.cancel()


def test_sysid_dynamic_reverse_schedules(feeder):
    cmd = feeder.sysIdDynamic(SysIdRoutine.Direction.kReverse)
    CommandScheduler.getInstance().schedule(cmd)
    assert CommandScheduler.getInstance().isScheduled(cmd)
    cmd.cancel()
