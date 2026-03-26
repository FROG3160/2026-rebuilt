from commands2 import Command
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.system.plant import DCMotor, LinearSystemId
from FROGlib.subsystem import FROGSubsystem, Direction
from phoenix6.controls import VoltageOut
from phoenix6 import controls, SignalLogger
from FROGlib.ctre import (
    FROGTalonFX,
    get_frog_talon_config,
    MOTOR_OUTPUT_CWP_COAST,
)
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    MotorOutputConfigs,
)
import constants
import wpilib
from wpiutil import SendableBuilder

# Slot 0: velocity control for normal run forward/backward
feed_velocity_slot = (
    Slot0Configs()
    .with_k_s(constants.Feeder.FEED_S)
    .with_k_v(constants.Feeder.FEED_V)
    .with_k_p(constants.Feeder.FEED_VELOCITY_P)
    .with_k_i(constants.Feeder.FEED_VELOCITY_I)
    .with_k_d(constants.Feeder.FEED_VELOCITY_D)
)

feed_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CWP_COAST)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(11.5425))
    .with_slot0(feed_velocity_slot)
)

kBackOffRotations = 0.15  # rotations to retract from flywheel
kBackOffTolerance = 0.02  # rotations


class Feeder(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            id=constants.CANIDs.FEED_MOTOR,
            motor_config=feed_motor_config,
            canbus="rio",
            motor_name="Feed Motor",
            signal_profile=FROGTalonFX.SignalProfile.BASIC,
        )
        self._feed_velocity = 4.81  # m/s
        self._back_off_target = 0.0

        # Set up SysID routine for the feeder
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "state-feeder", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda voltage: self.motor.set_control(
                    controls.VoltageOut(voltage, enable_foc=False)
                ),
                lambda log: None,
                self,
            ),
        )

        if wpilib.RobotBase.isSimulation():
            self.motor.simulation_init(moi=0.001)

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)

    def _runForward(self):
        self.motor.set_control(controls.VelocityVoltage(self._feed_velocity, slot=0))

    def _runBackward(self):
        self.motor.set_control(controls.VelocityVoltage(-self._feed_velocity, slot=0))

    def _startBackOff(self):
        self._back_off_target = self.motor.get_position().value - kBackOffRotations

    def _applyBackOff(self):
        self.motor.set_control(controls.PositionVoltage(self._back_off_target, slot=1))

    def _atBackOffTarget(self) -> bool:
        return (
            abs(self.motor.get_position().value - self._back_off_target)
            < kBackOffTolerance
        )

    def run_forward_cmd(self) -> Command:
        """Run the feeder motor forward at the configured feed velocity, stopping when interrupted."""
        return self.runEnd(self._runForward, self.stop)

    def run_backward_cmd(self) -> Command:
        """Run the feeder motor backward at the configured feed velocity, stopping when interrupted."""
        return self.runEnd(self._runBackward, self.stop)

    def back_off_cmd(self) -> Command:
        """Back the feeder off 0.15 rotations away from the flywheel using position control."""
        return self.runOnce(self._startBackOff).andThen(
            self.run(self._applyBackOff).until(self._atBackOffTarget)
        )

    def stop(self):
        self.motor.stopMotor()

    def get_direction(self) -> Direction:
        voltage = self.motor.get_motor_voltage().value
        if voltage > 0.1:
            return Direction.FORWARD
        elif voltage < -0.1:
            return Direction.REVERSE
        else:
            return Direction.IDLE

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(dt, battery_v)

    @FROGSubsystem.tunable(4.81, "Commanded Velocity")
    def feed_velocity_tunable(self, val):
        self._feed_velocity = val

    @FROGSubsystem.telemetry("Actual Position")
    def get_feed_position(self) -> float:
        return self.motor.get_position().value

    @FROGSubsystem.telemetry("Actual Velocity")
    def get_feed_velocity(self) -> float:
        return self.motor.get_velocity().value
