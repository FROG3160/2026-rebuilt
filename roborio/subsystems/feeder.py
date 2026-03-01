from commands2 import Command
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from wpimath.system.plant import DCMotor, LinearSystemId
from FROGlib.subsystem import FROGSubsystem
from phoenix6.controls import VoltageOut
from phoenix6 import controls, SignalLogger
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
    MOTOR_OUTPUT_CWP_COAST,
)
import constants
import wpilib
from wpiutil import SendableBuilder

# Slot 0: velocity control for normal run forward/backward
feed_velocity_slot = FROGSlotConfig(
    k_s=constants.kFeedS,
    k_v=constants.kFeedV,
    k_p=constants.kFeedVelocityP,
    k_i=constants.kFeedVelocityI,
    k_d=constants.kFeedVelocityD,
)

# TODO #67: get position control running again.
# Slot 1: position control for back-off move
# feed_position_slot = FROGSlotConfig(
#     k_s=constants.kFeedS,
#     k_v=constants.kFeedV,
#     k_p=constants.kFeedPositionP,
#     k_i=constants.kFeedPositionI,
#     k_d=constants.kFeedPositionD,
# )

feed_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt=f"{constants.kComponentSubtableName}/Feeder",
    motor_output=MOTOR_OUTPUT_CWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=5.0),
    slot0=feed_velocity_slot,
    # slot1=feed_position_slot,
)

kBackOffRotations = 0.15  # rotations to retract from flywheel
kBackOffTolerance = 0.02  # rotations


class Feeder(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            motor_config=feed_motor_config.with_id(
                constants.kFeedMotorID
            ).with_motor_name("Feed Motor")
        )
        self._feed_velocity = 20.0 / 3  # rotations per second
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
            feed_gearbox = DCMotor.falcon500(1)
            J_feed = 0.001  # kg·m², feed roller moment of inertia
            gearing = (
                self.motor.config.feedback.sensor_to_mechanism_ratio
            )  # matches sensor_to_mechanism_ratio
            feed_plant = LinearSystemId.DCMotorSystem(
                feed_gearbox,
                J_feed,
                gearing,
            )
            self.motor.simulation_init(feed_plant, feed_gearbox)

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

    def runForward(self):
        return self.runEnd(self._runForward, self.stop)

    def runBackward(self):
        return self.runEnd(self._runBackward, self.stop)

    def backOffCmd(self):
        """Back the feeder off 0.15 rotations away from the flywheel using position control."""
        return self.runOnce(self._startBackOff).andThen(
            self.run(self._applyBackOff).until(self._atBackOffTarget)
        )

    def stop(self):
        self.motor.stopMotor()

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(dt, battery_v)

    @FROGSubsystem.tunable(20.0 / 3, "Commanded Velocity")
    def feed_velocity_tunable(self, val):
        self._feed_velocity = val

    @FROGSubsystem.telemetry("Actual Position")
    def get_feed_position(self) -> float:
        return self.motor.get_position().value

    @FROGSubsystem.telemetry("Actual Velocity")
    def get_feed_velocity(self) -> float:
        return self.motor.get_velocity().value
