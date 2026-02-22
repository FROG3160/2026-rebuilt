from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
    MOTOR_OUTPUT_CWP_COAST,
)
from phoenix6 import controls
import constants
import wpilib
from wpiutil import SendableBuilder


feed_slot0 = FROGSlotConfig(
    k_s=constants.kFeedS,
)

feed_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Feeder",
    motor_output=MOTOR_OUTPUT_CWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=5.0),
    slot0=feed_slot0,
)


class Feeder(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            motor_config=feed_motor_config.with_id(
                constants.kFeedMotorID
            ).with_motor_name("Feed Motor")
        )
        self._feed_speed = 4.0  # in volts for now

    def _runForward(self):
        self.motor.set_control(controls.VoltageOut(self._feed_speed, enable_foc=False))

    def runForward(self):
        return self.startEnd(self._runForward, self.stop)

    def stop(self):
        self.motor.stopMotor()

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(
            dt, battery_v, max_velocity_rps=83.33, velocity_sign_multiplier=1
        )

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)
        builder.setSmartDashboardType("Feeder")

        builder.addDoubleProperty(
            "tunable/Feed Speed",
            lambda: self._feed_speed,
            lambda value: setattr(self, "_feed_speed", value),
        )
