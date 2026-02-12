from copy import deepcopy
from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
)
import constants
from phoenix6 import controls
from FROGlib.ctre import MOTOR_OUTPUT_CWP_COAST, MOTOR_OUTPUT_CCWP_COAST
from subsystems.drive import Drive

flywheel_slot0 = FROGSlotConfig(
    k_s=constants.kFlywheelS,
    k_v=constants.kFlywheelV,
    k_a=constants.kFlywheelA,
    k_p=constants.kFlywheelP,
    k_i=constants.kFlywheelI,
    k_d=constants.kFlywheelD,
)
feed_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageHopperS,
)

flywheel_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Shooter",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=flywheel_slot0,
)

feed_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Shooter",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=feed_slot0,
)


class Flywheel:
    def __init__(self, id: int, name: str):
        self.right_motor = FROGTalonFX(
            motor_config=deepcopy(flywheel_motor_config)
            .with_id(id)
            .with_motor_name(name)
        )
        self.left_motor = FROGTalonFX(
            motor_config=deepcopy(flywheel_motor_config)
            .with_id(id + 1)
            .with_motor_name(name.replace("Right", "Left"))
            .with_motor_output(MOTOR_OUTPUT_CWP_COAST)
        )
        self.left_motor.set_control(controls.Follower(self.right_motor.device_id, True))

    def _set_speed(self, speed: float):
        self.right_motor.set_control(controls.VelocityVoltage(speed))

    def _stop_motor(self):
        self.right_motor.stopMotor()


class Shooter(Subsystem):
    def __init__(self, drive: Drive):
        self.drive = drive
        self.flywheel = Flywheel(constants.kShooterLeftFlywheelID, "Flywheel")
        self.feed_motor = FROGTalonFX(
            motor_config=FROGTalonFXConfig(feed_motor_config)
            .with_id(constants.kFeedMotorID)
            .with_motor_name("FeedMotor")
        )

    # boolean to indicate if flywheel is at target speed
    def _is_flywheel_at_speed(
        self, target_speed: float, tolerance: float = 50.0
    ) -> bool:
        current_rps = self.flywheel.right_motor.get_velocity().value
        return abs(current_rps - target_speed) <= tolerance

    # method to run feed motor forward
    def _run_feed_motor_forward(self):
        self.feed_motor.set_control(
            controls.VoltageOut(constants.kVoltageHopperS, enable_foc=False)
        )

    # method to stop feed motor
    def _stop_feed_motor(self):
        self.feed_motor.stopMotor()

    # generate a command to run the flywheel at a target speed
    def run_flywheel_at_speed(self, target_speed: float):
        return self.startEnd(
            lambda: self.flywheel._set_speed(target_speed),
            self.flywheel._stop_motor,
        )
