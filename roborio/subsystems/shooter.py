from copy import deepcopy
from phoenix6.hardware import TalonFX
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
)
import constants
from phoenix6 import controls
from FROGlib.ctre import MOTOR_OUTPUT_CWP_BRAKE, MOTOR_OUTPUT_CCWP_BRAKE

flywheel_slot0 = FROGSlotConfig(
    k_s=constants.kFlywheelS,
    k_v=constants.kFlywheelV,
    k_a=constants.kFlywheelA,
    k_p=constants.kFlywheelP,
    k_i=constants.kFlywheelI,
    k_d=constants.kFlywheelD,
)

flywheel_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Shooter",
    motor_output=MOTOR_OUTPUT_CWP_BRAKE,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=flywheel_slot0,
)


class Flywheel:
    def __init__(self, id: int, name: str):
        self.motor = FROGTalonFX(
            motor_config=deepcopy(flywheel_motor_config)
            .with_id(id)
            .with_motor_name(name)
        )

    def _set_speed(self, speed: float):
        self.motor.set_control(controls.VelocityVoltage(speed))

    def _stop_motor(self):
        self.motor.stopMotor()
