from phoenix6.hardware import TalonFX
from FROGlib.ctre import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
from roborio.constants import INTAKE_MOTOR_ID
from phoenix6 import controls
from phoenix6 import Slot0Configs


class Intake:
    def __init__(self):
        self.motor = FROGTalonFX(
            can_id=INTAKE_MOTOR_ID,
            can_bus="rio",
            motor_config=FROGTalonFXConfig(),
            feedback_config=FROGFeedbackConfig(),
            slot0gains=Slot0Configs(),
            parent_nt="Intake",
            motor_name="Intake Motor",
        )

    def _run_intake_motor(self):
        self.motor.set_control(
            controls.VoltageOut(-4, enable_foc=False, limit_forward_motion=True)
        )

    def _stop_intake_motor(self):
        self.motor.stopMotor()

    # I need it to check if the motor is running

    def _at_position(self, position) -> bool:
        return abs(self.motor.get_motor_voltage())
