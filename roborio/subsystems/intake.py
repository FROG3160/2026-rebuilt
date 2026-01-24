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


intake_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageIntakeS,
)

intake_motor_config = FROGTalonFXConfig(
    id=constants.kIntakeMotorID,
    can_bus="rio",
    motor_name="Intake",
    parent_nt="Intake",
    motor_output=MOTOR_OUTPUT_CWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=intake_slot0,
)


class Intake:
    def __init__(self):
        self.motor = FROGTalonFX(motor_config=intake_motor_config)
        self._default_voltage = 4

    def _run_intake_motor(self):
        self.motor.set_control(
            controls.VoltageOut(self._default_voltage, enable_foc=False)
        )

    def _stop_intake_motor(self):
        self.motor.stopMotor()

    # I need it to check if the motor is running
    def _is_running(self) -> bool:
        return abs(self.motor.get_motor_voltage()) > 0
