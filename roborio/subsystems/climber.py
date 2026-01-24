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

deploy_slot0 = FROGSlotConfig(
    k_s=constants.kDeployS,
    k_p=constants.kDeployP,
    k_i=constants.kDeployI,
    k_d=constants.kDeployD,
)
lift_slot0 = FROGSlotConfig(
    k_s=constants.kLiftS,
    k_v=constants.kLiftV,
    k_g=constants.kLiftG,
    k_p=constants.kLiftP,
    k_i=constants.kLiftI,
    k_d=constants.kLiftD,
)
deploy_motor_config = FROGTalonFXConfig(
    id=constants.kClimberDeployMotorID,
    can_bus="rio",
    motor_name="Deploy",
    parent_nt="Climber",
    motor_output=MOTOR_OUTPUT_CWP_BRAKE,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=deploy_slot0,
)
lift_motor_config = FROGTalonFXConfig(
    id=constants.kClimberLeftLiftMotorID,
    can_bus="rio",
    motor_name="LeftLift",
    parent_nt="Climber",
    motor_output=MOTOR_OUTPUT_CCWP_BRAKE,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=lift_slot0,
)


class Climber:
    def __init__(self):
        self.deploy_motor = FROGTalonFX(motor_config=deploy_motor_config)
        self.left_lift_motor = FROGTalonFX(
            motor_config=deepcopy(lift_motor_config).with_id(
                constants.kClimberLeftLiftMotorID
            )
        )
        self.right_lift_motor = FROGTalonFX(
            motor_config=deepcopy(lift_motor_config)
            .with_id(constants.kClimberRightLiftMotorID)
            .with_motor_output(MOTOR_OUTPUT_CWP_BRAKE)
        )
        self.right_lift_motor.set_control(
            controls.Follower(self.left_lift_motor.device_id, False)
        )

    def _deploy_position(self, position: float):
        self.deploy_motor.set_control(
            controls.PositionVoltage(position, enable_foc=False)
        )

    def _stop_deploy(self):
        self.deploy_motor.stopMotor()

    def _lift_position(self, position: float):
        self.left_lift_motor.set_control(
            controls.PositionVoltage(position, enable_foc=False)
        )

    def _stop_lift(self):
        self.left_lift_motor.stopMotor()
