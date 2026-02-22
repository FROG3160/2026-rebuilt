from copy import deepcopy
from commands2 import Subsystem, Command
from phoenix6.hardware import TalonFX
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
)
import constants
from wpimath.system.plant import DCMotor, LinearSystemId
from phoenix6 import controls
from FROGlib.ctre import MOTOR_OUTPUT_CWP_BRAKE, MOTOR_OUTPUT_CCWP_BRAKE
from phoenix6.signals import MotorAlignmentValue
import wpilib

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
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=constants.kDeployRatio),
    slot0=deploy_slot0,
)
lift_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    motor_name="LeftLift",
    parent_nt="Climber",
    motor_output=MOTOR_OUTPUT_CCWP_BRAKE,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=constants.kLiftRatio),
    slot0=lift_slot0,
)


class Climber(Subsystem):
    def __init__(self):
        """Initialize the Climber subsystem."""
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
            controls.Follower(
                self.left_lift_motor.device_id, MotorAlignmentValue.OPPOSED
            )
        )

        if wpilib.RobotBase.isSimulation():
            deploy_gearbox = DCMotor.falcon500(1)
            J_deploy = 0.001
            gearing_deploy = 1.0
            deploy_plant = LinearSystemId.DCMotorSystem(
                deploy_gearbox, J_deploy, gearing_deploy
            )
            self.deploy_motor.simulation_init(deploy_plant, deploy_gearbox)

            lift_gearbox = DCMotor.falcon500(1)
            J_lift = 0.001
            gearing_lift = 1.0
            lift_plant = LinearSystemId.DCMotorSystem(
                lift_gearbox, J_lift, gearing_lift
            )
            self.left_lift_motor.simulation_init(lift_plant, lift_gearbox)

    def _deploy_position(self, position: float) -> None:
        """Run the deploy motor to the specified position."""
        self.deploy_motor.set_control(
            controls.PositionVoltage(position, enable_foc=False)
        )

    def _stop_deploy(self) -> None:
        """Stop the deploy motor."""
        self.deploy_motor.stopMotor()

    def _lift_position(self, position: float) -> None:
        """Run the lift motor to the specified position."""
        self.left_lift_motor.set_control(
            controls.PositionVoltage(position, enable_foc=False)
        )

    def _stop_lift(self) -> None:
        """Stop the lift motor."""
        self.left_lift_motor.stopMotor()

    def _is_at_deploy_target(self) -> bool:
        """Check if the deploy motor is at the target position within tolerance."""
        tolerance = 0.5  # Adjust tolerance as needed
        current_position = self.deploy_motor.get_position().value
        target_position = self.deploy_motor.get_closed_loop_reference().value
        return abs(current_position - target_position) < tolerance

    # returns inline command to deploy climber to a position
    def deploy_to_position(self, position: float) -> Command:
        """Return a command to deploy the climber to the specified position."""
        return self.runOnce(lambda: self._deploy_position(position))

    # returns inline command to lift climber to a position
    def lift_to_position(self, position: float) -> Command:
        """Return a command to lift the climber to the specified position."""
        return self.runOnce(lambda: self._lift_position(position))
        # No need to wait for lift to reach position since it's follower

    def simulationPeriodic(self) -> None:
        """Update simulation state for the climber motors."""
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.deploy_motor.simulation_update(dt, battery_v)
        self.left_lift_motor.simulation_update(dt, battery_v, [self.right_lift_motor])
