from copy import deepcopy
from typing import Callable
from commands2 import Subsystem, Command
from phoenix6.hardware import TalonFX
from FROGlib.ctre import (
    FROGTalonFX,
    get_frog_talon_config,
    MOTOR_OUTPUT_CWP_BRAKE,
    MOTOR_OUTPUT_CCWP_BRAKE,
)
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    MotorOutputConfigs,
)
import constants
from wpimath.system.plant import DCMotor, LinearSystemId
from phoenix6 import controls, SignalLogger
from phoenix6.signals import MotorAlignmentValue
import wpilib
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from phoenix6.configs import MotionMagicConfigs
from FROGlib.subsystem import FROGSubsystem
deploy_slot0 = (
    Slot0Configs()
    .with_k_s(constants.kDeployS)
    .with_k_v(constants.kDeployV)
    .with_k_p(constants.kDeployP)
    .with_k_i(constants.kDeployI)
    .with_k_d(constants.kDeployD)
)
lift_slot0 = (
    Slot0Configs()
    .with_k_s(constants.kLiftS)
    .with_k_v(constants.kLiftV)
    .with_k_g(constants.kLiftG)
    .with_k_p(constants.kLiftP)
    .with_k_i(constants.kLiftI)
    .with_k_d(constants.kLiftD)
)
deploy_motion_magic_config = (
    MotionMagicConfigs()
    .with_motion_magic_cruise_velocity(constants.kDeployMM_V)
    .with_motion_magic_acceleration(constants.kDeployMM_A)
)
deploy_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CCWP_BRAKE)
    .with_motion_magic(deploy_motion_magic_config)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(constants.kDeployRatio))
    .with_slot0(deploy_slot0)
)
lift_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CWP_BRAKE)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(constants.kLiftRatio))
    .with_slot0(lift_slot0)
)


class Climber(FROGSubsystem):
    def __init__(self):
        """Initialize the Climber subsystem."""
        super().__init__()
        self.deploy_motor = FROGTalonFX(
            id=constants.kClimberDeployMotorID,
            motor_config=deploy_motor_config,
            motor_name="Deploy",
            signal_profile=FROGTalonFX.SignalProfile.POSITION_MM,
        )
        self.left_lift_motor = FROGTalonFX(
            id=constants.kClimberLeftLiftMotorID,
            motor_config=deepcopy(lift_motor_config),
            motor_name="LeftLift",
            signal_profile=FROGTalonFX.SignalProfile.FLYWHEEL,
        )
        self.right_lift_motor = FROGTalonFX(
            id=constants.kClimberRightLiftMotorID,
            motor_config=deepcopy(lift_motor_config),
            motor_name="RightLift",
            signal_profile=FROGTalonFX.SignalProfile.FOLLOWER,
        )
        self.right_lift_motor.set_control(
            controls.Follower(
                self.left_lift_motor.device_id, MotorAlignmentValue.OPPOSED
            )
        )

        # Set up SysID routine for the deploy motor
        self.sys_id_routine_deploy = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "state-climber-deploy", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda voltage: self.deploy_motor.set_control(
                    controls.VoltageOut(voltage, enable_foc=False)
                ),
                lambda log: None,
                self,
            ),
        )

        # Set up SysID routine for the lift motors
        self.sys_id_routine_lift = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "state-climber-lift", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda voltage: self.left_lift_motor.set_control(
                    controls.VoltageOut(voltage, enable_foc=False)
                ),
                lambda log: None,
                self,
            ),
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
            controls.MotionMagicVoltage(position, enable_foc=False)
        )

    def _set_deploy_voltage(self, volts: float) -> None:
        """Run the deploy motor at the specified voltage."""
        self.deploy_motor.set_control(controls.VoltageOut(volts))

    def _stop_deploy(self) -> None:
        """Stop the deploy motor."""
        self.deploy_motor.stopMotor()

    def _lift_position(self, position: float) -> None:
        """Run the lift motor to the specified position."""
        self.left_lift_motor.set_control(
            controls.PositionVoltage(position, enable_foc=False)
        )

    def _set_lift_voltage(self, volts: float) -> None:
        """Run the lift motor at the specified voltage."""
        self.left_lift_motor.set_control(controls.VoltageOut(volts))

    def _stop_lift(self) -> None:
        """Stop the lift motor."""
        self.left_lift_motor.stopMotor()

    def _is_at_deploy_target(self) -> bool:
        """Check if the deploy motor is at the target position within tolerance."""
        tolerance = 0.5  # Adjust tolerance as needed
        current_position = self.deploy_motor.get_position().value
        target_position = self.deploy_motor.get_closed_loop_reference().value
        return abs(current_position - target_position) < tolerance

    def sysIdQuasistaticDeploy(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_deploy.quasistatic(direction)

    def sysIdDynamicDeploy(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_deploy.dynamic(direction)

    def sysIdQuasistaticLift(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_lift.quasistatic(direction)

    def sysIdDynamicLift(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_lift.dynamic(direction)

    def is_deployed(self) -> bool:
        """Returns True if the deploy motor is at the deployed position."""
        # Check if the deploy motor is at least 90% of the way to the deployed position
        return self.deploy_motor.get_position().value > (
            constants.kClimberDeployed * 0.9
        )

    def lift_forward_cmd(self) -> Command:
        """Run the lift motors forward at 1 V until interrupted."""
        return self.manual_lift_voltage_cmd(lambda: 1.0).withName("Lift Forward 1V")

    def lift_reverse_cmd(self) -> Command:
        """Run the lift motors in reverse at 1 V until interrupted."""
        return self.manual_lift_voltage_cmd(lambda: -1.0).withName(
            "Lift Reverse 1V"
        )

    def deploy_cmd(self) -> Command:
        """Drive the climber arm to the fully deployed position using MotionMagic (one-shot)."""
        return self.runOnce(
            lambda: self._deploy_position(constants.kClimberDeployed),
        )

    def stow_cmd(self) -> Command:
        """Drive the climber arm to the stowed position using MotionMagic (one-shot)."""
        return self.runOnce(
            lambda: self._deploy_position(constants.kClimberStowed),
        )

    # returns inline command to lift climber to a position
    def lift_to_position_cmd(self, position: float) -> Command:
        """Drive the lift motors to the specified position using position control (one-shot)."""
        return self.runOnce(lambda: self._lift_position(position))
        # No need to wait for lift to reach position since it's follower

    def manual_deploy_voltage_cmd(self, volts: float) -> Command:
        """Apply a constant voltage to the deploy motor while the command is active, stopping on end."""
        return self.runEnd(lambda: self._set_deploy_voltage(volts), self._stop_deploy)

    def manual_lift_voltage_cmd(self, voltage_supplier: Callable[[], float]) -> Command:
        """Apply a dynamic voltage from *voltage_supplier* to the lift motors while the command is active, stopping on end."""
        return self.runEnd(
            lambda: self._set_lift_voltage(voltage_supplier()), self._stop_lift
        )

    def simulationPeriodic(self) -> None:
        """Update simulation state for the climber motors."""
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.deploy_motor.simulation_update(dt, battery_v)
        self.left_lift_motor.simulation_update(dt, battery_v, [self.right_lift_motor])

    @FROGSubsystem.telemetry("Deploy Position")
    def deploy_position_telem(self) -> float:
        return self.deploy_motor.get_position().value

    @FROGSubsystem.telemetry("Lift Position")
    def lift_position_telem(self) -> float:
        return self.left_lift_motor.get_position().value

    @FROGSubsystem.telemetry("Deploy Velocity")
    def deploy_velocity_telem(self) -> float:
        return self.deploy_motor.get_velocity().value

    @FROGSubsystem.telemetry("Lift Velocity")
    def lift_velocity_telem(self) -> float:
        return self.left_lift_motor.get_velocity().value
