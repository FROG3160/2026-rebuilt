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
from phoenix6 import controls
from FROGlib.ctre import MOTOR_OUTPUT_CWP_BRAKE, MOTOR_OUTPUT_CCWP_BRAKE
from phoenix6.signals import MotorAlignmentValue
import wpilib
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations
from phoenix6 import unmanaged
import numpy as np

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
    can_bus="rio",
    motor_name="LeftLift",
    parent_nt="Climber",
    motor_output=MOTOR_OUTPUT_CCWP_BRAKE,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
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
            self.simulationInit()

    def simulationInit(self) -> None:
        """Initialize simulation models for the climber motors."""
        # Deploy motor simulation
        deploy_gearbox = DCMotor.falcon500(1)
        J_deploy = 0.001  # kg·m² — tune based on real system
        gearing_deploy = 1.0
        deploy_plant = LinearSystemId.DCMotorSystem(
            deploy_gearbox, J_deploy, gearing_deploy
        )
        self.deploy_physim = DCMotorSim(
            deploy_plant, deploy_gearbox, np.array([0.0, 0.0])
        )
        self.deploy_physim.setState(0.0, 0.0)

        # Lift motor simulation (leader)
        lift_gearbox = DCMotor.falcon500(1)
        J_lift = 0.001  # kg·m² — tune based on real system
        gearing_lift = 1.0
        lift_plant = LinearSystemId.DCMotorSystem(lift_gearbox, J_lift, gearing_lift)
        self.lift_physim = DCMotorSim(lift_plant, lift_gearbox, np.array([0.0, 0.0]))
        self.lift_physim.setState(0.0, 0.0)

    def simulationPeriodic(self) -> None:
        """Update simulation state for the climber motors."""
        unmanaged.feed_enable(0.100)  # Required for Phoenix sim

        dt = 0.020  # WPILib sim timestep

        battery_v = wpilib.RobotController.getBatteryVoltage()

        # Set supply voltage on all motors
        self.deploy_motor.sim_state.set_supply_voltage(battery_v)
        self.left_lift_motor.sim_state.set_supply_voltage(battery_v)
        self.right_lift_motor.sim_state.set_supply_voltage(battery_v)

        # Deploy motor
        deploy_applied_v = self.deploy_motor.get_motor_voltage().value
        self.deploy_physim.setInputVoltage(deploy_applied_v)
        self.deploy_physim.update(dt)
        deploy_pos_rot = self.deploy_physim.getAngularPositionRotations()
        deploy_vel_rps = radiansToRotations(self.deploy_physim.getAngularVelocity())
        self.deploy_motor.sim_state.set_raw_rotor_position(deploy_pos_rot)
        self.deploy_motor.sim_state.set_rotor_velocity(deploy_vel_rps)

        # Lift motor (leader)
        lift_applied_v = self.left_lift_motor.get_motor_voltage().value
        self.lift_physim.setInputVoltage(lift_applied_v)
        self.lift_physim.update(dt)
        lift_pos_rot = self.lift_physim.getAngularPositionRotations()
        lift_vel_rps = radiansToRotations(self.lift_physim.getAngularVelocity())
        self.left_lift_motor.sim_state.set_raw_rotor_position(lift_pos_rot)
        self.left_lift_motor.sim_state.set_rotor_velocity(lift_vel_rps)
        # Follower motor (right lift) - same state as leader
        self.right_lift_motor.sim_state.set_raw_rotor_position(lift_pos_rot)
        self.right_lift_motor.sim_state.set_rotor_velocity(lift_vel_rps)

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
