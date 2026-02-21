from copy import deepcopy
import numpy as np
from commands2 import Command, Subsystem
from wpimath.units import inchesToMeters
from phoenix6.hardware import TalonFX
from phoenix6.configs import SoftwareLimitSwitchConfigs
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
)
import constants
from phoenix6 import controls
from FROGlib.ctre import (
    MOTOR_OUTPUT_CWP_COAST,
    MOTOR_OUTPUT_CCWP_COAST,
    MOTOR_OUTPUT_CCWP_BRAKE,
)
from FROGlib.utils import DriveTrain
from subsystems.drive import Drive
from phoenix6.configs import SlotConfigs
from wpiutil import Sendable, SendableBuilder
import wpilib
from commands2.button import Trigger
from FROGlib.ctre import MAX_FALCON_RPM, MAX_KRAKEN_X60_RPM
from phoenix6 import unmanaged
import math
from wpilib.simulation import DCMotorSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations
from commands2 import cmd
from phoenix6.signals import MotorAlignmentValue

flywheel_gearing = DriveTrain(
    gear_stages=[], wheel_diameter=inchesToMeters(4.0)
)  # for velocity conversions, etc.

flywheel_slot0 = FROGSlotConfig(
    k_s=constants.kFlywheelS,
    k_v=constants.kFlywheelV,
    k_a=constants.kFlywheelA,
    k_p=constants.kFlywheelP,
    k_i=constants.kFlywheelI,
    k_d=constants.kFlywheelD,
)
hood_slot0 = FROGSlotConfig(k_s=constants.kHoodS, k_p=constants.kHoodP)

flywheel_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Shooter",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(
        sensor_to_mechanism_ratio=flywheel_gearing.system_reduction
    ),
    slot0=flywheel_slot0,
)

hood_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Shooter",
    motor_output=MOTOR_OUTPUT_CCWP_BRAKE,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=60.0),
    slot0=hood_slot0,
)

hood_software_limits = (
    SoftwareLimitSwitchConfigs()
    .with_forward_soft_limit_threshold(constants.kHoodForwardLimit)
    .with_reverse_soft_limit_threshold(constants.kHoodReverseLimit)
    .with_forward_soft_limit_enable(True)
    .with_forward_soft_limit_enable(True)
)


class Shooter(Subsystem):
    def __init__(self, drive: Drive):
        self.motor = FROGTalonFX(
            motor_config=deepcopy(flywheel_motor_config)
            .with_id(constants.kShooterLeftFlywheelID)
            .with_motor_name("LeftFlywheel")
            .with_motor_output(MOTOR_OUTPUT_CCWP_COAST)
        )
        self._follower = FROGTalonFX(
            motor_config=deepcopy(flywheel_motor_config)
            .with_id(constants.kShooterRightFlywheelID)
            .with_motor_name("RightFlywheel")
            .with_slot0(FROGSlotConfig())
        )
        self._follower.set_control(
            controls.Follower(self.motor.device_id, MotorAlignmentValue.OPPOSED)
        )

        self.drive = drive

        self.hood_motor = FROGTalonFX(
            motor_config=FROGTalonFXConfig(hood_motor_config)
            .with_id(constants.kHoodMotorID)
            .with_motor_name("Hood Motor")
        )

        self._flywheel_tolerance = (
            constants.kFlywheelTolerance
        )  # RPM tolerance for "at speed" check

        # used in simulationPeriodic to track simulated velocity
        # these attributes won't show as being referenced in the code,
        # but they're referenced by name as a string in simulationPeriodic.
        self._commanded_flywheel_speed = 0.0
        if wpilib.RobotBase.isSimulation():
            self.simulationInit()

    def _apply_speed_by_distance(self):
        """Gets speed from the distance to the target, then applies the speed to the flywheel motors"""
        if speed := self._get_speed_from_distance():
            self._set_commanded_speed(speed)
        else:
            self._set_commanded_speed(0.0)
        self._apply_commanded_speed()

    def _set_commanded_speed(self, speed: float):
        self._commanded_flywheel_speed = speed

    def _apply_commanded_speed(self):
        self.motor.set_control(controls.VelocityVoltage(self._commanded_flywheel_speed))

    def _get_speed_from_distance(self) -> float | None:
        """
        Linearly interpolates speed based on distance.

        Given:
        - 1 meter → 7 units speed
        - 4 meters → 30 units speed

        Returns speed as a float.
        Uses exact fraction: speed = (23 * distance - 2) / 3
        """
        if distance := self.drive.get_distance_to_target():
            return (23 * distance - 2) / 3
        else:
            return None

    def get_commanded_speed(self) -> float:
        return self._commanded_flywheel_speed

    # boolean to indicate if flywheel is at target speed
    def is_at_speed(self) -> bool:
        error = self.motor.get_closed_loop_error().value
        speed = self._get_flywheel_velocity()
        return abs(error) <= self._flywheel_tolerance and speed > 0

    def _stop_flywheel(self):
        self.motor.stopMotor()

    # generate a command to fire
    def cmd_fire_with_distance(self) -> Command:
        return self.runEnd(self._apply_speed_by_distance, self._stop_flywheel)

    def cmd_fire_at_set_speed(self) -> Command:
        return self.runEnd(self._apply_commanded_speed, self._stop_flywheel)

    def _set_hood_position(self):
        self.hood_motor.set_position(0)
        self.hood_motor.config.software_limit_switch = hood_software_limits
        self.hood_motor.configurator.apply(self.hood_motor.config)

    def zero_hood_command(self):
        return (
            self.runOnce(
                lambda: self.hood_motor.set_control(controls.VoltageOut(-0.18))
            )
            .andThen(
                cmd.waitUntil(lambda: self.hood_motor.get_torque_current().value < -4)  # type: ignore
            )
            .andThen(self.runOnce(self.hood_motor.stopMotor))
            .andThen(self.runOnce(self._set_hood_position))
        )

    def simulationInit(self):
        flywheel_gearbox = DCMotor.falcon500(
            2
        )  # 2 motors → doubles torque, halves current per motor

        # Create the linear plant (dynamics model)
        # Parameters: gearbox, J (inertia kg·m²), gearing (reduction ratio, >1 if motor spins faster than mechanism)
        #   - Use your actual reflected inertia at the rotor (including flywheel mass, radius^2, etc.)
        #   - Gearing: rotor revolutions per mechanism revolution (often ~1–2 for shooters)
        J_flywheel = 0.001  # kg·m² — tune this based on real ramp-up time
        gearing = 1.0

        flywheel_plant = LinearSystemId.DCMotorSystem(
            flywheel_gearbox,  # DCMotor with count=2
            J_flywheel,  # moment of inertia
            gearing,  # reduction ratio
        )

        # Now instantiate DCMotorSim with required args
        # measurementStdDevs = [pos_noise_std, vel_noise_std] — use zeros for clean/no noise
        self.motor_physim = DCMotorSim(
            flywheel_plant,
            flywheel_gearbox,  # same gearbox object
            np.array(
                [0.0, 0.0]
            ),  # or [0.01, 0.2] for realistic sensor noise if desired
        )

        self.motor_physim.setState(0.0, 0.0)  # pos (rad), vel (rad/s)

    def simulationPeriodic(self):
        unmanaged.feed_enable(0.100)  # Required for Phoenix sim

        dt = 0.020  # WPILib sim timestep

        battery_v = wpilib.RobotController.getBatteryVoltage()

        # Set supply voltage on all motors
        self.motor.sim_state.set_supply_voltage(battery_v)
        self._follower.sim_state.set_supply_voltage(battery_v)

        # Flywheel: since left follows right, use leader's (right) voltage for the combined model
        flywheel_applied_v = (
            self.motor.get_motor_voltage().value
        )  # volts (follower should match)

        # Advance flywheel physics (combined 2-motor model)
        self.motor_physim.setInputVoltage(flywheel_applied_v)
        self.motor_physim.update(dt)

        # Extract states (convert rad → rotations, rad/s → rps)
        flywheel_pos_rot = self.motor_physim.getAngularPositionRotations()
        flywheel_vel_rps = radiansToRotations(self.motor_physim.getAngularVelocity())

        # Feed back same states to BOTH flywheel motors' sim_states (coupled mechanism)
        self.motor.sim_state.set_raw_rotor_position(flywheel_pos_rot)
        self.motor.sim_state.set_rotor_velocity(flywheel_vel_rps)
        self._follower.sim_state.set_raw_rotor_position(flywheel_pos_rot)
        self._follower.sim_state.set_rotor_velocity(flywheel_vel_rps)

    def _updateFlywheelSlot0(self, **kwargs):
        slot0 = SlotConfigs()
        self.motor.configurator.refresh(slot0)
        for k, v in kwargs.items():
            setattr(slot0, k, v)
        self.motor.configurator.apply(slot0)

    def _get_slot0_param(self, param: str) -> float:
        # Use .configs for a cached/snapshot view (safer in Sendable callbacks)
        return getattr(self.motor.config.slot0, param)

    def _get_flywheel_velocity(self) -> float:
        return self.motor.get_velocity().value

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)
        builder.setSmartDashboardType("Shooter")  # or "Flywheel Tunables" etc.

        # Read-only telemetry (shown in main widget)
        builder.addDoubleProperty(
            "Flywheel Rotor Velocity",
            lambda: self.motor.get_rotor_velocity().value,
            lambda: None,
        )
        builder.addDoubleProperty(
            "Flywheel Mech Velocity",
            lambda: self.motor.get_velocity().value,
            lambda: None,
        )
        builder.addDoubleProperty(
            "Flywheel Closed Loop Ref.",
            lambda: self.motor.get_closed_loop_reference().value,
            lambda: None,
        )
        builder.addDoubleProperty(
            "Flywheel Closed Loop Error",
            lambda: self.motor.get_closed_loop_error().value,
            lambda: None,
        )
        builder.addDoubleProperty(
            "Flywheel Commanded Speed",
            lambda: self._commanded_flywheel_speed,
            lambda value: setattr(self, "_commanded_flywheel_speed", value),
        )
        builder.addBooleanProperty(
            "At Speed", lambda: self.is_at_speed(), lambda: None
        )  # if you track target

        # Tunable gains – these are editable
        for param in ["k_s", "k_v", "k_a", "k_p", "k_i", "k_d"]:
            builder.addDoubleProperty(
                f"tunables/Flywheel {param.upper()}",
                lambda p=param: self._get_slot0_param(p),
                lambda v, p=param: self._updateFlywheelSlot0(**{p: v}),
            )
