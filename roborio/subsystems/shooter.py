from copy import deepcopy
from commands2 import Command
from wpimath.units import inchesToMeters, volts
from wpimath.system.plant import DCMotor, LinearSystemId
from phoenix6.hardware import TalonFX
from phoenix6.configs import MotionMagicConfigs
from FROGlib.ctre import (
    FROGTalonFX,
    get_frog_talon_config,
    MOTOR_OUTPUT_CWP_BRAKE,
    MOTOR_OUTPUT_CWP_COAST,
    MOTOR_OUTPUT_CCWP_COAST,
    MOTOR_OUTPUT_CCWP_BRAKE,
)
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    MotorOutputConfigs,
)
import constants
from phoenix6 import controls, SignalLogger
from FROGlib.utils import DriveTrain
from wpiutil import Sendable, SendableBuilder
import wpilib
from commands2.button import Trigger
from FROGlib.ctre import MAX_FALCON_RPM, MAX_KRAKEN_X60_RPM
from commands2 import cmd
from phoenix6.signals import MotorAlignmentValue
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from FROGlib.subsystem import FROGSubsystem
import numpy as np

flywheel_gearing = DriveTrain(
    gear_stages=[], wheel_diameter=inchesToMeters(4.0)
)  # for velocity conversions, etc.

flywheel_slot0 = (
    Slot0Configs()
    .with_k_s(constants.Shooter.FLYWHEEL_S)
    .with_k_v(constants.Shooter.FLYWHEEL_V)
    .with_k_a(constants.Shooter.FLYWHEEL_A)
    .with_k_p(constants.Shooter.FLYWHEEL_P)
    .with_k_i(constants.Shooter.FLYWHEEL_I)
    .with_k_d(constants.Shooter.FLYWHEEL_D)
)
hood_slot0 = (
    Slot0Configs()
    .with_k_s(constants.Shooter.HOOD_S)
    .with_k_p(constants.Shooter.HOOD_P)
    .with_k_g(constants.Shooter.HOOD_G)
    .with_k_v(constants.Shooter.HOOD_V)
)

flywheel_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CCWP_COAST)
    .with_feedback(
        FeedbackConfigs().with_sensor_to_mechanism_ratio(
            flywheel_gearing.system_reduction
        )
    )
    .with_slot0(flywheel_slot0)
)

hood_motion_magic_config = (
    MotionMagicConfigs()
    .with_motion_magic_cruise_velocity(constants.Shooter.HOOD_MMV)
    .with_motion_magic_acceleration(constants.Shooter.HOOD_MMA)
)
hood_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CWP_BRAKE)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(1.0))
    .with_motion_magic(hood_motion_magic_config)
    .with_slot0(hood_slot0)
)


from typing import Callable, Optional


class Shooter(FROGSubsystem):
    def __init__(self, distance_to_target_supplier: Callable[[], Optional[float]]):
        super().__init__()
        self.motor = FROGTalonFX(
            id=constants.CANIDs.SHOOTER_LEFT_FLYWHEEL,
            motor_config=deepcopy(flywheel_motor_config).with_motor_output(
                MOTOR_OUTPUT_CCWP_COAST
            ),
            canbus="rio",
            motor_name="LeftFlywheel",
            signal_profile=FROGTalonFX.SignalProfile.FLYWHEEL,
        )
        self._follower = FROGTalonFX(
            id=constants.CANIDs.SHOOTER_RIGHT_FLYWHEEL,
            motor_config=deepcopy(flywheel_motor_config).with_slot0(Slot0Configs()),
            canbus="rio",
            motor_name="RightFlywheel",
            signal_profile=FROGTalonFX.SignalProfile.FOLLOWER,
        )
        self._follower.set_control(
            controls.Follower(self.motor.device_id, MotorAlignmentValue.OPPOSED)
        )

        self._slot0 = deepcopy(flywheel_slot0)
        self._hood_slot0 = deepcopy(hood_slot0)

        self.distance_to_target_supplier = distance_to_target_supplier

        self.hood_motor = FROGTalonFX(
            id=constants.CANIDs.HOOD_MOTOR,
            motor_config=hood_motor_config,
            canbus="rio",
            motor_name="Hood Motor",
            signal_profile=FROGTalonFX.SignalProfile.POSITION_MM,
        )

        self._flywheel_tolerance = (
            constants.Shooter.FLYWHEEL_TOLERANCE
        )  # RPM tolerance for "at speed" check

        # used in simulationPeriodic to track simulated velocity
        # these attributes won't show as being referenced in the code,
        # but they're referenced by name as a string in simulationPeriodic.
        self._commanded_flywheel_speed = 0.0
        self._speed_multiplier = 1.00
        if wpilib.RobotBase.isSimulation():
            # Flywheel simulation - override gearing to 1.0 (actual mechanical ratio)
            # as the config uses rotations/meter (~3.13)
            self.motor.simulation_init(
                moi=0.001, motor_model=DCMotor.falcon500(2), gearing=1.0
            )

            # Hood simulation - override gearing to 60.0 (actual mechanical ratio)
            # as the config uses 1.0
            self.hood_motor.simulation_init(
                moi=0.001, motor_model=DCMotor.falcon500(1), gearing=60.0
            )

        # Set up SysID routine for the shooter
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage=4.0,
            ),
            SysIdRoutine.Mechanism(
                lambda voltage: self.motor.set_control(
                    controls.VoltageOut(voltage, enable_foc=False)
                ),
                lambda log: None,
                self,
            ),
        )

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)

    def _apply_speed_by_distance(self):
        """Gets speed from the distance to the target, then applies the speed to the flywheel motors"""
        if speed := self._get_speed_from_distance():
            self._set_commanded_speed(speed)
        else:
            self._set_commanded_speed(0.0)
        self._apply_commanded_speed()

    def _set_commanded_speed(self, speed: float):
        self._commanded_flywheel_speed = speed * self._speed_multiplier

    def _apply_commanded_speed(self):
        self.motor.set_control(controls.VelocityVoltage(self._commanded_flywheel_speed))

    def _get_speed_from_distance(self) -> float | None:
        """
        Calculates speed based on an interpolation map for the Hub target.
        (Distance, Speed) points: (2.06, 17.12), (2.2, 17.61), (2.89, 19.25), (3.57, 20.76), (5.05, 23.05)
        """
        if distance := self.distance_to_target_supplier():
            # TODO: Differentiate flywheel speed between Hub and Floor targets based on field zone or aim state.
            return float(
                np.interp(
                    distance,
                    constants.Shooter.SHOOTERS_HUB_DISTANCES,
                    constants.Shooter.SHOOTERS_HUB_SPEEDS,
                )
            )
        else:
            return None

    def get_commanded_speed(self) -> float:
        return self._commanded_flywheel_speed

    # boolean to indicate if flywheel is at target speed
    def is_at_speed(self) -> bool:
        error = self.motor.get_closed_loop_error().value
        speed = self.motor.get_velocity().value
        return abs(error) <= self._flywheel_tolerance and speed > 0

    def _stop_flywheel(self):
        self.motor.stopMotor()

    def fire_with_distance_cmd(self) -> Command:
        """Spin up the flywheel to a speed interpolated from the current target distance, stopping when interrupted."""
        return self.runEnd(self._apply_speed_by_distance, self._stop_flywheel)

    def fire_at_set_speed_cmd(self) -> Command:
        """Spin up the flywheel to the last commanded speed, stopping when interrupted."""
        return self.runEnd(self._apply_commanded_speed, self._stop_flywheel)

    def deploy_hood(self):
        self.hood_motor.set_control(
            controls.MotionMagicVoltage(constants.Shooter.HOOD_FORWARD_LIMIT)
        )

    def retract_hood(self):
        self.hood_motor.set_control(
            controls.MotionMagicVoltage(constants.Shooter.HOOD_REVERSE_LIMIT)
        )

    def _set_hood_position(self):
        self.hood_motor.set_position(0.0)

    def zero_hood_cmd(self) -> Command:
        """Drive the hood slowly into its reverse hard stop, zero the position sensor, then stop."""
        return self.runOnce(self._set_hood_position)

    def reset_hood_cmd(self) -> Command:
        """Sets the hood position to 1.3 and retracts it."""
        return cmd.sequence(
            self.runOnce(lambda: self.hood_motor.set_position(1.3)),
            self.runOnce(self.retract_hood)
        )

        # return (
        #     self.runOnce(
        #         lambda: self.hood_motor.set_control(
        #             controls.VoltageOut(constants.Shooter.HOOD_HOMING_VOLTAGE)
        #         )
        #     )
        #     .andThen(
        #         cmd.waitUntil(lambda: self.hood_motor.get_stator_current().value > constants.Shooter.HOOD_HOMING_CURRENT)  # type: ignore
        #     )
        #     .andThen(self.runOnce(self.hood_motor.stopMotor))
        #     .andThen(self.runOnce(self._set_hood_position))
        # )

    def is_hood_open(self) -> bool:
        """Returns True if the hood is currently open (position within 0.05 of forward limit)"""

        return self.hood_motor.get_position().value > (
            constants.Shooter.HOOD_FORWARD_LIMIT
            - constants.Shooter.HOOD_POSITION_TOLERANCE
        )

    def is_hood_closed(self) -> bool:
        """Returns True if the hood is currently closed (position within 0.05 of reverse limit)"""
        return self.hood_motor.get_position().value < (
            constants.Shooter.HOOD_REVERSE_LIMIT
            + constants.Shooter.HOOD_POSITION_TOLERANCE
        )

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(dt, battery_v, [self._follower])
        self.hood_motor.simulation_update(dt, battery_v)

    def _updateFlywheelSlot(self, **kwargs):
        for k, v in kwargs.items():
            if hasattr(self._slot0, k):
                setattr(self._slot0, k, v)
        self.motor.configurator.apply(self._slot0)

    def _updateHoodSlot(self, **kwargs):
        for k, v in kwargs.items():
            if hasattr(self._hood_slot0, k):
                setattr(self._hood_slot0, k, v)
        self.hood_motor.configurator.apply(self._hood_slot0)

    @FROGSubsystem.telemetry("Flywheel Commanded Speed")
    def commanded_speed_telem(self) -> float:
        return self._commanded_flywheel_speed

    @FROGSubsystem.telemetry("Flywheel Rotor Velocity")
    def flywheel_rotor_velocity_telem(self) -> float:
        return self.motor.get_rotor_velocity().value

    @FROGSubsystem.telemetry("Flywheel Mech Velocity")
    def flywheel_mech_velocity_telem(self) -> float:
        return self.motor.get_velocity().value

    @FROGSubsystem.telemetry("Flywheel Closed Loop Ref.")
    def flywheel_closed_loop_ref_telem(self) -> float:
        return self.motor.get_closed_loop_reference().value

    @FROGSubsystem.telemetry("Flywheel Closed Loop Error")
    def flywheel_closed_loop_error_telem(self) -> float:
        return self.motor.get_closed_loop_error().value

    @FROGSubsystem.telemetry("Hood Position")
    def hood_position_telem(self) -> float:
        return self.hood_motor.get_position().value

    @FROGSubsystem.telemetry("Hood Velocity")
    def hood_velocity_telem(self) -> float:
        return self.hood_motor.get_velocity().value

    @FROGSubsystem.telemetry("At Speed")
    def at_speed_telem(self) -> bool:
        return self.is_at_speed()

    @FROGSubsystem.telemetry("Hood Open")
    def hood_open_telem(self) -> bool:
        return self.is_hood_open()

    @FROGSubsystem.telemetry("Hood Closed")
    def hood_closed_telem(self) -> bool:
        return self.is_hood_closed()

    @FROGSubsystem.tunable(constants.Shooter.FLYWHEEL_S, "Flywheel K_S")
    def flywheel_ks(self, val):
        self._updateFlywheelSlot(k_s=val)

    @FROGSubsystem.tunable(constants.Shooter.FLYWHEEL_V, "Flywheel K_V")
    def flywheel_kv(self, val):
        self._updateFlywheelSlot(k_v=val)

    @FROGSubsystem.tunable(constants.Shooter.FLYWHEEL_A, "Flywheel K_A")
    def flywheel_ka(self, val):
        self._updateFlywheelSlot(k_a=val)

    @FROGSubsystem.tunable(constants.Shooter.FLYWHEEL_P, "Flywheel K_P")
    def flywheel_kp(self, val):
        self._updateFlywheelSlot(k_p=val)

    @FROGSubsystem.tunable(constants.Shooter.FLYWHEEL_I, "Flywheel K_I")
    def flywheel_ki(self, val):
        self._updateFlywheelSlot(k_i=val)

    @FROGSubsystem.tunable(constants.Shooter.FLYWHEEL_D, "Flywheel K_D")
    def flywheel_kd(self, val):
        self._updateFlywheelSlot(k_d=val)

    @FROGSubsystem.tunable(1.00, "Flywheel Speed Multiplier")
    def speed_multiplier(self, val):
        self._speed_multiplier = val

    @FROGSubsystem.tunable(0.0, "Flywheel Commanded Speed")
    def commanded_speed(self, val):
        self._commanded_flywheel_speed = val

    @FROGSubsystem.tunable(constants.Shooter.FLYWHEEL_TOLERANCE, "Flywheel Tolerance")
    def flywheel_tolerance(self, val):
        self._flywheel_tolerance = val

    @FROGSubsystem.tunable(constants.Shooter.HOOD_S, "Hood K_S")
    def hood_ks(self, val):
        self._updateHoodSlot(k_s=val)

    @FROGSubsystem.tunable(constants.Shooter.HOOD_P, "Hood K_P")
    def hood_kp(self, val):
        self._updateHoodSlot(k_p=val)
