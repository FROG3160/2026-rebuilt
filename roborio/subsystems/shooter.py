from copy import deepcopy
from commands2 import Command
from wpimath.units import inchesToMeters, volts
from wpimath.system.plant import DCMotor, LinearSystemId
from phoenix6.hardware import TalonFX
from phoenix6.configs import SoftwareLimitSwitchConfigs
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
)
import constants
from phoenix6 import controls, SignalLogger
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
from commands2 import cmd
from phoenix6.signals import MotorAlignmentValue
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
from FROGlib.subsystem import FROGSubsystem

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


class Shooter(FROGSubsystem):
    def __init__(self, drive: Drive):
        super().__init__()
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

        self._slot0 = deepcopy(flywheel_slot0)
        self._hood_slot0 = deepcopy(hood_slot0)

        self.drive = drive

        self.hood_motor = FROGTalonFX(
            motor_config=hood_motor_config.with_id(
                constants.kHoodMotorID
            ).with_motor_name("Hood Motor")
        )

        self._flywheel_tolerance = (
            constants.kFlywheelTolerance
        )  # RPM tolerance for "at speed" check

        # used in simulationPeriodic to track simulated velocity
        # these attributes won't show as being referenced in the code,
        # but they're referenced by name as a string in simulationPeriodic.
        self._commanded_flywheel_speed = 0.0
        if wpilib.RobotBase.isSimulation():
            flywheel_gearbox = DCMotor.falcon500(2)
            J_flywheel = 0.001
            gearing = 1.0
            flywheel_plant = LinearSystemId.DCMotorSystem(
                flywheel_gearbox, J_flywheel, gearing
            )
            self.motor.simulation_init(flywheel_plant, flywheel_gearbox)

        # Set up SysID routine for the shooter
        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string(
                    "state", SysIdRoutineLog.stateEnumToString(state)
                ),
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
        speed = self.motor.get_velocity().value
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

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(dt, battery_v, [self._follower])

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

    @FROGSubsystem.telemetry("At Speed")
    def at_speed_telem(self) -> bool:
        return self.is_at_speed()

    @FROGSubsystem.tunable(constants.kFlywheelS, "Flywheel K_S")
    def flywheel_ks(self, val):
        self._updateFlywheelSlot(k_s=val)

    @FROGSubsystem.tunable(constants.kFlywheelV, "Flywheel K_V")
    def flywheel_kv(self, val):
        self._updateFlywheelSlot(k_v=val)

    @FROGSubsystem.tunable(constants.kFlywheelA, "Flywheel K_A")
    def flywheel_ka(self, val):
        self._updateFlywheelSlot(k_a=val)

    @FROGSubsystem.tunable(constants.kFlywheelP, "Flywheel K_P")
    def flywheel_kp(self, val):
        self._updateFlywheelSlot(k_p=val)

    @FROGSubsystem.tunable(constants.kFlywheelI, "Flywheel K_I")
    def flywheel_ki(self, val):
        self._updateFlywheelSlot(k_i=val)

    @FROGSubsystem.tunable(constants.kFlywheelD, "Flywheel K_D")
    def flywheel_kd(self, val):
        self._updateFlywheelSlot(k_d=val)

    @FROGSubsystem.tunable(0.0, "Flywheel Commanded Speed")
    def commanded_speed(self, val):
        self._commanded_flywheel_speed = val

    @FROGSubsystem.tunable(constants.kHoodS, "Hood K_S")
    def hood_ks(self, val):
        self._updateHoodSlot(k_s=val)

    @FROGSubsystem.tunable(constants.kHoodP, "Hood K_P")
    def hood_kp(self, val):
        self._updateHoodSlot(k_p=val)
