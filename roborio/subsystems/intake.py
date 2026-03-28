import math
from typing import Callable
import commands2
from commands2 import Command
from phoenix6.hardware import TalonFX
from FROGlib.ctre import (
    FROGTalonFX,
    get_frog_talon_config,
    MOTOR_OUTPUT_CWP_COAST,
    MOTOR_OUTPUT_CCWP_COAST,
    MOTOR_OUTPUT_CWP_BRAKE,
)
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    MotorOutputConfigs,
    CurrentLimitsConfigs,
    MotionMagicConfigs,
)
import constants
from phoenix6 import controls
from FROGlib.subsystem import FROGSubsystem
import wpilib
from wpimath.system.plant import DCMotor, LinearSystemId

# Falcon 500 direct-drives a 1-inch diameter roller (no intermediate gearing).
# 1 motor rotation = π × 0.0254 m of surface travel.
# sensor_to_mechanism_ratio converts motor RPS → mechanism units (m/s surface speed).
_ROLLER_DIAMETER_M = 0.0254  # 1 inch in meters
_SENSOR_TO_MECHANISM_RATIO = 1.0 / (math.pi * _ROLLER_DIAMETER_M)  # ≈ 12.5331

intake_slot0 = (
    Slot0Configs()
    .with_k_s(constants.Intake.VOLTAGE_INTAKE_S)
    .with_k_v(constants.Intake.INTAKE_V)
    .with_k_p(constants.Intake.INTAKE_P)
)

intake_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CCWP_COAST)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(_SENSOR_TO_MECHANISM_RATIO))
    .with_slot0(intake_slot0)
)

intake_deploy_slot0 = (
    Slot0Configs()
    .with_k_s(constants.Intake.INTAKE_DEPLOY_S)
    .with_k_v(constants.Intake.INTAKE_DEPLOY_V)
    .with_k_p(constants.Intake.INTAKE_DEPLOY_P)
    .with_k_i(constants.Intake.INTAKE_DEPLOY_I)
    .with_k_d(constants.Intake.INTAKE_DEPLOY_D)
)

intake_deploy_mm = (
    MotionMagicConfigs()
    .with_motion_magic_cruise_velocity(constants.Intake.INTAKE_DEPLOY_MM_V)
    .with_motion_magic_acceleration(constants.Intake.INTAKE_DEPLOY_MM_A)
)

intake_deploy_current_limits = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(constants.Intake.INTAKE_DEPLOY_CURRENT_LIMIT)
    .with_stator_current_limit_enable(True)
)

intake_deploy_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CWP_BRAKE)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(1.0 / constants.Intake.INTAKE_DEPLOY_DISTANCE_PER_ROTATION))
    .with_slot0(intake_deploy_slot0)
    .with_motion_magic(intake_deploy_mm)
    .with_current_limits(intake_deploy_current_limits)
)

class Intake(FROGSubsystem):
    def __init__(self, robot_speed_supplier: Callable[[], float] = lambda: 0.0):
        super().__init__()
        self._robot_speed_supplier = robot_speed_supplier
        self.motor = FROGTalonFX(
            id=constants.CANIDs.INTAKE_MOTOR,
            motor_config=intake_motor_config,
            canbus="rio",
            motor_name="Intake",
        )
        self.deploy_motor = FROGTalonFX(
            id=constants.CANIDs.INTAKE_DEPLOY_MOTOR,
            motor_config=intake_deploy_motor_config,
            canbus="rio",
            motor_name="IntakeDeploy",
            signal_profile=FROGTalonFX.SignalProfile.POSITION_MM,
        )
        self._min_speed = constants.Intake.INTAKE_MIN_SPEED
        self._speed_multiplier = constants.Intake.INTAKE_SPEED_MULTIPLIER

        if wpilib.RobotBase.isSimulation():
            # Roller simulation
            self.motor.simulation_init(moi=0.001)

            # Deploy motor simulation
            self.deploy_motor.simulation_init(moi=0.01)

    def _compute_target_speed(self) -> float:
        """Compute intake target speed (m/s) based on robot linear speed."""
        robot_speed = abs(self._robot_speed_supplier())
        return max(self._min_speed, robot_speed * self._speed_multiplier)

    def _run_intake_motor_forward(self):
        target = self._compute_target_speed()
        self.motor.set_control(controls.VelocityVoltage(target, slot=0))

    def _stop_intake_motor(self):
        self.motor.stopMotor()

    def _run_deploy_to_target(self):
        self.deploy_motor.set_control(controls.MotionMagicVoltage(constants.Intake.INTAKE_DEPLOY_TARGET_METERS, slot=0, enable_foc=False))

    def _run_deploy_to_stowed(self):
        self.deploy_motor.set_control(controls.MotionMagicVoltage(0.0, slot=0, enable_foc=False))

    # ────────────────────────────────────────────────
    #          Command Factory Methods
    # ────────────────────────────────────────────────

    def run_forward_cmd(self) -> Command:
        """Deploy the intake, wait for it to extend, then run the intake roller forward until interrupted. Retracts when finished."""
        return (
            self.deploy_cmd()
            .andThen(
                # Wait until the deploy motor reaches close to the target position
                commands2.cmd.waitUntil(
                    lambda: abs(self.deploy_motor.get_position().value - constants.Intake.INTAKE_DEPLOY_TARGET_METERS) < 0.1
                )
            )
            .andThen(self.run(self._run_intake_motor_forward))
            .finallyDo(lambda interrupted: (self._stop_intake_motor(), self._run_deploy_to_stowed()))
            .withName("Intake Forward")
        )

    def stop_cmd(self) -> Command:
        """Stop the intake motor immediately (one-shot runOnce command)."""
        return self.runOnce(self._stop_intake_motor).withName("Intake Stop")

    def deploy_cmd(self) -> Command:
        """Deploy the intake to the target position using MotionMagic."""
        return self.runOnce(self._run_deploy_to_target).withName("Intake Deploy")

    def retract_cmd(self) -> Command:
        """Retract the intake to the stowed position using MotionMagic."""
        return self.runOnce(self._run_deploy_to_stowed).withName("Intake Retract")

    # Alternative style using run() + explicit stop condition (if you prefer)
    # This version keeps running the execute lambda every loop until interrupted
    def run_forward_continuous_cmd(self) -> Command:
        """Run the intake roller forward continuously, calling stop in a finallyDo handler."""
        return (
            self.run(self._run_intake_motor_forward)
            .finallyDo(lambda interrupted: self._stop_intake_motor())
            .withName("Intake Forward Continuous")
        )

    # Very simple one-shot version (runs once then ends immediately)
    # Useful if you just want a quick "pulse"
    def pulse_forward_cmd(self) -> Command:
        """Pulse the intake roller forward for 150 ms, then stop."""
        return (
            self.runOnce(self._run_intake_motor_forward)
            .withTimeout(0.15)
            .andThen(self.runOnce(self._stop_intake_motor))
            .withName("Intake Pulse Forward")
        )

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(dt, battery_v)
        self.deploy_motor.simulation_update(dt, battery_v)

    @FROGSubsystem.telemetry("Voltage")
    def voltage_telem(self) -> float:
        return self.motor.get_motor_voltage().value

    @FROGSubsystem.telemetry("Velocity")
    def velocity_telem(self) -> float:
        return self.motor.get_velocity().value

    @FROGSubsystem.telemetry("Deploy Position")
    def deploy_position_telem(self) -> float:
        return self.deploy_motor.get_position().value

    @FROGSubsystem.tunable(constants.Intake.INTAKE_MIN_SPEED, "Min Speed")
    def min_speed_tunable(self, val):
        self._min_speed = val

    @FROGSubsystem.tunable(constants.Intake.INTAKE_SPEED_MULTIPLIER, "Speed Multiplier")
    def speed_multiplier_tunable(self, val):
        self._speed_multiplier = val
