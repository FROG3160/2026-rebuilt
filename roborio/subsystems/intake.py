import math
from typing import Callable
import commands2
from commands2 import Command, cmd
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

roller_slot0 = (
    Slot0Configs()
    .with_k_s(constants.Intake.VOLTAGE_ROLLER_S)
    .with_k_v(constants.Intake.ROLLER_V)
    .with_k_p(constants.Intake.ROLLER_P)
)

roller_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CCWP_COAST)
    .with_feedback(
        FeedbackConfigs().with_sensor_to_mechanism_ratio(_SENSOR_TO_MECHANISM_RATIO)
    )
    .with_slot0(roller_slot0)
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
    .with_feedback(
        FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1.0 / constants.Intake.INTAKE_DEPLOY_DISTANCE_PER_ROTATION
        )
    )
    .with_slot0(intake_deploy_slot0)
    .with_motion_magic(intake_deploy_mm)
    .with_current_limits(intake_deploy_current_limits)
)


class Intake(FROGSubsystem):
    def __init__(self, robot_speed_supplier: Callable[[], float] = lambda: 0.0):
        super().__init__()
        self._robot_speed_supplier = robot_speed_supplier
        self.roller_motor = FROGTalonFX(
            id=constants.CANIDs.ROLLER_MOTOR,
            motor_config=roller_motor_config,
            canbus="rio",
            motor_name="Roller",
        )
        self.deploy_motor = FROGTalonFX(
            id=constants.CANIDs.DEPLOY_MOTOR,
            motor_config=intake_deploy_motor_config,
            canbus="rio",
            motor_name="IntakeDeploy",
            signal_profile=FROGTalonFX.SignalProfile.POSITION_MM,
        )
        self._min_speed = constants.Intake.ROLLER_MIN_SPEED
        self._speed_multiplier = constants.Intake.ROLLER_SPEED_MULTIPLIER
        self._cycle_deploying = True

        if wpilib.RobotBase.isSimulation():
            # Roller simulation
            self.roller_motor.simulation_init(moi=0.001)

            # Deploy motor simulation
            self.deploy_motor.simulation_init(moi=0.01)

    def _compute_target_speed(self) -> float:
        """Compute roller target speed (m/s) based on robot linear speed."""
        robot_speed = abs(self._robot_speed_supplier())
        return max(self._min_speed, robot_speed * self._speed_multiplier)

    def _run_roller_motor_forward(self):
        target = self._compute_target_speed()
        self.roller_motor.set_control(
            controls.VelocityVoltage(target, slot=0, enable_foc=False)
        )

    def _stop_roller_motor(self):
        self.roller_motor.stopMotor()

    def _run_deploy_to_target(self):
        self.deploy_motor.set_control(
            controls.MotionMagicVoltage(
                constants.Intake.INTAKE_DEPLOY_TARGET_METERS, slot=0, enable_foc=False
            )
        )

    def _run_deploy_to_cycle(self):
        self.deploy_motor.set_control(
            controls.MotionMagicVoltage(
                constants.Intake.INTAKE_CYCLE_TARGET_METERS, slot=0, enable_foc=False
            )
        )

    def _run_deploy_to_stowed(self):
        self.deploy_motor.set_control(
            controls.MotionMagicVoltage(0.0, slot=0, enable_foc=False)
        )

    def _run_cycle(self):
        self._run_roller_motor_forward()

        pos = self.deploy_motor.get_position().value
        # Check if we need to flip direction
        if self._cycle_deploying:
            self._run_deploy_to_cycle()
            if abs(pos - constants.Intake.INTAKE_CYCLE_TARGET_METERS) < 0.02:
                self._cycle_deploying = False
        else:
            self._run_deploy_to_stowed()
            if abs(pos) < 0.02:
                self._cycle_deploying = True

    def _set_deploy_position(self):
        self.deploy_motor.set_position(0.0)

    # ────────────────────────────────────────────────
    #          Command Factory Methods
    # ────────────────────────────────────────────────

    def cycle_cmd(self) -> Command:
        """Cycle the intake deploy in and out continuously while running the roller motor."""
        return (
            self.run(self._run_cycle)
            .beforeStarting(lambda: setattr(self, "_cycle_deploying", True))
            .withName("Intake Cycle")
        )

    def run_and_deploy_cmd(self) -> Command:
        """Deploy the intake and immediately run the roller motor forward."""
        return (
            self.runOnce(self._run_deploy_to_target)
            .andThen(self.run(self._run_roller_motor_forward))
            .withName("Intake Run and Deploy")
        )

    def retract_and_stop_cmd(self) -> Command:
        """Retract the intake, waiting for it to reach the stowed position before stopping the roller motor."""
        return (
            self.runOnce(self._run_deploy_to_stowed)
            .andThen(
                self.run(self._run_roller_motor_forward).until(
                    lambda: abs(self.deploy_motor.get_position().value) < 0.1
                )
            )
            .andThen(self.runOnce(self._stop_roller_motor))
            .withName("Intake Retract And Stop")
        )

    def stop_cmd(self) -> Command:
        """Stop the roller motor immediately (one-shot runOnce command)."""
        return self.runOnce(self._stop_roller_motor).withName("Roller Stop")

    def deploy_cmd(self) -> Command:
        """Deploy the intake to the target position using MotionMagic."""
        return self.runOnce(self._run_deploy_to_target).withName("Intake Deploy")

    def retract_cmd(self) -> Command:
        """Retract the intake to the stowed position using MotionMagic."""
        return self.runOnce(self._run_deploy_to_stowed).withName("Intake Retract")

    def zero_intake_deploy_cmd(self) -> Command:
        """Drive the hood slowly into its reverse hard stop, zero the position sensor, then stop."""
        # return self.runOnce(self._set_hood_position())

        return (
            self.runOnce(
                lambda: self.deploy_motor.set_control(
                    controls.VoltageOut(constants.Intake.HOMING_VOLTAGE)
                )
            )
            .andThen(
                cmd.waitUntil(lambda: self.hood_motor.get_stator_current().value > constants.Shooter.HOOD_HOMING_CURRENT)  # type: ignore
            )
            .andThen(self.runOnce(self.deploy_motor.stopMotor))
            .andThen(self.runOnce(self._set_deploy_position))
        )

    # Alternative style using run() + explicit stop condition (if you prefer)
    # This version keeps running the execute lambda every loop until interrupted
    def run_forward_continuous_cmd(self) -> Command:
        """Run the roller forward continuously, calling stop in a finallyDo handler."""
        return (
            self.run(self._run_roller_motor_forward)
            .finallyDo(lambda interrupted: self._stop_roller_motor())
            .withName("Roller Forward Continuous")
        )

    # Very simple one-shot version (runs once then ends immediately)
    # Useful if you just want a quick "pulse"
    def pulse_forward_cmd(self) -> Command:
        """Pulse the roller forward for 150 ms, then stop."""
        return (
            self.runOnce(self._run_roller_motor_forward)
            .withTimeout(0.15)
            .andThen(self.runOnce(self._stop_roller_motor))
            .withName("Roller Pulse Forward")
        )

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.roller_motor.simulation_update(dt, battery_v)
        self.deploy_motor.simulation_update(dt, battery_v)

    @FROGSubsystem.telemetry("Voltage")
    def voltage_telem(self) -> float:
        return self.roller_motor.get_motor_voltage().value

    @FROGSubsystem.telemetry("Velocity")
    def velocity_telem(self) -> float:
        return self.roller_motor.get_velocity().value

    @FROGSubsystem.telemetry("Deploy Position")
    def deploy_position_telem(self) -> float:
        return self.deploy_motor.get_position().value

    @FROGSubsystem.tunable(constants.Intake.ROLLER_MIN_SPEED, "Min Speed")
    def min_speed_tunable(self, val):
        self._min_speed = val

    @FROGSubsystem.tunable(constants.Intake.ROLLER_SPEED_MULTIPLIER, "Speed Multiplier")
    def speed_multiplier_tunable(self, val):
        self._speed_multiplier = val
