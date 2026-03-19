import math
from typing import Callable
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
from FROGlib.subsystem import FROGSubsystem, Direction
import wpilib

# Falcon 500 direct-drives a 1-inch diameter roller (no intermediate gearing).
# 1 motor rotation = π × 0.0254 m of surface travel.
# sensor_to_mechanism_ratio converts motor RPS → mechanism units (m/s surface speed).
_ROLLER_DIAMETER_M = 0.0254  # 1 inch in meters
_SENSOR_TO_MECHANISM_RATIO = 1.0 / (math.pi * _ROLLER_DIAMETER_M)  # ≈ 12.5331

# Falcon 500 free-speed used for simple simulation model
_FALCON500_MAX_RPS = 6380.0 / 60.0  # ≈ 106.33 RPS

intake_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageIntakeS,
    k_v=constants.kIntakeV,
    k_p=constants.kIntakeP,
)

intake_motor_config = FROGTalonFXConfig(
    id=constants.kIntakeMotorID,
    can_bus="rio",
    motor_name="Intake",
    parent_nt="Intake",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=_SENSOR_TO_MECHANISM_RATIO),
    slot0=intake_slot0,
)


class Intake(FROGSubsystem):
    def __init__(self, robot_speed_supplier: Callable[[], float] = lambda: 0.0):
        super().__init__()
        self._robot_speed_supplier = robot_speed_supplier
        self.motor = FROGTalonFX(
            motor_config=intake_motor_config,
        )
        self._min_speed = constants.kIntakeMinSpeed
        self._speed_multiplier = constants.kIntakeSpeedMultiplier
        self._reverse_speed = constants.kIntakeReverseSpeed

        if wpilib.RobotBase.isSimulation():
            self._sim_velocity = 0.0
            # if motor output is inverted (i.e. = 1)
            inverted = bool(self.motor.config.motor_output.inverted.value)
            self._velocity_sign_multiplier = -1 if inverted else 1

    def _compute_target_speed(self) -> float:
        """Compute intake target speed (m/s) based on robot linear speed."""
        robot_speed = abs(self._robot_speed_supplier())
        return max(self._min_speed, robot_speed * self._speed_multiplier)

    def _run_intake_motor_forward(self):
        target = self._compute_target_speed()
        self.motor.set_control(controls.VelocityVoltage(target, slot=0))

    def _run_intake_motor_backward(self):
        self.motor.set_control(
            controls.VelocityVoltage(-self._reverse_speed, slot=0)
        )

    def _stop_intake_motor(self):
        self.motor.stopMotor()

    # Optional: helper to check if motor is actively driven
    def get_direction(self) -> Direction:
        velocity = self.motor.get_velocity().value
        if velocity > 0.1:
            return Direction.FORWARD
        elif velocity < -0.1:
            return Direction.REVERSE
        else:
            return Direction.IDLE

    # ────────────────────────────────────────────────
    #          Command Factory Methods
    # ────────────────────────────────────────────────

    def run_forward_cmd(self):
        """Runs the intake forward, tracking robot speed with a minimum floor."""
        return self.startEnd(
            self._run_intake_motor_forward, self._stop_intake_motor
        ).withName("Intake Forward")

    def run_backward_cmd(self):
        """Runs the intake backward (eject/reverse) at fixed speed until interrupted."""
        return self.startEnd(
            self._run_intake_motor_backward, self._stop_intake_motor
        ).withName("Intake Backward")

    def stop_cmd(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        return self.runOnce(self._stop_intake_motor)

    # Alternative style using run() + explicit stop condition (if you prefer)
    # This version keeps running the execute lambda every loop until interrupted
    def run_forward_continuous_cmd(self):
        return (
            self.run(self._run_intake_motor_forward)
            .finallyDo(lambda interrupted: self._stop_intake_motor())
            .withName("Intake Forward Continuous")
        )

    # Very simple one-shot version (runs once then ends immediately)
    # Useful if you just want a quick "pulse"
    def pulse_forward_cmd(self):
        return (
            self.runOnce(self._run_intake_motor_forward)
            .withTimeout(0.15)
            .andThen(self.runOnce(self._stop_intake_motor))
            .withName("Intake Pulse Forward")
        )

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(
            dt,
            battery_v,
            max_velocity_rps=_FALCON500_MAX_RPS,
            velocity_sign_multiplier=self._velocity_sign_multiplier,
        )

    @FROGSubsystem.telemetry("Voltage")
    def voltage_telem(self) -> float:
        return self.motor.get_motor_voltage().value

    @FROGSubsystem.telemetry("Velocity")
    def velocity_telem(self) -> float:
        return self.motor.get_velocity().value

    @FROGSubsystem.tunable(constants.kIntakeMinSpeed, "Min Speed")
    def min_speed_tunable(self, val):
        self._min_speed = val

    @FROGSubsystem.tunable(constants.kIntakeSpeedMultiplier, "Speed Multiplier")
    def speed_multiplier_tunable(self, val):
        self._speed_multiplier = val

