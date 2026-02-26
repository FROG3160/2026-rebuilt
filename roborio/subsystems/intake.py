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
from FROGlib.subsystem import FROGSubsystem
import wpilib


intake_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageIntakeS,
)

intake_motor_config = FROGTalonFXConfig(
    id=constants.kIntakeMotorID,
    can_bus="rio",
    motor_name="Intake",
    parent_nt="Intake",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=intake_slot0,
)


class Intake(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(motor_config=intake_motor_config)
        self._default_voltage = 4

        if wpilib.RobotBase.isSimulation():
            self._sim_velocity = 0.0
            # if motor output is inverted (i.e. = 1)
            inverted = bool(self.motor.config.motor_output.inverted.value)
            self._velocity_sign_multiplier = -1 if inverted else 1

    def _run_intake_motor_forward(self):
        self.motor.set_control(
            controls.VoltageOut(self._default_voltage, enable_foc=False)
        )

    def _run_intake_motor_backward(self):
        self.motor.set_control(
            controls.VoltageOut(-self._default_voltage, enable_foc=False)
        )

    def _stop_intake_motor(self):
        self.motor.stopMotor()

    # Optional: helper to check if motor is actively driven
    def _is_running(self) -> bool:
        return abs(self.motor.get_motor_voltage().value) > 0

    # ────────────────────────────────────────────────
    #          Command Factory Methods
    # ────────────────────────────────────────────────

    def runForward(self):
        """Runs the intake forward at default voltage until interrupted."""
        return self.startEnd(
            self._run_intake_motor_forward, self._stop_intake_motor
        ).withName("Intake Forward")

    def runBackward(self):
        """Runs the intake backward (eject/reverse) at default voltage until interrupted."""
        return self.startEnd(
            self._run_intake_motor_backward, self._stop_intake_motor
        ).withName("Intake Backward")

    # Alternative style using run() + explicit stop condition (if you prefer)
    # This version keeps running the execute lambda every loop until interrupted
    def runForwardContinuous(self):
        return (
            self.run(self._run_intake_motor_forward)
            .finallyDo(lambda interrupted: self._stop_intake_motor())
            .withName("Intake Forward Continuous")
        )

    # Very simple one-shot version (runs once then ends immediately)
    # Useful if you just want a quick "pulse"
    def pulseForward(self):
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
            max_velocity_rps=83.33,
            velocity_sign_multiplier=self._velocity_sign_multiplier,
        )

    @FROGSubsystem.telemetry("Voltage")
    def voltage_telem(self) -> float:
        return self.motor.get_motor_voltage().value

    @FROGSubsystem.telemetry("Velocity")
    def velocity_telem(self) -> float:
        return self.motor.get_velocity().value

    @FROGSubsystem.tunable(4.0, "Default Voltage")
    def default_voltage_tunable(self, val):
        self._default_voltage = val

