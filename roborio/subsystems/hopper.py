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
from commands2 import Subsystem
import wpilib
from wpiutil import SendableBuilder


hopper_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageHopperS,
)

hopper_motor_config = FROGTalonFXConfig(
    id=constants.kHopperMotorID,
    can_bus="rio",
    motor_name="Hopper",
    parent_nt="Hopper",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=hopper_slot0,
)


class Hopper(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(motor_config=hopper_motor_config)
        self._default_voltage = 4

        if wpilib.RobotBase.isSimulation():
            self._sim_velocity = 0.0
            inverted = bool(self.motor.config.motor_output.inverted.value)
            self._velocity_sign_multiplier = -1 if inverted else 1

    def _run_hopper_motor_forward(self):
        self.motor.set_control(
            controls.VoltageOut(self._default_voltage, enable_foc=False)
        )

    def _run_hopper_motor_backward(self):
        self.motor.set_control(
            controls.VoltageOut(-self._default_voltage, enable_foc=False)
        )

    def _stop_hopper_motor(self):
        self.motor.stopMotor()

    # Optional: helper to check if motor is actively driven
    def _is_running(self) -> bool:
        return abs(self.motor.get_motor_voltage().value) > 0

    # ────────────────────────────────────────────────
    #          Command Factory Methods
    # ────────────────────────────────────────────────

    def runForward(self):
        """Runs the hopper forward at default voltage until interrupted."""
        return self.startEnd(
            self._run_hopper_motor_forward, self._stop_hopper_motor
        ).withName("Hopper Forward")

    def runBackward(self):
        """Runs the hopper backward (eject/reverse) at default voltage until interrupted."""
        return self.startEnd(
            self._run_hopper_motor_backward, self._stop_hopper_motor
        ).withName("Hopper Backward")

    # Alternative style using run() + explicit stop condition (if you prefer)
    # This version keeps running the execute lambda every loop until interrupted
    def runForwardContinuous(self):
        return (
            self.run(self._run_hopper_motor_forward)
            .finallyDo(lambda interrupted: self._stop_hopper_motor())
            .withName("Hopper Forward Continuous")
        )

    # Very simple one-shot version (runs once then ends immediately)
    # Useful if you just want a quick "pulse"
    def pulseForward(self):
        return (
            self.runOnce(self._run_hopper_motor_forward)
            .withTimeout(0.15)
            .andThen(self.runOnce(self._stop_hopper_motor))
            .withName("Hopper Pulse Forward")
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

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)
        builder.setSmartDashboardType("Hopper")
        builder.addDoubleProperty(
            "Voltage", lambda: self.motor.get_motor_voltage().value, lambda _: None
        )
        builder.addDoubleProperty(
            "Velocity", lambda: self.motor.get_velocity().value, lambda _: None
        )
        builder.addDoubleProperty(
            "tunables/Default Voltage",
            lambda: self._default_voltage,
            lambda value: setattr(self, "_default_voltage", value),
        )
