from phoenix6.hardware import TalonFX
from commands2 import Command
from FROGlib.ctre import (
    FROGTalonFX,
    get_frog_talon_config,
    MOTOR_OUTPUT_CWP_COAST,
    MOTOR_OUTPUT_CCWP_COAST,
)
from phoenix6.configs import (
    TalonFXConfiguration,
    Slot0Configs,
    FeedbackConfigs,
    MotorOutputConfigs,
)
import constants
from phoenix6 import controls
from FROGlib.subsystem import FROGSubsystem
import wpilib


hopper_slot0 = Slot0Configs().with_k_s(constants.kVoltageHopperS)

hopper_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CCWP_COAST)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(1.0))
    .with_slot0(hopper_slot0)
)


class Hopper(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            id=constants.kHopperMotorID,
            motor_config=hopper_motor_config,
            canbus="rio",
            motor_name="Hopper",
        )
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

    def run_forward_cmd(self) -> Command:
        """Run the hopper motor forward at default voltage until interrupted."""
        return self.startEnd(
            self._run_hopper_motor_forward, self._stop_hopper_motor
        ).withName("Hopper Forward")

    def run_backward_cmd(self) -> Command:
        """Run the hopper motor backward at default voltage until interrupted."""
        return self.startEnd(
            self._run_hopper_motor_backward, self._stop_hopper_motor
        ).withName("Hopper Backward")

    # Alternative style using run() + explicit stop condition (if you prefer)
    # This version keeps running the execute lambda every loop until interrupted
    def run_forward_continuous_cmd(self) -> Command:
        """Run the hopper motor forward continuously, calling stop in a finallyDo handler."""
        return (
            self.run(self._run_hopper_motor_forward)
            .finallyDo(lambda interrupted: self._stop_hopper_motor())
            .withName("Hopper Forward Continuous")
        )

    # Very simple one-shot version (runs once then ends immediately)
    # Useful if you just want a quick "pulse"
    def pulse_forward_cmd(self) -> Command:
        """Pulse the hopper motor forward for 150 ms, then stop."""
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

    @FROGSubsystem.telemetry("Voltage")
    def voltage_telem(self) -> float:
        return self.motor.get_motor_voltage().value

    @FROGSubsystem.telemetry("Velocity")
    def velocity_telem(self) -> float:
        return self.motor.get_velocity().value

    @FROGSubsystem.telemetry("Not Stalled")
    def not_stalled_telem(self) -> bool:
        """Returns False if the motor is stalled."""
        return not self.motor.is_stalled()

    @FROGSubsystem.tunable(4.0, "Default Voltage")
    def default_voltage_tunable(self, val):
        self._default_voltage = val
