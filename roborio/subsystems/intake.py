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


intake_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageIntakeS,
)

intake_motor_config = FROGTalonFXConfig(
    id=constants.kIntakeMotorID,
    can_bus="rio",
    motor_name="Intake",
    parent_nt="Intake",
    motor_output=MOTOR_OUTPUT_CWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=intake_slot0,
)


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(motor_config=intake_motor_config)
        self._default_voltage = 4
        self._is_sim = wpilib.RobotBase.isSimulation()

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
        if self._is_sim:
            self.motor.sim_state.set_supply_voltage(
                wpilib.RobotController.getBatteryVoltage()
            )
            # Simple simulation of motor velocity based on applied voltage
            applied_voltage = self.motor.get_motor_voltage().value
            # Assume a simple linear model: velocity proportional to voltage
            max_velocity = 5000  # RPM, arbitrary max for simulation
            velocity = (
                applied_voltage / 12.0
            ) * max_velocity  # Assuming 12V full speed
            self.motor.sim_state.set_rotor_velocity(velocity)
            self.motor.sim_state.add_rotor_position(
                velocity * 0.02 / 60.0
            )  # Convert RPM to rotations per 20ms
