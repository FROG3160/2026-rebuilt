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
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=intake_slot0,
)


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(motor_config=intake_motor_config)
        self._default_voltage = 4
        if wpilib.RobotBase.isSimulation():
            self._sim_velocity = 0.0
            # if motor output is inverted (i.e. = 1)
            self._velocity_sign_multiplier = [1, -1][
                self.motor.config.motor_output.inverted.value
            ]

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
        self.motor.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        # Simple simulation of motor velocity based on applied voltage
        applied_voltage = self.motor.get_motor_voltage().value
        # Assume a simple linear model: velocity proportional to voltage
        max_velocity_rot_per_sec = 83.33  # ← CHANGE THIS TO MATCH YOUR MECHANISM
        target_velocity = (
            applied_voltage / 12.0
        ) * max_velocity_rot_per_sec  # Assuming 12V full speed

        # Apply smoothing (simulates inertia/friction ramp)
        self._sim_velocity += 0.3 * (target_velocity - self._sim_velocity)

        # IMPORTANT: Apply direction sign from motor config
        directed_velocity = 1 * self._sim_velocity

        self.motor.sim_state.set_rotor_velocity(directed_velocity)

        # Integrate to update position (dt ≈ 0.020 s in TimedRobot)
        dt = 0.020
        position_change = directed_velocity * dt
        self.motor.sim_state.add_rotor_position(position_change)
