from copy import deepcopy
from commands2 import Subsystem
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
from subsystems.drive import Drive
from phoenix6.configs import SlotConfigs
from wpiutil import Sendable, SendableBuilder
import wpilib

flywheel_slot0 = FROGSlotConfig(
    k_s=constants.kFlywheelS,
    k_v=constants.kFlywheelV,
    k_a=constants.kFlywheelA,
    k_p=constants.kFlywheelP,
    k_i=constants.kFlywheelI,
    k_d=constants.kFlywheelD,
)
feed_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageHopperS,
)

flywheel_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Shooter",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=flywheel_slot0,
)

feed_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Shooter",
    motor_output=MOTOR_OUTPUT_CCWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=1.0),
    slot0=feed_slot0,
)


class FlywheelTunables(Sendable):
    """Sendable exposing only flywheel slot0 gains for dashboard tuning."""

    def __init__(self, shooter: "Shooter"):
        self.shooter = shooter

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("Flywheel Tunables")

        def get_param(param: str) -> float:
            # Safe access to the configured slot0
            return getattr(self.shooter.right_motor.config.slot0, param)

        for param in ["k_s", "k_v", "k_a", "k_p", "k_i", "k_d"]:
            # Clean names since it's a dedicated tuning widget
            builder.addDoubleProperty(
                param.upper(),
                lambda p=param: get_param(p),
                lambda v, p=param: self.shooter._updateFlywheelSlot0(**{p: v}),
            )


class Shooter(Subsystem):
    def __init__(self, drive: Drive):
        self.right_motor = FROGTalonFX(
            motor_config=deepcopy(flywheel_motor_config)
            .with_id(constants.kShooterRightFlywheelID)
            .with_motor_name("RightFlywheel")
        )
        self.left_motor = FROGTalonFX(
            motor_config=deepcopy(flywheel_motor_config)
            .with_id(constants.kShooterLeftFlywheelID)
            .with_motor_name("LeftFlywheel")
            .with_motor_output(MOTOR_OUTPUT_CWP_COAST)
        )
        self.left_motor.set_control(controls.Follower(self.right_motor.device_id, True))

        self.drive = drive
        self.feed_motor = FROGTalonFX(
            motor_config=FROGTalonFXConfig(feed_motor_config)
            .with_id(constants.kFeedMotorID)
            .with_motor_name("FeedMotor")
        )

        self.tunables = FlywheelTunables(self)

        if wpilib.RobotBase.isSimulation():
            self._flywheel_right_sim_velocity = 0.0
            self._flywheel_left_sim_velocity = 0.0
            self._feed_sim_velocity = 0.0

    def _set_speed(self, speed: float):
        self.right_motor.set_control(controls.VelocityVoltage(speed))

    def _stop_motor(self):
        self.right_motor.stopMotor()

    # boolean to indicate if flywheel is at target speed
    def _flywheel_at_speed(self, target_speed: float, tolerance: float = 50.0) -> bool:
        current_rps = self.right_motor.get_velocity().value
        return abs(current_rps - target_speed) <= tolerance

    # method to run feed motor forward
    def _run_feed_motor_forward(self):
        self.feed_motor.set_control(
            controls.VoltageOut(constants.kVoltageHopperS, enable_foc=False)
        )

    # method to stop feed motor
    def _stop_feed_motor(self):
        self.feed_motor.stopMotor()

    # generate a command to run the flywheel at a target speed
    def run_flywheel_at_speed(self, target_speed: float):
        return self.startEnd(
            lambda: self._set_speed(target_speed),
            self._stop_motor,
        )

    def simulationPeriodic(self):

        dt = 0.020
        max_rps = 100.0
        battery_v = wpilib.RobotController.getBatteryVoltage()

        for motor, sim_vel_attr in [
            (self.right_motor, "_flywheel_right_sim_velocity"),
            (self.left_motor, "_flywheel_left_sim_velocity"),
            (self.feed_motor, "_feed_sim_velocity"),
        ]:
            motor.sim_state.set_supply_voltage(battery_v)
            applied_v = motor.get_motor_voltage().value
            target_vel = (applied_v / 12.0) * max_rps
            sim_vel = getattr(self, sim_vel_attr)
            sim_vel += 0.3 * (target_vel - sim_vel)
            setattr(self, sim_vel_attr, sim_vel)
            directed_vel = sim_vel
            motor.sim_state.set_rotor_velocity(directed_vel)
            pos_change = directed_vel * dt
            motor.sim_state.add_rotor_position(pos_change)

    def _updateFlywheelSlot0(self, **kwargs):
        for motor in [self.right_motor, self.left_motor]:
            slot0 = SlotConfigs()
            motor.configurator.refresh(slot0)
            for k, v in kwargs.items():
                setattr(slot0, k, v)
            motor.configurator.apply(slot0)

    def _get_slot0_param(self, param: str) -> float:
        # Use .configs for a cached/snapshot view (safer in Sendable callbacks)
        return getattr(self.right_motor.config.slot0, param)

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)
        builder.setSmartDashboardType("Shooter")  # or "Flywheel Tunables" etc.

        # Read-only telemetry (shown in main widget)
        builder.addDoubleProperty(
            "Flywheel RPS",
            lambda: self.right_motor.get_velocity().value_as_double,
            None,
        )
        builder.addBooleanProperty(
            "At Speed", lambda: self._flywheel_at_speed(self.target_speed or 0), None
        )  # if you track target

        # Tunable gains – these are editable
        for param in ["k_s", "k_v", "k_a", "k_p", "k_i", "k_d"]:
            builder.addDoubleProperty(
                f"Flywheel {param.upper()}",
                lambda p=param: self._get_slot0_param(p),
                lambda v, p=param: self._updateFlywheelSlot0(**{p: v}),
            )
