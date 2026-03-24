import wpilib
from phoenix6.hardware import TalonFX, CANrange
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
    CANrangeConfiguration,
    ProximityParamsConfigs,
    MotionMagicConfigs,
)
import constants
from phoenix6 import controls
from FROGlib.subsystem import FROGSubsystem
from wpimath.system.plant import DCMotor, LinearSystemId
from wpilib import Timer, SmartDashboard

hopper_slot0 = (
    Slot0Configs()
    .with_k_s(constants.kVoltageHopperS)
    .with_k_v(constants.kHopperV)
    .with_k_p(constants.kHopperP)
    .with_k_i(constants.kHopperI)
    .with_k_d(constants.kHopperD)
)

hopper_mm = (
    MotionMagicConfigs()
    .with_motion_magic_cruise_velocity(constants.kHopperMM_V)
    .with_motion_magic_acceleration(constants.kHopperMM_A)
)

hopper_left_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CCWP_COAST)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(1.0))
    .with_slot0(hopper_slot0)
    .with_motion_magic(hopper_mm)
)

hopper_right_motor_config = (
    get_frog_talon_config()
    .with_motor_output(MOTOR_OUTPUT_CWP_COAST)
    .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(1.0))
    .with_slot0(hopper_slot0)
    .with_motion_magic(hopper_mm)
)

# CANrange detection threshold: 0.75 inches = 0.01905 meters
CANRANGE_THRESHOLD_METERS = 0.01905

canrange_config = (
    CANrangeConfiguration()
    .with_proximity_params(
        ProximityParamsConfigs()
        .with_proximity_threshold(CANRANGE_THRESHOLD_METERS)
        .with_proximity_hysteresis(0.005) # Add a small hysteresis
    )
)

class Spindexer:
    def __init__(self, name: str, motor_id: int, motor_config: TalonFXConfiguration, sensor_id: int, forward_velocity: float, reverse_velocity: float):
        self.name = name
        self.motor = FROGTalonFX(
            id=motor_id,
            motor_config=motor_config,
            canbus="rio",
            motor_name=f"Hopper_{name}",
        )
        self.sensor = CANrange(sensor_id, "rio")
        self.sensor.configurator.apply(canrange_config)

        self.forward_velocity = forward_velocity
        self.reverse_velocity = reverse_velocity
        
        self.running_timer = Timer()
        self.running_timer.start()
        
        self.burst_timer = Timer()
        self.burst_timer.start()
        self.is_bursting = False
        
        self.staged_idle_timer = Timer()
        self.staged_idle_timer.start()
        
        self.pulse_timer = Timer()
        self.pulse_timer.start()
        self.is_pulsing = False
        
        # Configuration
        self.burst_timeout = 1.0  # seconds running without fuel before bursting
        self.burst_duration = 0.5 # seconds to run in reverse
        self.pulse_interval = 1.0 # seconds to wait when staged before pulsing
        self.pulse_duration = 0.2 # seconds to run forward when pulsing
        
        self.mock_detected = False
        if wpilib.RobotBase.isSimulation():
            SmartDashboard.putBoolean(f"Hopper/{self.name}_Mock_Detected", self.mock_detected)
            self.motor.simulation_init(moi=0.001, motor_model=DCMotor.krakenX44(1))

    def execute_serialize(self):
        """Run the serialization state machine for this side."""
        # Check if fuel is detected using the configured threshold
        if wpilib.RobotBase.isSimulation():
            detected = self.mock_detected
        else:
            detected = self.sensor.get_is_detected().value
        
        if detected:
            # Reset empty running timers
            self.running_timer.reset()
            self.is_bursting = False
            
            if self.is_pulsing:
                self.motor.set_control(controls.MotionMagicVelocityVoltage(self.forward_velocity, slot=0, enable_foc=False))
                if self.pulse_timer.hasElapsed(self.pulse_duration):
                    self.is_pulsing = False
                    self.staged_idle_timer.reset()
            else:
                self.motor.set_control(controls.NeutralOut())
                if self.staged_idle_timer.hasElapsed(self.pulse_interval):
                    self.is_pulsing = True
                    self.pulse_timer.reset()
        else:
            # Reset staged timers
            self.staged_idle_timer.reset()
            self.is_pulsing = False
            
            if self.is_bursting:
                self.motor.set_control(controls.MotionMagicVelocityVoltage(self.reverse_velocity, slot=0, enable_foc=False))
                if self.burst_timer.hasElapsed(self.burst_duration):
                    self.is_bursting = False
                    self.running_timer.reset()
            else:
                self.motor.set_control(controls.MotionMagicVelocityVoltage(self.forward_velocity, slot=0, enable_foc=False))
                if self.running_timer.hasElapsed(self.burst_timeout):
                    self.is_bursting = True
                    self.burst_timer.reset()

    def run_forward(self, velocity=None):
        self.motor.set_control(controls.MotionMagicVelocityVoltage(velocity or self.forward_velocity, slot=0, enable_foc=False))
        self.running_timer.reset()
        self.staged_idle_timer.reset()
        
    def run_backward(self, velocity=None):
        self.motor.set_control(controls.MotionMagicVelocityVoltage(velocity or self.reverse_velocity, slot=0, enable_foc=False))
        self.running_timer.reset()
        self.staged_idle_timer.reset()

    def stop_motor(self):
        """Stop the motor but maintain state variables."""
        self.motor.stopMotor()
        
    def stop_all_and_reset(self):
        """Stop the motor and reset all state machine variables."""
        self.motor.stopMotor()
        self.is_bursting = False
        self.is_pulsing = False
        self.burst_timer.reset()
        self.running_timer.reset()
        self.pulse_timer.reset()
        self.staged_idle_timer.reset()

    def simulationPeriodic(self):
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()
        self.motor.simulation_update(
            dt,
            battery_v,
        )
        # Mocking CANrange
        self.mock_detected = SmartDashboard.getBoolean(f"Hopper/{self.name}_Mock_Detected", self.mock_detected)
        if self.mock_detected:
            self.sensor.sim_state.set_distance(0.01)
        else:
            self.sensor.sim_state.set_distance(0.1)


class Hopper(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self._default_velocity = 20.0
        self._burst_timeout = 1.0
        self._burst_duration = 0.5
        self._pulse_interval = 1.0
        self._pulse_duration = 0.2
        
        self.left_side = Spindexer(
            "Left", 
            constants.kHopperLeftMotorID, 
            hopper_left_motor_config, 
            constants.kHopperLeftSensorID, 
            self._default_velocity, 
            -self._default_velocity
        )
        self.right_side = Spindexer(
            "Right", 
            constants.kHopperRightMotorID, 
            hopper_right_motor_config, 
            constants.kHopperRightSensorID, 
            self._default_velocity, 
            -self._default_velocity
        )

    def _update_side_configs(self):
        for side in (self.left_side, self.right_side):
            side.forward_velocity = self._default_velocity
            side.reverse_velocity = -self._default_velocity
            side.burst_timeout = self._burst_timeout
            side.burst_duration = self._burst_duration
            side.pulse_interval = self._pulse_interval
            side.pulse_duration = self._pulse_duration

    # ────────────────────────────────────────────────
    #          Command Factory Methods
    # ────────────────────────────────────────────────

    def serialize_cmd(self) -> Command:
        """Run the dual spindexer to serialize fuel independently on both sides until interrupted."""
        return (
            self.run(
                lambda: (self.left_side.execute_serialize(), self.right_side.execute_serialize())
            )
            .finallyDo(lambda interrupted: self.stop_all())
            .withName("Hopper Serialize")
        )

    def run_forward_cmd(self) -> Command:
        """Run both hopper motors forward manually until interrupted."""
        return self.startEnd(
            lambda: (self.left_side.run_forward(), self.right_side.run_forward()), 
            self.stop_all
        ).withName("Hopper Forward")

    def run_backward_cmd(self) -> Command:
        """Run both hopper motors backward manually until interrupted."""
        return self.startEnd(
            lambda: (self.left_side.run_backward(), self.right_side.run_backward()), 
            self.stop_all
        ).withName("Hopper Backward")

    def stop_cmd(self) -> Command:
        """Stop the hopper motors immediately (one-shot runOnce command)."""
        return self.runOnce(self.stop_all).withName("Hopper Stop")

    def stop_all(self):
        self.left_side.stop_all_and_reset()
        self.right_side.stop_all_and_reset()

    def simulationPeriodic(self):
        self.left_side.simulationPeriodic()
        self.right_side.simulationPeriodic()

    @FROGSubsystem.telemetry("Left Voltage")
    def left_voltage_telem(self) -> float:
        return self.left_side.motor.get_motor_voltage().value
        
    @FROGSubsystem.telemetry("Right Voltage")
    def right_voltage_telem(self) -> float:
        return self.right_side.motor.get_motor_voltage().value
        
    @FROGSubsystem.telemetry("Left Distance (m)")
    def left_distance_telem(self) -> float:
        return self.left_side.sensor.get_distance().value
        
    @FROGSubsystem.telemetry("Right Distance (m)")
    def right_distance_telem(self) -> float:
        return self.right_side.sensor.get_distance().value

    @FROGSubsystem.tunable(20.0, "Default Velocity (RPS)")
    def default_velocity_tunable(self, val):
        self._default_velocity = val
        self._update_side_configs()
        
    @FROGSubsystem.tunable(1.0, "Burst Timeout (s)")
    def burst_timeout_tunable(self, val):
        self._burst_timeout = val
        self._update_side_configs()

    @FROGSubsystem.tunable(0.5, "Burst Duration (s)")
    def burst_duration_tunable(self, val):
        self._burst_duration = val
        self._update_side_configs()

    @FROGSubsystem.tunable(1.0, "Pulse Interval (s)")
    def pulse_interval_tunable(self, val):
        self._pulse_interval = val
        self._update_side_configs()

    @FROGSubsystem.tunable(0.2, "Pulse Duration (s)")
    def pulse_duration_tunable(self, val):
        self._pulse_duration = val
        self._update_side_configs()

    @FROGSubsystem.tunable(20.0, "Test Speed")
    def test_speed_tunable(self, val):
        self._test_speed = val

    def run_test_cmd(self) -> Command:
        """Run both hopper motors manually at Test Speed until interrupted."""
        return self.startEnd(
            lambda: (
                self.left_side.run_forward(getattr(self, "_test_speed", 20.0)), 
                self.right_side.run_forward(getattr(self, "_test_speed", 20.0))
            ), 
            self.stop_all
        ).withName("Hopper Test")
