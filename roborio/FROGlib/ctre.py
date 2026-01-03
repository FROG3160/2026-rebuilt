import math
from ntcore import NetworkTableInstance
from phoenix6 import StatusSignal
from phoenix6.configs.cancoder_configs import (
    CANcoderConfiguration,
    MagnetSensorConfigs,
    SensorDirectionValue,
)
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration, MotorOutputConfigs
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import Slot0Configs, Slot1Configs, FeedbackConfigs
from phoenix6.signals.spn_enums import (
    GravityTypeValue,
    InvertedValue,
    NeutralModeValue,
    SensorDirectionValue,
    StaticFeedforwardSignValue,
)
from wpimath.geometry import Rotation2d


# Motor output config for ClockWise Positive rotation and Brake neutral mode
MOTOR_OUTPUT_CWP_BRAKE = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.BRAKE)
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
)
# Motor output config for Counter-ClockWise Positive rotation and Brake neutral mode
MOTOR_OUTPUT_CCWP_BRAKE = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.BRAKE)
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
)
MOTOR_OUTPUT_CWP_COAST = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.COAST)
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
)
MOTOR_OUTPUT_CCWP_COAST = (
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.COAST)
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
)

# Magnet sensor config for Counter-ClockWise Positive rotation with continuous wrap
MAGNET_CONFIG_CONTWRAP_CCWP = (
    MagnetSensorConfigs()
    .with_absolute_sensor_discontinuity_point(0.5)
    .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
)


class FROGMotorOutputConfig(MotorOutputConfigs):
    """FROG custom MotorOutputConfig that takes parameters during instantiation."""

    def __init__(
        self,
        **kwargs,
    ):
        """
        Initialize motor output configuration for this motor controller.
        This constructor sets up configurations that affect the motor output behavior of
        the motor controller, such as neutral mode and inversion.
        Args:
            **kwargs: Arbitrary keyword arguments mapping configuration names to their
                values. Recognized keys (if present) include:
                - neutral_mode (NeutralModeValue): The neutral mode of the motor (e.g., BRAKE or COAST).
                - inverted (InvertedValue): The inversion state of the motor output.

        """
        super().__init__()
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)


class FROGSlotConfig(Slot0Configs, Slot1Configs):
    """FROG custom Slot0Configs that takes parameters during instantiation."""

    def __init__(self, **kwargs):
        """Gains for the specified slot.

        These gains are used in closed-loop control requests when this slot
        is selected.

        Args:
            **kwargs: Keyword arguments to override default gain values. Supported keys:
                k_p (float): Proportional gain. Defaults to 0.0.
                k_i (float): Integral gain.
                k_d (float): Derivative gain.
                k_s (float): Static feedforward gain.
                k_v (float): Velocity feedforward gain.
                k_a (float): Acceleration feedforward gain.
                k_g (float): Gravity feedforward gain.
                gravity_type (GravityTypeValue): Gravity compensation type.
                static_feedforward_sign (StaticFeedforwardSignValue): Sign for static feedforward.
        """
        # sets the default values for all attributes in Slot0Configs/Slot1Configs
        super().__init__()
        # only set attrubutes that already exist in the class
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)


class FROGFeedbackConfig(FeedbackConfigs):
    """FROG custom FeedbackConfig that takes parameters during instantiation."""

    def __init__(
        self,
        **kwargs,
    ):
        """
        Initialize feedback-related configuration for this motor controller.
        This constructor sets up configurations that affect the feedback behavior of
        the motor controller, such as the feedback sensor source, sensor/rotor offsets,
        and unit conversion ratios used for closed-loop control.
        Args:
            **kwargs: Arbitrary keyword arguments mapping configuration names to their
                values. Recognized keys (if present) include:
                - feedback_rotor_offset (float): Offset to apply to rotor feedback.
                - sensor_to_mechanism_ratio (float): Ratio to convert sensor units to
                    mechanism units.
                - rotor_to_sensor_ratio (float): Ratio to convert rotor units to sensor
                    units.
                - feedback_sensor_source (str|int): Identifier or type of the feedback
                    sensor source.
                - feedback_remote_sensor_id (int): ID of a remote feedback sensor, if
                    used.
                - velocity_filter_time_constant (float): Time constant used when
                    filtering velocity measurements.
        Notes:
            For each key provided in kwargs, if the instance has an attribute with the
            same name, the attribute will be set to the provided value. Unrecognized
            keys are ignored.
        """

        super().__init__()
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)


class FROGTalonFXConfig(TalonFXConfiguration):
    """FROG custom TalonFXConfiguration that takes parameters during instantiation."""

    def __init__(
        self,
        id=0,
        can_bus="rio",
        motor_name: str = "",
        parent_nt: str = "Undefined",
        **kwargs,
    ):
        """
        Initialize the CTRE configuration wrapper.
                Args:
                    **kwargs: Optional configuration overrides. Supported keyword keys are:
                        future_proof_configs (bool): Whether to enable future-proof configs. Defaults to True.
                        motor_output (MotorOutputConfigs): Motor output configuration. Defaults to MotorOutputConfigs().
                        current_limits (CurrentLimitsConfigs): Current limiting configuration. Defaults to CurrentLimitsConfigs().
                        voltage (VoltageConfigs): Voltage-related configuration. Defaults to VoltageConfigs().
                        torque_current (TorqueCurrentConfigs): Torque/current configuration. Defaults to TorqueCurrentConfigs().
                        feedback (FeedbackConfigs): Additional feedback configuration. Defaults to FeedbackConfigs().
                        differential_sensors (DifferentialSensorsConfigs): Differential sensor configuration.
                            Defaults to DifferentialSensorsConfigs().
                        differential_constants (DifferentialConstantsConfigs): Differential constants configuration.
                            Defaults to DifferentialConstantsConfigs().
                        open_loop_ramps (OpenLoopRampsConfigs): Open-loop ramp configuration. Defaults to OpenLoopRampsConfigs().
                        closed_loop_ramps (ClosedLoopRampsConfigs): Closed-loop ramp configuration.
                            Defaults to ClosedLoopRampsConfigs().
                        hardware_limit_switch (HardwareLimitSwitchConfigs): Hardware limit switch configuration.
                            Defaults to HardwareLimitSwitchConfigs().
                        audio (AudioConfigs): Audio-related configuration. Defaults to AudioConfigs().
                        software_limit_switch (SoftwareLimitSwitchConfigs): Software limit switch configuration.
                            Defaults to SoftwareLimitSwitchConfigs().
                        motion_magic (MotionMagicConfigs): Motion Magic configuration. Defaults to MotionMagicConfigs().
                        custom_params (CustomParamsConfigs): Custom parameter mappings. Defaults to CustomParamsConfigs().
                        closed_loop_general (ClosedLoopGeneralConfigs): General closed-loop settings.
                            Defaults to ClosedLoopGeneralConfigs().
                        slot0 (Slot0Configs): Alternative Slot 0 configuration object. Defaults to Slot0Configs().
                        slot1 (Slot1Configs): Alternative Slot 1 configuration object. Defaults to Slot1Configs().
                        slot2 (Slot2Configs): Slot 2 configuration object. Defaults to Slot2Configs().
        """
        self.id = id
        self.can_bus = can_bus
        self.motor_name = motor_name
        self.parent_nt = parent_nt
        super().__init__()
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)

    def with_id(self, id: int):
        """Sets the CAN ID for this motor configuration.

        Args:
            id (int): The CAN ID to set.
        Returns:
            FROGTalonFXConfig: The updated motor configuration.
        """
        self.id = id
        return self


class FROGTalonFX(TalonFX):
    """FROG custom TalonFX that takes parameters during instantiation."""

    def __init__(
        self,
        motor_config: FROGTalonFXConfig = FROGTalonFXConfig(),
    ):
        """Creates a TalonFX motor object with applied configuration

        Args:
            motor_config (FROGTalonFXConfig): The configuration to apply to the motor. Defaults to a default FROGTalonFXConfig.
        """
        super().__init__(device_id=motor_config.id, canbus=motor_config.can_bus)
        self.config = motor_config
        self.configurator.apply(self.config)

        if motor_config.motor_name == "":
            motor_config.motor_name = f"TalonFX({motor_config.id})"
        table = f"{motor_config.parent_nt}/{motor_config.motor_name}"

        self._motorVelocityPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/velocity")
            .publish()
        )
        self._motorPositionPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/position")
            .publish()
        )
        self._motorVoltagePub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{table}/voltage")
            .publish()
        )

    def getMotorVoltage(self):
        return self.get_motor_voltage().value

    def logData(self):
        """Logs data to network tables for this motor"""
        self._motorVelocityPub.set(self.get_velocity().value)


# TODO: #7 Refactor gyro class as a subclass of Pigeon2
class FROGPigeonGyro:
    "Gyro class that creates an instance of the Pigeon 2.0 Gyro"

    def __init__(self, can_id: int):
        self.gyro = Pigeon2(can_id)
        self.gyro.reset()

    def getAngleCCW(self) -> float:
        # returns gyro heading
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return self.getYaw()

    def getYaw(self) -> float:
        return self.gyro.get_yaw().value

    def getRoll(self) -> float:
        return self.gyro.get_roll().value

    def getPitch(self) -> float:
        return self.gyro.get_pitch().value

    def getDegreesPerSecCCW(self) -> float:
        return self.gyro.get_angular_velocity_z_world().value

    def getRadiansPerSecCCW(self) -> float:
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self) -> Rotation2d:
        return self.gyro.getRotation2d()

    def setAngleAdjustment(self, angle):
        self.gyro.set_yaw(angle)


class FROGCANCoderConfig(CANcoderConfiguration):
    """Inheretis from CANcoderConfiguration and add the ability to pass in steer offset
    during instantiation."""

    def __init__(self, steer_offset=0):
        super().__init__()
        self.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        self.magnet_sensor.magnet_offset = steer_offset
        self.magnet_sensor.sensor_direction = (
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )

    def with_id(self, id: int):
        """Sets the CAN ID for this CANCoder configuration.

        Args:
            id (int): The CAN ID to set.
        Returns:
            FROGCANCoderConfig: The updated CANCoder configuration.
        """
        self.id = id
        return self

    def with_offset(self, offset: float):
        """Sets the steer offset for this CANCoder configuration.

        Args:
            offset (float): The steer offset to set.
        Returns:
            FROGCANCoderConfig: The updated CANCoder configuration.
        """
        self.magnet_sensor.magnet_offset = offset
        return self


class FROGCanCoder(CANcoder):
    def __init__(self, config: FROGCANCoderConfig):
        super().__init__(config.id)
        self.configurator.apply(config)
        # self._motorPositionPub.set(self.get_position().value)
        # self._motorVoltagePub.set(self.get_motor_voltage().value)
