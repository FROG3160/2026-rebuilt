import math
from FROGlib.swerve import SwerveChassis
from FROGlib.ctre import (
    FROGCANCoderConfig,
    FROGFeedbackConfig,
    FROGPigeonGyro,
    FROGTalonFX,
)

# from configs import ctre
from wpilib import DriverStation, Field2d
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Transform2d,
    Transform3d,
    Rotation3d,
)
from wpimath.units import volts
from wpilib.sysid import SysIdRoutineLog

# from subsystems.vision import PositioningSubsystem
# from subsystems.elevation import ElevationSubsystem
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from commands2.sysid import SysIdRoutine
from FROGlib.utils import DriveTrain, RobotRelativeTarget, remap
import constants

# from subsystems.positioning import Position
from wpimath.units import degreesToRadians, lbsToKilograms, inchesToMeters
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.controls import (
    PositionDutyCycle,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants, ModuleConfig, DCMotor
from pathplannerlib.path import PathPlannerPath, PathConstraints

from FROGlib.swerve import SwerveModuleConfig
from FROGlib.ctre import (
    FROGTalonFXConfig,
    FROGSlotConfig,
    MOTOR_OUTPUT_CCWP_BRAKE,
    MOTOR_OUTPUT_CWP_BRAKE,
)
from FROGlib.sds import MK4C_L3_GEARING, MK5I_R3_GEARING, WHEEL_DIAMETER
from phoenix6.configs.config_groups import ClosedLoopGeneralConfigs
from copy import deepcopy

# from subsystems.leds import LEDSubsystem
# from subsystems.vision import VisionPose

# TODO: #3 Switch gear_stages to correct swerve module gearing when available
drivetrain = DriveTrain(gear_stages=MK4C_L3_GEARING, wheel_diameter=WHEEL_DIAMETER)

drive_slot0 = FROGSlotConfig(
    k_s=constants.kVoltageDriveS,
    k_v=constants.kVoltageDriveV,
    k_a=constants.kVoltageDriveA,
    k_p=constants.kVoltageDriveP,
)
steer_slot0 = FROGSlotConfig(
    k_p=constants.kSteerP,
    k_i=constants.kSteerI,
    k_s=constants.kSteerS,
    k_v=constants.kSteerV,
)

# configure drive motor used for all swerve modules
drive_config = FROGTalonFXConfig(
    motor_output=MOTOR_OUTPUT_CWP_BRAKE,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=drivetrain.system_reduction),
    slot0=drive_slot0,
    motor_name="Drive",
)
# configure steer motor used for all swerve modules
steer_config = FROGTalonFXConfig(
    motor_output=MOTOR_OUTPUT_CCWP_BRAKE,
    # set continuous wrap to wrap around the 180 degree point
    closed_loop_general=ClosedLoopGeneralConfigs().with_continuous_wrap(True),
    slot0=steer_slot0,
    motor_name="Steer",
)


front_left_module_config = {
    "name": "FrontLeft",
    "location": Translation2d(
        constants.kWheelBaseMeters / 2, constants.kTrackWidthMeters
    ),
    "drive_motor_config": deepcopy(drive_config.with_id(constants.kFrontLeftDriveID)),
    "steer_motor_config": deepcopy(steer_config.with_id(constants.kFrontLeftSteerID)),
    "cancoder_config": FROGCANCoderConfig()
    .with_id(constants.kFrontLeftSensorID)
    .with_offset(constants.kFrontLeftOffset),
    "wheel_diameter": WHEEL_DIAMETER,
}
front_right_module_config = {
    "name": "FrontRight",
    "location": Translation2d(
        constants.kWheelBaseMeters / 2, -constants.kTrackWidthMeters
    ),
    "drive_motor_config": deepcopy(drive_config.with_id(constants.kFrontRightDriveID)),
    "steer_motor_config": deepcopy(steer_config.with_id(constants.kFrontRightSteerID)),
    "cancoder_config": FROGCANCoderConfig()
    .with_id(constants.kFrontRightSensorID)
    .with_offset(constants.kFrontRightOffset),
    "wheel_diameter": WHEEL_DIAMETER,
}
back_left_module_config = {
    "name": "BackLeft",
    "location": Translation2d(
        -constants.kWheelBaseMeters / 2, constants.kTrackWidthMeters
    ),
    "drive_motor_config": deepcopy(drive_config.with_id(constants.kBackLeftDriveID)),
    "steer_motor_config": deepcopy(steer_config.with_id(constants.kBackLeftSteerID)),
    "cancoder_config": FROGCANCoderConfig()
    .with_id(constants.kBackLeftSensorID)
    .with_offset(constants.kBackLeftOffset),
    "wheel_diameter": WHEEL_DIAMETER,
}
back_right_module_config = {
    "name": "BackRight",
    "location": Translation2d(
        -constants.kWheelBaseMeters / 2, -constants.kTrackWidthMeters
    ),
    "drive_motor_config": deepcopy(drive_config.with_id(constants.kBackRightDriveID)),
    "steer_motor_config": deepcopy(steer_config.with_id(constants.kBackRightSteerID)),
    "cancoder_config": FROGCANCoderConfig()
    .with_id(constants.kBackRightSensorID)
    .with_offset(constants.kBackRightOffset),
    "wheel_diameter": WHEEL_DIAMETER,
}


class Drive(SwerveChassis, Subsystem):
    """The drive subsystem that subclasses FROGlib.SwerveChassis and adds
    additional components, attributes and methods for autonomous driving, etc.

    """

    def __init__(
        self,
        # positioningCameras: list[VisionPose],
        # positioning=Position(),
    ):
        super().__init__(
            swerve_module_configs=(
                SwerveModuleConfig(**front_left_module_config),
                SwerveModuleConfig(**front_right_module_config),
                SwerveModuleConfig(**back_left_module_config),
                SwerveModuleConfig(**back_right_module_config),
            ),
            gyro=FROGPigeonGyro(constants.kGyroID),
            max_speed=constants.kMaxMetersPerSecond,
            max_rotation_speed=constants.kMaxChassisRadiansPerSec,
            parent_nt=constants.kComponentSubtableName,
        )
        self.resetController = True

        # self.positioningCameras = positioningCameras

        # initializing the estimator to 0, 0, 0
        self.estimatorPose = Pose2d(0, 0, Rotation2d(0))
        self.pose_set = False

        # self.positioning = positioning

        self.profiledRotationConstraints = TrapezoidProfileRadians.Constraints(
            constants.kProfiledRotationMaxVelocity, constants.kProfiledRotationMaxAccel
        )
        self.profiledRotationController = ProfiledPIDControllerRadians(
            constants.kProfiledRotationP,
            constants.kProfiledRotationI,
            constants.kProfiledRotationD,
            self.profiledRotationConstraints,
        )
        self.profiledRotationController.enableContinuousInput(-math.pi, math.pi)

        self.field = Field2d()
        SmartDashboard.putData("DrivePose", self.field)

        autobuilder_config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.apply_chassis_speeds(
                speeds
            ),  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(1.0, 0.0, 0.0),  # Translation PID constants
                PIDConstants(1.0, 0.0, 0.0),  # Rotation PID constants
            ),
            autobuilder_config,  # The robot configuration
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self,  # Reference to this subsystem to set requirements
        )

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine_drive = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(self.sysid_drive, self.sysid_log_drive, self),
        )

        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(self.sysid_steer, self.sysid_log_steer, self),
        )

    # Tell SysId how to plumb the driving voltage to the motors.
    def sysid_drive(self, voltage: volts) -> None:
        for module in self.modules:
            module.steer_motor.set_control(
                PositionVoltage(
                    position=0,
                    slot=0,  # Duty Cycle gains for steer
                )
            )
            module.drive_motor.set_control(VoltageOut(output=voltage, enable_foc=False))

    def sysid_steer(self, voltage: volts) -> None:
        for module in self.modules:
            module.steer_motor.set_control(VoltageOut(output=voltage, enable_foc=False))
            module.drive_motor.stopMotor()

    def sysid_log_drive(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for each module.  Since these share an encoder, we consider
        # the entire group to be one motor.
        for module in self.modules:
            with module.drive_motor as m:
                sys_id_routine.motor(module.name).voltage(
                    m.get_motor_voltage().value
                ).position(m.get_position().value).velocity(m.get_velocity().value)

    def sysid_log_steer(self, sys_id_routine: SysIdRoutineLog) -> None:
        # Record a frame for each module.  Since these share an encoder, we consider
        # the entire group to be one motor.
        for module in self.modules:
            with module.steer_motor as m:
                sys_id_routine.motor(module.name).voltage(
                    m.get_motor_voltage().value
                ).angularPosition(m.get_position().value).angularVelocity(
                    m.get_velocity().value
                )

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def sysIdQuasistaticDrive(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_drive.quasistatic(direction)

    def sysIdDynamicDrive(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_drive.dynamic(direction)

    def sysIdQuasistaticSteer(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.quasistatic(direction)

    def sysIdDynamicSteer(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.dynamic(direction)

    def driveAutoPath(self, pathname) -> Command:
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathname))

    def resetRotationController(self):
        self.profiledRotationController.reset(
            self.getRotation2d().radians(),
            self.gyro.getRadiansPerSecCCW(),
        )
        self.resetController = False

    def enableResetController(self):
        self.resetController = True

    def periodic(self):
        # update estimator with chassis data
        self.estimatorPose = self.estimator.update(
            self.gyro.getRotation2d(), tuple(self.getModulePositions())
        )

        # Updates pose estimator with target data from positioning cameras.
        # for camera in self.positioningCameras:
        #     camera_pose = camera.periodic()
        #     if camera_pose is not None:
        #         estimated_pose2d = camera_pose.estimatedPose.toPose2d()

        #         for target in camera_pose.targetsUsed:
        #             translation = (
        #                 target.bestCameraToTarget.translation().toTranslation2d()
        #             )
        #             distance = math.sqrt(translation.x**2 + translation.y**2)
        #             target.getFiducialId()
        #             target.getPoseAmbiguity()

        #         translationStdDev = remap(distance, 0, 6, 0.2, 0.8)
        #         # rotation stdev is high because we are relying on the gyro as more accurate
        #         rotationStdDev = math.pi

        #         self.estimator.addVisionMeasurement(
        #             camera_pose.estimatedPose.toPose2d(),
        #             camera_pose.timestampSeconds,
        #             (translationStdDev, translationStdDev, rotationStdDev),
        #         )
        #         # put camera pose on the field
        #         cameraPoseObject = self.field.getObject(
        #             camera.estimator._camera.getName()
        #         )
        #         cameraPoseObject.setPose(camera_pose.estimatedPose.toPose2d())

        # self.field.setRobotPose(self.estimatorPose)
        SmartDashboard.putNumberArray(
            "Drive Pose",
            [
                self.estimatorPose.x,
                self.estimatorPose.y,
                self.estimatorPose.rotation().radians(),
            ],
        )
        SmartDashboard.putNumber(
            "Pose Rotation", self.estimatorPose.rotation().degrees()
        )

        # run periodic method of the superclass, in this case SwerveChassis.periodic()
        super().periodic()
