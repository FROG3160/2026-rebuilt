import math
from FROGlib.swerve import SwerveBase
from FROGlib.ctre import FROGPigeonGyro
from configs import ctre

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
from FROGlib.utils import RobotRelativeTarget, remap
import constants
from subsystems.positioning import Position
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

# from subsystems.leds import LEDSubsystem
from subsystems.vision import VisionPose


class DriveChassis(SwerveBase):

    def __init__(
        self,
        positioningCameras: list[VisionPose],
        positioning=Position(),
        parent_nt: str = "Subsystems",
    ):
        super().__init__(
            swerve_module_configs=(
                ctre.swerveModuleFrontLeft,
                ctre.swerveModuleFrontRight,
                ctre.swerveModuleBackLeft,
                ctre.swerveModuleBackRight,
            ),
            gyro=FROGPigeonGyro(constants.kGyroID),
            max_speed=constants.kMaxMetersPerSecond,
            max_rotation_speed=constants.kMaxChassisRadiansPerSec,
            parent_nt=parent_nt,
        )
        self.resetController = True

        self.positioningCameras = positioningCameras

        # initializing the estimator to 0, 0, 0
        self.estimatorPose = Pose2d(0, 0, Rotation2d(0))
        self.pose_set = False

        self.positioning = positioning

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
            lambda speeds, feedforwards: self.setChassisSpeeds(
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
        self.reef_scoring_position = self.positioning.CENTER

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

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

    def sysIdQuasistaticDrive(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_drive.quasistatic(direction)

    def sysIdDynamicDrive(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_drive.dynamic(direction)

    def sysIdQuasistaticSteer(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.quasistatic(direction)

    def sysIdDynamicSteer(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine_steer.dynamic(direction)

    # PathPlanner Auto Commands
    def _get_reef_scoring_pose(self) -> Pose2d:
        reef_pose = self.positioning.get_closest_reef_pose(
            self.estimator.getEstimatedPosition()
        )

        return reef_pose.transformBy(
            self.positioning.TRANSFORMS[self.reef_scoring_position]
        )

    def _get_reef_scoring_path(self) -> str:
        path_suffixes = ["Left Stem", "Center", "Right Stem"]
        closest_tag = self.positioning.get_closest_reef_tag_num(
            self.estimator.getEstimatedPosition()
        )
        return f"Reef {str(self.positioning.get_reef_enum_name(closest_tag))} - {path_suffixes[self.reef_scoring_position]}"

    def drive_to_reef_scoring_pose(self) -> Command:
        return AutoBuilder.pathfindToPose(
            self._get_reef_scoring_pose(),
            PathConstraints(
                # constants.kMaxTrajectorySpeed,
                # constants.kMaxTrajectoryAccel,
                2,
                2,
                constants.kProfiledRotationMaxVelocity,
                constants.kProfiledRotationMaxAccel,
            ),
        ).withName("PathFindToReefScoringPose")

    def drive_to_reef_scoring_path(self) -> Command:
        return AutoBuilder.followPath(
            PathPlannerPath.fromPathFile(self._get_reef_scoring_path())
        )

    def driveAutoPath(self, pathname) -> Command:
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathname))

    def resetRotationController(self):
        self.profiledRotationController.reset(
            self.getRotation2d().radians(),
            self.gyro.getRadiansPerSecCCW(),
        )

    def enableResetController(self):
        self.resetController = True

    def resetRotationControllerCommand(self):
        return self.runOnce(self.enableResetController)

    def periodic(self):
        # update estimator with chassis data
        self.estimatorPose = self.estimator.update(
            self.gyro.getRotation2d(), tuple(self.getModulePositions())
        )

        # Updates pose estimator with target data from positioning cameras.
        for camera in self.positioningCameras:
            camera_pose = camera.periodic()
            if camera_pose is not None:
                estimated_pose2d = camera_pose.estimatedPose.toPose2d()

                for target in camera_pose.targetsUsed:
                    translation = (
                        target.bestCameraToTarget.translation().toTranslation2d()
                    )
                    distance = math.sqrt(translation.x**2 + translation.y**2)
                    target.getFiducialId()
                    target.getPoseAmbiguity()
                    # print(f"Distance: {distance}")

                # if (
                #     abs(estimated_pose2d.x - self.estimatorPose.x) < 1
                #     and abs(estimated_pose2d.y - self.estimatorPose.y) < 1
                # ) or self.pose_set == False:
                #     self.pose_set = True
                # TODO:  We may want to validate the first instance of tagData
                # is a valid tag by checking tagData[0].id > 0

                translationStdDev = remap(distance, 0, 6, 0.2, 0.8)
                # rotation stdev is high because we are relying on the gyro as more accurate
                rotationStdDev = math.pi

                self.estimator.addVisionMeasurement(
                    camera_pose.estimatedPose.toPose2d(),
                    camera_pose.timestampSeconds,
                    (translationStdDev, translationStdDev, rotationStdDev),
                )
                # put camera pose on the field
                cameraPoseObject = self.field.getObject(
                    camera.estimator._camera.getName()
                )
                cameraPoseObject.setPose(camera_pose.estimatedPose.toPose2d())

        self.field.setRobotPose(self.estimator.getEstimatedPosition())
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
