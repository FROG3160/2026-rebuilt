import math
import random
from typing import Optional
from FROGlib.swerve import SwerveChassis, RotationControllerConfig
from FROGlib.ctre import (
    FROGCANCoderConfig,
    FROGFeedbackConfig,
    FROGPigeonGyro,
    FROGTalonFX,
)
from wpilib import DriverStation, Field2d, RobotBase
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Translation3d,
    Transform3d,
    Rotation3d,
    Pose3d,
)
from wpimath.units import volts
from wpilib.sysid import SysIdRoutineLog
from wpiutil import Sendable, SendableBuilder
from commands2 import Command
from commands2.sysid import SysIdRoutine
from FROGlib.utils import DriveTrain, remap
import constants
from phoenix6.controls import (
    PositionVoltage,
    VoltageOut,
)
from phoenix6 import SignalLogger
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.path import PathPlannerPath
from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from FROGlib.swerve import SwerveModuleConfig
from FROGlib.ctre import (
    FROGTalonFXConfig,
    FROGSlotConfig,
    MOTOR_OUTPUT_CCWP_BRAKE,
    MOTOR_OUTPUT_CWP_BRAKE,
)
from FROGlib.sds import MK5I_R3_GEARING, WHEEL_DIAMETER
from FROGlib.vision import (
    FROGPoseEstimator,
)
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from phoenix6.configs.config_groups import ClosedLoopGeneralConfigs
from copy import deepcopy
from FROGlib.subsystem import FROGSubsystem

# TODO: #3 Switch gear_stages to correct swerve module gearing when available
drivetrain = DriveTrain(gear_stages=MK5I_R3_GEARING, wheel_diameter=WHEEL_DIAMETER)

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
        constants.kWheelBaseMeters / 2, constants.kTrackWidthMeters / 2
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
        constants.kWheelBaseMeters / 2, -constants.kTrackWidthMeters / 2
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
        -constants.kWheelBaseMeters / 2, constants.kTrackWidthMeters / 2
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
        -constants.kWheelBaseMeters / 2, -constants.kTrackWidthMeters / 2
    ),
    "drive_motor_config": deepcopy(drive_config.with_id(constants.kBackRightDriveID)),
    "steer_motor_config": deepcopy(steer_config.with_id(constants.kBackRightSteerID)),
    "cancoder_config": FROGCANCoderConfig()
    .with_id(constants.kBackRightSensorID)
    .with_offset(constants.kBackRightOffset),
    "wheel_diameter": WHEEL_DIAMETER,
}


class Drive(FROGSubsystem, SwerveChassis):
    """The drive subsystem that subclasses FROGlib.SwerveChassis and adds
    additional components, attributes and methods for autonomous driving, etc.

    """

    def __init__(self):
        SwerveChassis.__init__(
            self,
            swerve_module_configs=(
                SwerveModuleConfig(**front_left_module_config),
                SwerveModuleConfig(**front_right_module_config),
                SwerveModuleConfig(**back_left_module_config),
                SwerveModuleConfig(**back_right_module_config),
            ),
            gyro=FROGPigeonGyro(constants.kGyroID),
            rotation_contoller_config=RotationControllerConfig(
                constants.kProfiledRotationP,
                constants.kProfiledRotationI,
                constants.kProfiledRotationD,
                constants.kProfiledRotationMaxVelocity,
                constants.kProfiledRotationMaxAccel,
            ),
            max_speed=constants.kMaxMetersPerSecond,
            max_rotation_speed=constants.kMaxChassisRadiansPerSec,
            parent_nt=constants.kComponentSubtableName,
        )
        FROGSubsystem.__init__(self)
        self.resetController = True
        self._distance_to_target = None

        self.photon_estimators: list[FROGPoseEstimator] = []

        if RobotBase.isSimulation():
            self.field_layout = AprilTagFieldLayout().loadField(
                AprilTagField.k2026RebuiltWelded
            )
        else:
            self.field_layout = AprilTagFieldLayout(
                r"/home/lvuser/py/2026_combined_field.json"
            )
        # field_layout = AprilTagFieldLayout().loadField(AprilTagField.kDefaultField)
        for config in constants.kCameraConfigs:
            self.photon_estimators.append(
                FROGPoseEstimator(
                    self.field_layout,  # field layout
                    config.name,
                    config.robotToCamera,
                )
            )

        # initializing the estimator to 0, 0, 0
        self.swerve_estimator_pose = Pose2d(0, 0, Rotation2d(0))
        self.initial_pose_set = False

        # self.positioning = positioning

        # create Field2d to display estimated swerve and camera poses
        self.estimator_field = Field2d()

        autobuilder_config = RobotConfig.fromGUISettings()

        self.holonomic_drive_ctrl = PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(4.0, 0.0, 0.0),  # Translation PID constants
            PIDConstants(4.0, 0.0, 0.0),  # Rotation PID constants
        )

        AutoBuilder.configure(
            self.getPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.apply_chassis_speeds(
                speeds
            ),  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            self.holonomic_drive_ctrl,
            autobuilder_config,  # The robot configuration
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self,  # Reference to this subsystem to set requirements
        )

        # Tell SysId to make generated commands require this subsystem, suffix test state in
        # WPILog with this subsystem's name ("drive")
        self.sys_id_routine_drive = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "state-drive", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(self._sysid_drive, lambda log: None, self),
        )

        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "state-steer", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(self._sysid_steer, lambda log: None, self),
        )
        self._vt_max_delta = 1.0
        self._vt_max_translationDistance = 6.0
        self._vt_min_translationStdDev = 0.2
        self._vt_max_translationStdDev = 0.8

        self.firing_target = None
        self._distance_to_target = None

    # Tell SysId how to plumb the driving voltage to the motors.
    def _sysid_drive(self, voltage: volts) -> None:
        for module in self.modules:
            module.steer_motor.set_control(
                PositionVoltage(
                    position=0,
                    slot=0,  # Duty Cycle gains for steer
                )
            )
            module.drive_motor.set_control(VoltageOut(output=voltage, enable_foc=False))

    def _sysid_steer(self, voltage: volts) -> None:
        for module in self.modules:
            module.steer_motor.set_control(VoltageOut(output=voltage, enable_foc=False))
            module.drive_motor.stopMotor()

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

    def invert_pose3d(self, pose: Pose3d) -> Pose3d:
        """Manually compute the inverse of a Pose3d."""
        original_rot = pose.rotation()
        original_trans = pose.translation()

        inv_rot = -original_rot  # Inverse rotation
        inv_trans = -original_trans  # -R^{-1} * t (unary minus on Translation3d)

        return Pose3d(inv_trans, inv_rot)

    def simulateTargets(self) -> list[PhotonTrackedTarget]:
        """Generate simulated PhotonTrackedTargets based on current odometry pose and field tags."""
        current_pose = (
            self.swerve_estimator.getEstimatedPosition()
        )  # 2D pose; extend to 3D if needed
        robot_pose3d = Pose3d(current_pose)  # Convert to 3D (Z=0 assumed)

        # Camera transform relative to robot (adjust based on your mount: e.g., forward 0.5m, up 0.3m)
        camera_offset = Transform3d(
            Translation3d(0.5, 0.0, 0.3), Rotation3d()
        )  # Example: front-mounted
        camera_pose = robot_pose3d.transformBy(camera_offset)

        targets = []
        # Sample 1-3 visible tags (in real sim, filter by angle/distance; here, pick first few for demo)
        visible_tag_ids = [
            1,
            2,
            3,
        ]  # Adjust to actual tag IDs you want to simulate (e.g., speaker tags)
        for tag_id in visible_tag_ids:
            if tag_id >= len(self.field_layout.getTags()):
                continue  # Skip invalid IDs

            tag_pose = self.field_layout.getTagPose(tag_id)
            if not tag_pose:
                continue

            # Compute camera-to-tag transform (inverse of tag-to-camera)
            tag_to_camera = camera_pose.relativeTo(tag_pose)
            camera_to_tag = self.invert_pose3d(tag_to_camera)

            # Add noise (Gaussian, realistic for sim: ~1-5% error)
            noise_trans = Translation3d(
                random.gauss(0, 0.01),  # X noise ~1cm
                random.gauss(0, 0.01),  # Y
                random.gauss(0, 0.02),  # Z ~2cm
            )
            noise_rot = Rotation3d(
                random.gauss(0, 0.01),  # Roll ~0.6°
                random.gauss(0, 0.01),  # Pitch
                random.gauss(0, 0.02),  # Yaw ~1.1°
            )
            noisy_camera_to_tag = camera_to_tag.transformBy(
                Transform3d(noise_trans, noise_rot)
            )

            # Alternate target (required; make it slightly off for ambiguity)
            alt_offset = Transform3d(
                Translation3d(0.05, 0.0, 0.0), Rotation3d(0, 0, 0.05)
            )
            alt_camera_to_tag = noisy_camera_to_tag.transformBy(alt_offset)

            # Create target (area=0.5 for ~50% visibility; poseAmbiguity=0.1 low confidence)
            target = PhotonTrackedTarget(
                tag_id,
                noisy_camera_to_tag,
                [alt_camera_to_tag],  # List of alternates (can be empty)
                0.5,  # detectedArea (0-1)
                0.1,  # poseAmbiguity (0-1; lower = better)
            )
            targets.append(target)

        return targets

    def simulateEstimatedPose(self) -> Optional[EstimatedRobotPose]:
        """Generate a simulated EstimatedRobotPose based on current odometry with noise."""
        if not RobotBase.isSimulation():
            return None  # Fallback to real estimator outside sim

        current_pose2d = self.swerve_estimator.getEstimatedPosition()
        current_pose3d = Pose3d(current_pose2d)  # Embed in 3D (Z=0)

        # Simulate vision correction: Add small Gaussian noise (e.g., 5cm trans, 2° rot)
        noise_trans = Translation3d(
            random.gauss(0, 0.05),  # X/Y noise ~5cm
            random.gauss(0, 0.05),
            random.gauss(0, 0.02),  # Z ~2cm
        )
        noise_rot = Rotation3d(
            random.gauss(0, 0.02),  # Roll/pitch ~1.1°
            random.gauss(0, 0.02),
            random.gauss(0, 0.035),  # Yaw ~2°
        )

        estimated_pose3d = current_pose3d.transformBy(
            Transform3d(noise_trans, noise_rot)
        )

        # High confidence for reliable sim corrections (tune: 0.5-0.9)
        confidence = 0.8 + random.uniform(-0.1, 0.1)  # Vary slightly: 0.7-0.9
        confidence = max(0.0, min(1.0, confidence))  # Clamp 0-1

        return EstimatedRobotPose(
            estimated_pose3d, self.loopTime, self.simulateTargets()
        )

    def getMotionAdjustedTarget(self, target_pose: Pose2d) -> Pose2d:
        # get the robot's current chassis speeds
        if RobotBase.isSimulation():
            chassis_speeds = self.chassisSpeeds
        else:
            chassis_speeds = self._actualChassisSpeeds
        robot_vector = Translation2d(chassis_speeds.vx, chassis_speeds.vy)
        robot_pose = self.getPose()
        # use the robot's rotation to convert robot-relative vector to field-relative
        field_vector = robot_vector.rotateBy(robot_pose.rotation())
        # move the translation of the target by the field-relative vector
        adjusted_translation = target_pose.translation() - field_vector
        # compose a new pose with the adjusted translation and original rotation
        new_target = Pose2d(adjusted_translation, target_pose.rotation())
        self._firing_target = new_target
        self._distance_to_target = robot_pose.translation().distance(
            new_target.translation()
        )
        return new_target

    def get_distance_to_target(self):
        return self._distance_to_target

    def getPathPlannerPath(self, pathname: str) -> PathPlannerPath:
        return PathPlannerPath.fromPathFile(pathname)

    def reset_initial_pose(self):
        self.initial_pose_set = False

    def periodic(self):
        SwerveChassis.periodic(self)
        FROGSubsystem.periodic(self)
        # update estimator with chassis data
        self.swerve_estimator_pose = self.swerve_estimator.update(
            self.gyro.getRotation2d(), tuple(self.getModulePositions())
        )
        # updates field2d object with the latest estimated pose
        self.estimator_field.setRobotPose(self.swerve_estimator_pose)

        # update estimator with photonvision estimates
        for estimator in self.photon_estimators:
            # TODO: #20 pass in swerve estimator pose and move the distance check into FROGPoseEstimator
            if RobotBase.isSimulation():
                vision_estimated_pose = self.simulateEstimatedPose()
            else:
                vision_estimated_pose = estimator.get_estimate()
            if type(vision_estimated_pose) is EstimatedRobotPose:
                # use distance to the target tags to calculate standard deviations
                tag_distance = (
                    vision_estimated_pose.targetsUsed[0]
                    .bestCameraToTarget.translation()
                    .toTranslation2d()
                    .norm()
                )
                translation_stddev = remap(
                    tag_distance,
                    0,
                    self._vt_max_translationDistance,
                    self._vt_min_translationStdDev,
                    self._vt_max_translationStdDev,
                )
                rotational_stddev = math.pi  # Rely on gyro for rotation
                # documentation for the swerve estimator recommends rejecting vision
                # measurements that are too far from the current estimate
                # calculate distance between vision pose and current estimator pose
                pose_delta = (
                    vision_estimated_pose.estimatedPose.toPose2d()
                    .translation()
                    .distance(self.swerve_estimator_pose.translation())
                )

                # only add vision measurement if within 1 meter of current estimate
                # put camera pose on the dashboard field
                cameraPoseObject = self.estimator_field.getObject(
                    estimator.camera.getName()
                )
                cameraPoseObject.setPose(vision_estimated_pose.estimatedPose.toPose2d())
                # if the initial pose hasn't been set yet, set it to the first valid vision pose
                if not self.initial_pose_set:
                    self.initial_pose_set = True
                    self.resetPose(
                        vision_estimated_pose.estimatedPose.toPose2d(),
                    )
                elif (
                    pose_delta < self._vt_max_delta
                    and tag_distance < self._vt_max_translationDistance
                ):
                    self.swerve_estimator.addVisionMeasurement(
                        vision_estimated_pose.estimatedPose.toPose2d(),
                        vision_estimated_pose.timestampSeconds,
                        (translation_stddev, translation_stddev, rotational_stddev),
                    )
                    self._useVisionMeasurementsPub.set(True)
                else:
                    self._useVisionMeasurementsPub.set(False)

                    # put camera pose on the estimator field2d

    @FROGSubsystem.tunable(1.0, "Vision/Max Delta")
    def vt_max_delta_tunable(self, val):
        self._vt_max_delta = val

    @FROGSubsystem.tunable(6.0, "Vision/Max Translation Distance")
    def vt_max_translationDistance_tunable(self, val):
        self._vt_max_translationDistance = val

    @FROGSubsystem.tunable(0.2, "Vision/Min Translation StdDev")
    def vt_min_translationStdDev_tunable(self, val):
        self._vt_min_translationStdDev = val

    @FROGSubsystem.tunable(0.8, "Vision/Max Translation StdDev")
    def vt_max_translationStdDev_tunable(self, val):
        self._vt_max_translationStdDev = val

    @FROGSubsystem.telemetry("Pose X")
    def pose_x_telem(self) -> float:
        return self.swerve_estimator_pose.x

    @FROGSubsystem.telemetry("Pose Y")
    def pose_y_telem(self) -> float:
        return self.swerve_estimator_pose.y

    @FROGSubsystem.telemetry("Pose Degrees")
    def pose_degrees_telem(self) -> float:
        return self.swerve_estimator_pose.rotation().degrees()

    @FROGSubsystem.telemetry("Distance to Target")
    def distance_to_target_telem(self) -> float:
        return self._distance_to_target if self._distance_to_target else 0.0

    @FROGSubsystem.telemetry("Estimated Pose")
    def estimated_pose_telem(self) -> Pose2d:
        return self.swerve_estimator_pose

    @FROGSubsystem.telemetry("Estimator Poses")
    def estimator_field_telem(self) -> Field2d:
        return self.estimator_field
