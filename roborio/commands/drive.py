from commands2 import Command
from ntcore import NetworkTableInstance
from FROGlib.xbox import FROGXboxDriver
from FROGlib.vision import FROGDetector
from subsystems.drive import Drive
from wpimath.geometry import Pose2d, Translation2d
from wpimath.units import degreesToRadians
import time


from typing import Callable, Optional

class ManualDrive(Command):
    def __init__(
        self, 
        controller: FROGXboxDriver, 
        drive: Drive, 
        speed_scalar_supplier: Optional[Callable[[], float]] = None,
        table: str = "Commands"
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            speed_scalar_supplier (Callable[[], float], optional): A supplier for a speed modifier scalar. Defaults to returning 1.0.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.speed_scalar_supplier = speed_scalar_supplier if speed_scalar_supplier else lambda: 1.0
        self.addRequirements(self.drive)
        self.resetController = True
        # profiledRotationConstraints = TrapezoidProfileRadians.Constraints(
        #     constants.kProfiledRotationMaxVelocity, constants.kProfiledRotationMaxAccel
        # )
        # self.profiledRotationController = ProfiledPIDControllerRadians(
        #     constants.kProfiledRotationP,
        #     constants.kProfiledRotationI,
        #     constants.kProfiledRotationD,
        #     profiledRotationConstraints,
        # )
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )
        self._rotation_degreesPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/rotation_degrees")
            .publish()
        )

    # def resetRotationController(self):
    #     self.profiledRotationController.reset(
    #         self.drive.getRotation2d().radians(),
    #         self.drive.gyro.getRadiansPerSecCCW(),
    #     )

    def execute(self) -> None:

        driveRotation2d = self.drive.getRotation2d()

        # Apply scalar to rotation
        scalar = self.speed_scalar_supplier()
        
        vT = self.controller.getSlewLimitedFieldRotation() * scalar
        self._rotation_degreesPub.set(driveRotation2d.degrees())

        # pov = self.controller.getPOVDebounced()
        # if pov != -1:
        #     vX, vY = povSpeeds[pov]
        # else:

        vX = self.controller.getSlewLimitedFieldForward() * scalar
        vY = self.controller.getSlewLimitedFieldLeft() * scalar

        self.drive.fieldOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )


class ManualDriveAndAim(Command):
    def __init__(
        self,
        target_supplier: Callable[[], Optional[Pose2d]],
        controller: FROGXboxDriver,
        drive: Drive,
        table: str = "Commands",
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller, with automatic aiming.

        Args:
            target_supplier (Callable[[], Optional[Pose2d]]): Function that returns the target the robot should aim at, or None for manual aiming.
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.target_supplier = target_supplier
        self.addRequirements(self.drive)
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def initialize(self):
        self.drive.resetRotationController()

    def execute(self) -> None:

        vX = self.controller.getSlewLimitedFieldForward()
        vY = self.controller.getSlewLimitedFieldLeft()

        target = self.target_supplier()

        if target is not None:
            # get target adjusted for robot movement
            new_target = self.drive.getMotionAdjustedTarget(target)

            # calculate the required rotational velocity to face the new target
            vT = self.drive.profiledRotationController.calculate(
                self.drive.getRotation2d().radians(),
                self.drive.calculateHeadingToTarget(new_target),
            )
            self._calculated_vTPub.set(vT)
        else:
            # Fallback to manual control if no target is provided
            vT = self.controller.getSlewLimitedFieldRotation()
            self._calculated_vTPub.set(0.0)

        self.drive.fieldOrientedAutoRotateDrive(
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )


class ManualDriveAndClusterAim(Command):
    def __init__(
        self,
        controller: FROGXboxDriver,
        drive: Drive,
        fuel_detector: FROGDetector,
        table: str = "Commands",
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller, with automatic aiming at detected fuel clusters.

        Args:
            aim_point (Pose2d): The target the robot should aim at.
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.fuel_detector = fuel_detector
        self.addRequirements(self.drive)
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def initialize(self):
        self.drive.resetRotationController()

    def execute(self) -> None:

        vX = self.controller.getSlewLimitedFieldForward()
        vY = self.controller.getSlewLimitedFieldLeft()

        # calculate the cluster center
        start_time = time.perf_counter()
        detection = (
            self.fuel_detector.get_detection_results()
        )  # or get_alt_detection_results
        end_time = time.perf_counter()

        # invert the yaw value of the detection target and convert to radians
        if detection:
            rotation_offset = degreesToRadians(
                -self.fuel_detector.detection_target.target_yaw  # type: ignore
            )
        else:
            rotation_offset = 0

        current_rotation = self.drive.getRotation2d().radians()

        # calculate the required rotational velocity to face the new target
        vT = self.drive.profiledRotationController.calculate(
            current_rotation,
            current_rotation + rotation_offset,
        )
        self._calculated_vTPub.set(vT)

        self.drive.fieldOrientedAutoRotateDrive(
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )


class DrivePathAndAim(Command):
    def __init__(
        self,
        aim_point: Pose2d,
        path_name: str,
        drive: Drive,
        table: str = "Undefined",
    ) -> None:
        self.target = aim_point
        self.drive = drive
        self.path = self.drive.getPathPlannerPath(path_name)
        self.addRequirements(self.drive)
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def getRotationSpeed(self) -> float:
        new_target = self.drive.getMotionAdjustedTarget(self.target)
        return self.drive.profiledRotationController.calculate(
            self.drive.getRotation2d().radians(),
            self.drive.calculateHeadingToTarget(new_target),
        )

    def initialize(self):
        self.drive.holonomic_drive_ctrl.overrideRotationFeedback(self.getRotationSpeed)
        self.drive.resetRotationController()

    def execute(self) -> None:
        pass

    def end(self, interrupted: bool) -> None:
        self.drive.holonomic_drive_ctrl.clearRotationFeedbackOverride()
