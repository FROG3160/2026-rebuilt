from commands2 import Command
from ntcore import NetworkTableInstance
from FROGlib.xbox import FROGXboxDriver
from subsystems.drive import Drive


class ManualDrive(Command):
    def __init__(
        self, controller: FROGXboxDriver, drive: Drive, table: str = "Undefined"
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
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

        vT = self.controller.getSlewLimitedFieldRotation()
        self._rotation_degreesPub.set(driveRotation2d.degrees())

        # pov = self.controller.getPOVDebounced()
        # if pov != -1:
        #     vX, vY = povSpeeds[pov]
        # else:

        vX = self.controller.getSlewLimitedFieldForward()
        vY = self.controller.getSlewLimitedFieldLeft()

        self.drive.fieldOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )
