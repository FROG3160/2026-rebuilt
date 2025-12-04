#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

from wpilib import DriverStation
import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain, linear_deadzone
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
)


import typing

if typing.TYPE_CHECKING:
    from robot import FROGBot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "FROGBot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot objet
        """

        self.physics_controller = physics_controller
        self.robot = robot
        self.startingPose = None

        self.startingPositionSet = False

    def update_sim(self, now, tm_diff):
        # if we have a starting pose and haven't set it yet
        if not self.startingPositionSet and self.robot.alliance is not None:
            if self.robot.alliance == DriverStation.Alliance.kRed:
                self.startingPose = Pose2d(17.543, 8.048, Rotation2d(180))
            else:
                self.startingPose = Pose2d(0, 0, Rotation2d(0))
            self.physics_controller.field.setRobotPose(self.startingPose)
            self.startingPositionSet = True

        pose = self.physics_controller.drive(self.robot.drive.chassisSpeeds, tm_diff)
        self.robot.drive.field.setRobotPose(pose)
