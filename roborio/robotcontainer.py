from pathplannerlib.auto import NamedCommands, AutoBuilder
from wpilib import SmartDashboard

from commands.drive import ManualDrive
from subsystems.drive import Drive
from FROGlib.xbox import FROGXboxDriver
from FROGlib.xbox import FROGXboxTactical
from commands2.sysid import SysIdRoutine
import constants


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self):
        self.alliance = None
        self.drive = Drive()
        self.driver_xbox = FROGXboxDriver(
            constants.kDriverControllerPort,
            constants.kDeadband,
            constants.kDebouncePeriod,
            constants.kTranslationSlew,
            constants.kRotSlew,
        )

        self.register_named_commands()
        self.configure_button_bindings()
        self.configure_automation_bindings()

        self.drive.setDefaultCommand(ManualDrive(self.driver_xbox, self.drive))

        # Set up PathPlanner autos and publish to dashboard
        # self.autochooser = AutoBuilder.buildAutoChooser()
        # SmartDashboard.putData("PathPlanner Autos", self.autochooser)

    def configure_automation_bindings(self):
        # Configure automation bindings
        pass

    def configure_button_bindings(self):
        self.configure_driver_controls()
        self.configure_tactical_controls()
        # Bind buttons to commands

    def configure_driver_controls(self):
        # Configure driver controls
        pass

    def configure_tactical_controls(self):
        # Configure operator controls
        pass

    def register_named_commands(self):
        # Register named commands
        # NamedCommands.registerCommand("shoot", ShootCommand(self.shooter))
        pass

    def configureSysIDButtonBindings(self):
        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.
        self.driver_xbox.a().whileTrue(
            self.drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.b().whileTrue(
            self.drive.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.x().whileTrue(
            self.drive.sysIdDynamicDrive(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.y().whileTrue(
            self.drive.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse)
        )
