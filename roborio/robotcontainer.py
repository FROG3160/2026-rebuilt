from pathplannerlib.auto import NamedCommands, AutoBuilder
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d

from commands.drive import ManualDrive, ManualDriveAndAim, ManualDriveAndClusterAim
from FROGlib.vision import FROGDetector
from subsystems.feedback import ShiftTracker
from subsystems.shooter import Shooter
from subsystems.hopper import Hopper
from subsystems.intake import Intake
from subsystems.drive import Drive
from FROGlib.xbox import FROGXboxDriver
from FROGlib.xbox import FROGXboxTactical
from commands2.sysid import SysIdRoutine
from wpilib.shuffleboard import Shuffleboard
from commands2.button import Trigger
from commands2 import StartEndCommand
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
        self.fuel_detector = FROGDetector(constants.kDetectorConfigs[0])
        self.drive = Drive()
        self.intake = Intake()
        self.hopper = Hopper()
        self.shooter = Shooter(self.drive)
        self.driver_xbox = FROGXboxDriver(
            constants.kDriverControllerPort,
            constants.kDeadband,
            constants.kDebouncePeriod,
            constants.kTranslationSlew,
            constants.kRotSlew,
        )
        self.shift_tracker = ShiftTracker()

        self.register_named_commands()
        self.configure_button_bindings()
        self.configure_automation_bindings()

        self.drive.setDefaultCommand(ManualDrive(self.driver_xbox, self.drive))
        Shuffleboard.getTab("Subsystems").add("Shooter", self.shooter)
        #   → /Subsystems/Shooter : shows subsystem status/command

        Shuffleboard.getTab("Tuning").add("Flywheel Gains", self.shooter)
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
        self.driver_xbox.a().whileTrue(
            ManualDriveAndAim(
                constants.kBlueHub, self.driver_xbox, self.drive, "DriveAndAim"
            )
        )
        self.driver_xbox.start().onTrue(
            self.drive.runOnce(self.drive.reset_initial_pose)
        )
        self.driver_xbox.y().whileTrue(
            ManualDriveAndClusterAim(
                self.driver_xbox,
                self.drive,
                self.fuel_detector,
                "DriveAndClusterAim",
            )
        )
        self.driver_xbox.x().whileTrue(
            self.intake.runBackward().alongWith(self.hopper.runBackward())
        )
        self.driver_xbox.b().whileTrue(
            self.intake.runForward().alongWith(self.hopper.runForward())
        )
        self.fuel_detector.get_trigger_targets_close().whileTrue(
            self.intake.runForward().alongWith(self.hopper.runForward())
        )
        # self.driver_xbox.leftBumper().and_(
        #     self.shooter.trigger_flywheel_at_speed()
        # ).whileTrue(
        #     self.shooter.run_feed_motor_forward().alongWith(self.hopper.runForward())
        # )
        self.driver_xbox.leftBumper().whileTrue(
            self.shooter.fire_command().alongWith(self.hopper.runForward())
        )  # max speed with 4" wheel is 33.8 m/s

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

    def configureComponentTestBindings(self):
        """Button bindings for manual component testing in test mode.
        Each press toggles the motor on/off (runs until toggled off or interrupted).
        """

        # Intake toggle forward
        self.driver_xbox.a().toggleOnTrue(
            self.intake.runForward().withName("Toggle Intake Forward")
        )

        # intake reverse toggles
        self.driver_xbox.b().toggleOnTrue(
            self.intake.runBackward().withName("Toggle Intake Reverse")
        )

        # Hopper toggles - forward
        self.driver_xbox.x().toggleOnTrue(
            self.hopper.runForward().withName("Toggle Hopper Forward")
        )
        # reverse
        self.driver_xbox.y().toggleOnTrue(
            self.hopper.runBackward().withName("Toggle Hopper Reverse")
        )

        # Shooter feed toggle
        self.driver_xbox.leftBumper().toggleOnTrue(
            StartEndCommand(
                self.shooter._run_feed_motor, lambda: self.shooter._stop_feed_motor
            ).withName("Run Shooter Feed")
        )

        # Flywheel: Using triggers as hold-to-run (common for speed control).
        # If you really want toggle on flywheel, we can use a button instead or add logic.
        self.driver_xbox.leftTrigger().whileTrue(
            StartEndCommand(
                self.shooter._apply_commanded_speed,
                self.shooter._stop_flywheel,
            ).withName("Run Flywheel")
        )
