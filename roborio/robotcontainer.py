from pathplannerlib.auto import NamedCommands, AutoBuilder

from subsystems.climber import Climber
from commands.drive import ManualDrive, ManualDriveAndAim, ManualDriveAndClusterAim
from FROGlib.vision import FROGDetector
from subsystems.feedback import ShiftTracker, FieldZones
from subsystems.shooter import Shooter
from subsystems.hopper import Hopper
from subsystems.intake import Intake
from subsystems.feeder import Feeder
from subsystems.drive import Drive
from FROGlib.xbox import FROGXboxDriver
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from commands2.button import Trigger
from commands2 import StartEndCommand, cmd
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

        self.climber = Climber()
        self.fuel_detector = FROGDetector(constants.kDetectorConfigs[0])
        self.drive = Drive()
        self.intake = Intake()
        self.hopper = Hopper()
        self.shooter = Shooter(self.drive.get_distance_to_target)
        self.feeder = Feeder()

        self.driver_xbox = FROGXboxDriver(
            constants.kDriverControllerPort,
            constants.kDeadband,
            constants.kDebouncePeriod,
            constants.kTranslationSlew,
            constants.kRotSlew,
        )
        self.shift_tracker = ShiftTracker()
        self.field_zones = FieldZones(self.drive.getPose, self.drive.estimator_field)

        self.register_named_commands()
        self.configure_automation_bindings()

        self.drive.setDefaultCommand(
            ManualDrive(
                self.driver_xbox, 
                self.drive, 
                self.field_zones.get_max_speed_scalar
            )
        )

        # Set up PathPlanner autos and publish to dashboard
        # self.autochooser = AutoBuilder.buildAutoChooser()
        # SmartDashboard.putData("Auto Chooser", self.autochooser)

    def configure_automation_bindings(self) -> None:
        """Configure automation bindings for the robot."""
        # Configure automation bindings
        pass

    def configure_xbox_bindings(self) -> None:
        """Configure button bindings for the xboxcontrollers."""
        self.configure_driver_controls()
        self.configure_tactical_controls()
        # Bind buttons to commands

    def configure_driver_controls(self) -> None:
        """Configure button bindings for the driver controller."""
        self.driver_xbox.a().whileTrue(
            ManualDriveAndAim(
                self.field_zones.get_aim_target, self.driver_xbox, self.drive, "DriveAndAim"
            )
        )
        self.driver_xbox.start().onTrue(
            self.drive.runOnce(self.drive.reset_initial_pose)
        )
        self.driver_xbox.y().whileTrue(self.climber.lift_to_position(7.3))
        self.driver_xbox.x().whileTrue(self.climber.deploy_to_position(1.5))
        self.driver_xbox.b().whileTrue(
            self.intake.runForward().alongWith(self.hopper.runForward())
        )
        self.fuel_detector.get_trigger_targets_close().whileTrue(
            self.intake.runForward().alongWith(self.hopper.runForward())
        )
        
        safe_to_shoot = self.field_zones.get_no_shoot_trigger().negate()

        self.driver_xbox.rightBumper().and_(safe_to_shoot).whileTrue(
            self.shooter.cmd_fire_at_set_speed()
            .alongWith(
                cmd.waitUntil(self.shooter.is_at_speed).andThen(
                    self.hopper.runForward().alongWith(self.feeder.runForward())
                )
            )
            .withName("Fire Command")
        )  # max speed with 4" wheel is 33.8 m/s

        self.driver_xbox.leftBumper().whileTrue(self.climber.deploy_to_position(0.0))
        self.driver_xbox.leftTrigger().whileTrue(
            self.climber.lift_to_position(10.0)
        )  # 10.0 inches is arbitrary for now, will need to be tuned based on actual robot

    def configure_tactical_controls(self) -> None:
        """Configure button bindings for the tactical controller."""
        # Configure operator controls
        pass

    def register_named_commands(self) -> None:
        """Register named commands for use in autonomous routines."""
        # Register named commands
        # NamedCommands.registerCommand("shoot", ShootCommand(self.shooter))
        pass

    def configureSysIDFeederButtonBindings(self) -> None:
        """Configure button bindings for Feeder SysId routine tests."""
        self.driver_xbox.a().whileTrue(
            self.feeder.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.b().whileTrue(
            self.feeder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.x().whileTrue(
            self.feeder.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.y().whileTrue(
            self.feeder.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
        self.driver_xbox.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))

    def configureSysIDShooterButtonBindings(self) -> None:
        """Configure button bindings for Shooter SysId routine tests."""
        self.driver_xbox.a().whileTrue(
            self.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.b().whileTrue(
            self.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.x().whileTrue(
            self.shooter.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.y().whileTrue(
            self.shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
        self.driver_xbox.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))

    def configureSysIDClimberButtonBindings(self) -> None:
        """Configure button bindings for Climber SysId routine tests."""
        # Deploy routines on A/B/X/Y
        self.driver_xbox.a().whileTrue(
            self.climber.sysIdQuasistaticDeploy(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.b().whileTrue(
            self.climber.sysIdQuasistaticDeploy(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.x().whileTrue(
            self.climber.sysIdDynamicDeploy(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.y().whileTrue(
            self.climber.sysIdDynamicDeploy(SysIdRoutine.Direction.kReverse)
        )
        # Lift routines on triggers maybe? Or just keep it separate.
        # Let's put lift on D-pad for now if needed, but usually we do one mechanism at a time.
        self.driver_xbox.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
        self.driver_xbox.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))

    def configureSysIDButtonBindings(self) -> None:
        """Configure button bindings for SysId routine tests."""
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

    def configureComponentTestBindings(self) -> None:
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
            self.feeder.runForward().withName("Run Feeder")
        )

        # Flywheel: Using triggers as hold-to-run (common for speed control).
        # If you really want toggle on flywheel, we can use a button instead or add logic.
        self.driver_xbox.leftTrigger().whileTrue(
            StartEndCommand(
                self.shooter._apply_commanded_speed,
                self.shooter._stop_flywheel,
            ).withName("Run Flywheel")
        )
