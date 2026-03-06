from pathplannerlib.auto import NamedCommands, AutoBuilder
from pathplannerlib.path import PathConstraints, PathPlannerPath
from wpilib import SmartDashboard

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
from FROGlib.subsystem import Direction
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from commands2.button import Trigger
from commands2 import Command, StartEndCommand, cmd, DeferredCommand
from typing import Optional, Callable
import constants

from wpimath.geometry import Pose2d, Rotation2d



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
        self.field_zones = FieldZones(
            self.drive.getPose,
            self.shooter.is_hood_deployed,
            self.drive.estimator_field,
        )

        self.register_named_commands()
        self.configure_automation_bindings()

        self.drive.setDefaultCommand(
            ManualDrive(
                self.driver_xbox,
                self.drive,
                None,  # self.field_zones.get_max_speed_scalar,
                self.field_zones.get_trench_velocity_limit,
            )
        )

        # Set up PathPlanner autos and publish to dashboard
        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)


    def get_pathfinding_command(self):
        """Returns a command to pathfind to a scoring position based on the robot's location."""
        path_name = self.field_zones.get_selection_zone()
        if path_name:
            path = PathPlannerPath.fromPathFile(path_name)
            constraints = PathConstraints(
                constants.kPPMaxVelocity,
                constants.kPPMaxAcceleration,
                constants.kPPMaxAngularVelocity,
                constants.kPPMaxAngularAcceleration,
            )
            return AutoBuilder.pathfindThenFollowPath(path, constraints)

        return cmd.none()

    def get_firing_command_group(
        self, target_supplier: Optional[Callable[[], Optional[Pose2d]]] = None
    ) -> Command:
        """Returns the full sequence for deploying hood, spinning up, and feeding.
        If target_supplier is provided, it will also update the distance calculation
        every loop and override the drive rotation feedback (useful for Auto).
        """
        # Command to update distance every loop. We don't add Drive requirement
        # here so it can run alongside manual drive or paths.
        update_dist_cmd = cmd.none()
        if target_supplier:
            update_dist_cmd = cmd.run(
                lambda: (
                    self.drive.getMotionAdjustedTarget(target)
                    if (target := target_supplier())
                    else None
                )
            )

        firing_sequence = cmd.sequence(
            cmd.runOnce(self.shooter.deploy_hood),
            cmd.waitUntil(self.shooter.is_hood_deployed),
            self.shooter.cmd_fire_with_distance().alongWith(
                cmd.waitUntil(self.shooter.is_at_speed).andThen(
                    self.feeder.runForward()
                )
            ),
        )

        # For the rotation override (used during autonomous paths)
        def get_vT_supplier():
            if target_supplier:
                if target := target_supplier():
                    return self.drive.calculate_vT_to_target(target)
            return 0.0

        return (
            firing_sequence.alongWith(update_dist_cmd)
            .beforeStarting(
                lambda: (
                    self.drive.holonomic_drive_ctrl.overrideRotationFeedback(
                        get_vT_supplier
                    )
                    if target_supplier
                    else None
                )
            )
            .finallyDo(
                lambda interrupted: (
                    self.shooter.retract_hood(),
                    (
                        self.drive.holonomic_drive_ctrl.clearRotationFeedbackOverride()
                        if target_supplier
                        else None
                    ),
                )
            )
            .withName("Firing Group")
        )

    def configure_automation_bindings(self) -> None:
        """Configure automation bindings for the robot."""
        # The hopper should run forward whenever either the intake OR the feed motors are running forward.
        Trigger(lambda: self.feeder.get_direction() == Direction.FORWARD).whileTrue(
            self.hopper.runForward()
        )
        # The hopper should run backward whenever either the intake OR the feed motors are running backward.
        Trigger(
            lambda: self.intake.get_direction() == Direction.REVERSE
            or self.feeder.get_direction() == Direction.REVERSE
        ).whileTrue(self.hopper.runBackward())

        self.fuel_detector.get_trigger_targets_close().whileTrue(
            self.intake.runForward()
        )

    def configure_xbox_bindings(self) -> None:
        """Configure button bindings for the xboxcontrollers."""
        self.configure_driver_controls()
        self.configure_tactical_controls()
        # Bind buttons to commands

    def configure_driver_controls(self) -> None:
        """Configure button bindings for the driver controller."""
        safe_to_shoot = self.field_zones.get_no_shoot_trigger().negate()
        self.driver_xbox.a().and_(safe_to_shoot).whileTrue(
            ManualDriveAndAim(
                self.field_zones.get_aim_target,
                self.driver_xbox,
                self.drive,
                None,  # self.field_zones.get_max_speed_scalar,
                self.field_zones.get_trench_velocity_limit,
                "DriveAndAim",
            )
            .alongWith(
                cmd.sequence(
                    cmd.runOnce(self.shooter.deploy_hood),
                    cmd.waitUntil(self.shooter.is_hood_deployed),
                    self.shooter.cmd_fire_with_distance().alongWith(
                        cmd.waitUntil(self.shooter.is_at_speed).andThen(
                            self.feeder.runForward()
                        )
                    ),
                ).finallyDo(lambda interrupted: self.shooter.retract_hood())
            )
            .withName("Aim and Fire")
        )  # name the command for better dashboard visibility
        self.driver_xbox.start().onTrue(
            self.drive.runOnce(self.drive.reset_initial_pose)
        )
        # self.driver_xbox.y().whileTrue(self.climber.lift_to_position(7.3))

        # Reverse intake and feed motors to empty the hopper (hopper will follow automatically via triggers)
        self.driver_xbox.x().whileTrue(
            self.intake.runBackward()
            .alongWith(self.feeder.runBackward())
            .withName("Eject All")
        )

        self.driver_xbox.y().whileTrue(self.intake.runForward())

        is_endgame = Trigger(self.shift_tracker.is_endgame)
        self.driver_xbox.leftBumper().and_(is_endgame).onTrue(
            self.climber.deploy_command()
        )
        self.driver_xbox.leftTrigger().and_(is_endgame).whileTrue(
            self.climber.stow_command()
        )

        # POV lift controls only when deployed AND in endgame
        is_deployed = Trigger(self.climber.is_deployed)
        self.driver_xbox.povUp().and_(is_deployed).and_(is_endgame).whileTrue(
            self.climber.lift_forward_cmd()
        )
        self.driver_xbox.povDown().and_(is_deployed).and_(is_endgame).whileTrue(
            self.climber.lift_reverse_cmd()
        )

        self.driver_xbox.rightBumper().whileTrue(
            DeferredCommand(lambda: self.get_pathfinding_command(), self.drive)
        )

    def configure_tactical_controls(self) -> None:
        """Configure button bindings for the tactical controller."""
        # Configure operator controls
        pass

    def register_named_commands(self) -> None:
        """Register named commands for use in autonomous routines."""
        # Register named commands
        NamedCommands.registerCommand(
            "Fire", self.get_firing_command_group(self.field_zones.get_aim_target)
        )

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

        # Climber Manual Controls (Test Mode)
        # Deploy on D-Pad (POV)
        self.driver_xbox.povUp().whileTrue(
            self.climber.manual_deploy_voltage_command(1.5).withName(
                "Manual Climber Deploy"
            )
        )
        self.driver_xbox.povDown().whileTrue(
            self.climber.manual_deploy_voltage_command(-1.5).withName(
                "Manual ClimberRetract"
            )
        )

        # Lift on Right Stick Y axis
        # Assuming the right stick Y value is positive when pushed down,
        # we negate it and multiply by max voltage (e.g. 12.0)
        def get_lift_voltage():
            # Apply deadband manually if FROGXboxDriver doesn't have a direct helper for right stick Y with deadband
            # Get raw right Y. WPILib XboxController returns negative for up, positive for down.
            # We want UP to lift (positive voltage), DOWN to lower (negative voltage).
            raw_y = self.driver_xbox.getRightY()
            if abs(raw_y) < constants.kDeadband:
                return 0.0
            return raw_y * -2.0

        # We need a trigger that evaluates to True when the stick is outside deadband
        right_stick_active = Trigger(
            lambda: abs(self.driver_xbox.getRightY()) > constants.kDeadband
        )
        right_stick_active.whileTrue(
            self.climber.manual_lift_voltage_command(get_lift_voltage).withName(
                "Manual Lift"
            )
        )
