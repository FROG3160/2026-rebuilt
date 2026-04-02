from pathplannerlib.auto import NamedCommands, AutoBuilder
from pathplannerlib.path import PathConstraints, PathPlannerPath
from wpilib import SmartDashboard

from commands.drive import ManualDrive, ManualDriveAndAim, ManualDriveAndClusterAim
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

        self.drive = Drive()
        self.intake = Intake(self.drive.get_linear_speed)
        self.hopper = Hopper()
        self.shooter = Shooter(self.drive.get_distance_to_target)
        self.feeder = Feeder()

        self.driver_xbox = FROGXboxDriver(
            constants.Controller.DRIVER_CONTROLLER_PORT,
            constants.Controller.DEADBAND,
            constants.Controller.DEBOUNCE_PERIOD,
            constants.Controller.TRANSLATION_SLEW,
            constants.Controller.ROT_SLEW,
        )
        self.shift_tracker = ShiftTracker()
        self.field_zones = FieldZones(
            self.drive.getPose,
            lambda: not self.shooter.is_hood_closed(),
            self.drive.estimator_field,
        )

        self.register_named_commands()
        self.configure_automation_bindings()

        # self.hopper.setDefaultCommand(self.hopper.serialize_cmd())

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
                constants.Drive.PP_MAX_VELOCITY,
                constants.Drive.PP_MAX_ACCELERATION,
                constants.Drive.PP_MAX_ANGULAR_VELOCITY,
                constants.Drive.PP_MAX_ANGULAR_ACCELERATION,
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
            # Reverse while opening (Hopper will follow Feeder via Trigger)
            cmd.parallel(
                cmd.runOnce(self.shooter.deploy_hood),
                self.feeder.run_backward_cmd(),
            ).until(self.shooter.is_hood_open),
            # Transition to firing logic once open
            self.shooter.fire_with_distance_cmd().alongWith(
                cmd.waitUntil(self.shooter.is_at_speed).andThen(
                    self.feeder.run_forward_cmd()
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
        feeder_running_forward_tgr = self.feeder.is_feeding_forward()
        feeder_running_backward_tgr = self.feeder.is_reversing()

        is_intaking = self.driver_xbox.leftTrigger()

        # The hopper should follow the feeder automatically.
        feeder_running_forward_tgr.whileTrue(
            self.hopper.fire_forward_cmd().withName("Fire Hopper")
        )

        # The hopper should run backward whenever the feed motor is running backward.
        feeder_running_backward_tgr.whileTrue(self.hopper.run_backward_cmd())

        # Intake behavior when firing: Cycle if NOT intaking, otherwise manual takes precedence
        feeder_running_forward_tgr.and_(is_intaking.negate()).whileTrue(
            self.intake.cycle_cmd().withName("Firing Cycle")
        )

        # Centralized retract logic: Retract only when neither firing nor intaking
        feeder_running_forward_tgr.or_(is_intaking).onFalse(
            self.intake.retract_and_stop_cmd().withName("Smart Retract")
        )

        # Rumble driver controller when shift is ending soon (5s left)
        Trigger(self.shift_tracker.is_shift_ending_soon).onTrue(
            cmd.startEnd(self.driver_xbox.leftRumble, self.driver_xbox.stopLeftRumble)
            .withTimeout(0.5)
            .ignoringDisable(True)
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
                # Use the unified firing group logic for teleop as well
                self.get_firing_command_group()
            )
            .withName("Aim and Fire")
        )  # name the command for better dashboard visibility
        self.driver_xbox.start().onTrue(
            self.drive.runOnce(self.drive.reset_initial_pose)
        )

        # Reverse feed motor to empty the hopper (hopper will follow automatically via triggers)
        self.driver_xbox.x().whileTrue(self.feeder.run_backward_cmd().withName("Unjam"))

        self.driver_xbox.leftTrigger().onTrue(self.intake.run_and_deploy_cmd())

        self.driver_xbox.rightBumper().whileTrue(
            DeferredCommand(lambda: self.get_pathfinding_command(), self.drive)
        )

        self.driver_xbox.back().onTrue(self.shooter.zero_hood_cmd())

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
        NamedCommands.registerCommand("Intake Start", self.intake.run_and_deploy_cmd())
        NamedCommands.registerCommand("Intake Stop", self.intake.retract_and_stop_cmd())
        NamedCommands.registerCommand("Intake Cycle", self.intake.cycle_cmd())

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

    def configureSysIDButtonBindings(self) -> None:
        """Configure button bindings for SysId drive motor routine tests."""
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
        self.driver_xbox.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
        self.driver_xbox.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))

    def configureSysIDSteerButtonBindings(self) -> None:
        """Configure button bindings for SysId steer motor routine tests."""
        # Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        # once.
        self.driver_xbox.a().whileTrue(
            self.drive.sysIdQuasistaticSteer(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.b().whileTrue(
            self.drive.sysIdQuasistaticSteer(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.x().whileTrue(
            self.drive.sysIdDynamicSteer(SysIdRoutine.Direction.kForward)
        )
        self.driver_xbox.y().whileTrue(
            self.drive.sysIdDynamicSteer(SysIdRoutine.Direction.kReverse)
        )
        self.driver_xbox.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
        self.driver_xbox.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))

    def configureComponentTestBindings(self) -> None:
        """Button bindings for manual component testing in test mode."""

        # Intake via left trigger, like teleop
        self.driver_xbox.leftTrigger().onTrue(
            self.intake.run_and_deploy_cmd().withName("Test Intake")
        ).onFalse(self.intake.retract_and_stop_cmd())

        # Run the full sequence: hopper, feeder, flywheel, and hood
        # using the commanded flywheel speed instead of calculating from distance.
        # Hopper is triggered automatically when feeder runs forward.
        self.driver_xbox.a().whileTrue(
            cmd.sequence(
                # Reverse while opening
                cmd.parallel(
                    cmd.runOnce(self.shooter.deploy_hood),
                    self.feeder.run_backward_cmd(),
                    self.hopper.run_backward_cmd(),
                ).until(self.shooter.is_hood_open),
                # Transition to firing logic once open
                self.shooter.fire_at_set_speed_cmd().alongWith(
                    cmd.waitUntil(self.shooter.is_at_speed).andThen(
                        self.feeder.run_forward_cmd().alongWith(
                            self.hopper.fire_forward_cmd()
                        )
                    )
                ),
            )
            .finallyDo(lambda interrupted: self.shooter.retract_hood())
            .withName("Fire Test Sequence")
        )
