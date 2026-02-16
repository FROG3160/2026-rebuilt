#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from wpilib import DriverStation, SendableChooser, SmartDashboard
from robotcontainer import RobotContainer


class FROGBot(commands2.TimedCommandRobot):
    """Main robot class"""

    def robotInit(self) -> None:
        """Initialize all wpilib motors & sensors"""
        self.alliance = None

        self.container = RobotContainer()

        # Add SendableChooser for test modes
        self.test_chooser = SendableChooser()
        self.test_chooser.setDefaultOption("SysId Characterization", "sysid")
        self.test_chooser.addOption("Component Tests", "components")
        SmartDashboard.putData("Test Mode Chooser", self.test_chooser)

    def setAlliance(self):
        self.alliance = DriverStation.getAlliance()
        if self.alliance:
            self.container.driver_xbox.set_alliance(self.alliance)

    def autonomousInit(self):

        self.container.drive.enable()

    def teleopInit(self) -> None:
        self.setAlliance()
        self.container.drive.enable()

    def teleopPeriodic(self):
        pass

    def testInit(self) -> None:
        # self.container.drive.removeDefaultCommand()
        self.container.drive.enable()

        # Configure bindings based on chosen test mode
        selected = self.test_chooser.getSelected()
        if selected == "sysid":
            self.container.configureSysIDButtonBindings()
        elif selected == "components":
            self.container.configureComponentTestBindings()

    def testPeriodic(self) -> None:
        pass

    def robotPeriodic(self):
        self.container.shift_tracker.update()
        self.container.shift_tracker.put_to_dashboard()
