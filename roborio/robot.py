#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from wpilib import DriverStation

from FROGlib.xbox import FROGXboxDriver
from robotcontainer import RobotContainer
import constants


# from components.component1 import Component1
# from components.component2 import Component2


class FROGBot(commands2.TimedCommandRobot):
    """Main robot class"""

    def robotInit(self) -> None:
        """Initialize all wpilib motors & sensors"""
        self.alliance = None

        self.container = RobotContainer()

        # self.component1_motor = wpilib.Talon(1)
        # self.some_motor = wpilib.Talon(2)

        # self.joystick = wpilib.Joystick(0)

    #
    # No autonomous routine boilerplate required here, anything in the
    # autonomous folder will automatically get added to a list
    #

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
        self.container.configureSysIDButtonBindings()
        # self.container.drive.removeDefaultCommand()
        self.container.drive.enable()

    def testPeriodic(self) -> None:
        pass

    def robotPeriodic(self):
        self.container.shift_tracker.update()
        self.container.shift_tracker.put_to_dashboard()
