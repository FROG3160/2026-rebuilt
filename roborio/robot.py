#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import DriverStation
from magicbot import MagicRobot

from components.drive import Drive
from FROGlib.xbox import FROGXboxDriver
import constants
from components.drive_control import DriveState

# from components.component1 import Component1
# from components.component2 import Component2


class FROGBot(MagicRobot):
    # Define components here
    #
    drive: Drive
    drive_statemachine: DriveState
    # component1: Component1
    # component2: Component2

    # You can even pass constants to components
    # SOME_CONSTANT = 1

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.alliance = None

        self.driver_xbox = FROGXboxDriver(
            constants.kDriverControllerPort,
            constants.kDeadband,
            constants.kDebouncePeriod,
            constants.kTranslationSlew,
            constants.kRotSlew,
        )

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
            self.driver_xbox.set_alliance(self.alliance)

    def autonomousInit(self):
        self.setAlliance()
        self.drive.enable()

    def teleopInit(self) -> None:
        self.setAlliance()
        self.drive.enable()

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        try:
            if self.driver_xbox.getRightTriggerAxis() > 0.5:
                pass
                # self.component2.do_something()
        except:
            self.onException()
