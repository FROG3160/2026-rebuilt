import math
from magicbot import StateMachine, state, default_state
from components.drive import Drive
from FROGlib.xbox import FROGXboxDriver
from commands2 import Subsystem

povSpeed = 0.1
povSpeeds = {
    0: (povSpeed, 0),
    45: (povSpeed, -povSpeed),
    90: (0, -povSpeed),
    135: (-povSpeed, -povSpeed),
    180: (-povSpeed, 0),
    225: (-povSpeed, povSpeed),
    270: (0, povSpeed),
    315: (povSpeed, povSpeed),
}


class DriveState(StateMachine):
    drive: Drive
    driver_xbox: FROGXboxDriver

    def __init__(self) -> None:
        super().__init__()

    def setup(self) -> None:
        pass

    # State fieldOriented (as the default state) This will be the first state.
    @default_state
    def fieldOriented(self, initial_call):
        # if initial_call:
        #     self.drive.enableMinSpeed()
        rightStickY = self.driver_xbox.getRightY()
        if rightStickY > 0.5:
            if self.drive.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.logger.info("Resetting profiledRotationController")
                self.drive.resetRotationController()
            # Rotate to 0 degrees, point downfield
            vT = self.drive.profiledRotationController.calculate(
                self.drive.gyro.getRotation2d().radians(), math.radians(0)
            )
        elif rightStickY < -0.5:
            if self.drive.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.logger.info("Resetting profiledRotationController")
                self.drive.resetRotationController()
            # Rotate to 180 degrees
            # TODO: #5 switch from reading gyro to reading drive estimator
            vT = self.drive.profiledRotationController.calculate(
                self.drive.gyro.getRotation2d().radians(), math.radians(180)
            )
        else:
            # set to true so the first time the other if conditionals evaluate true
            # the controller will be reset
            self.drive.resetController = True
            vT = self.driver_xbox.getSlewLimitedFieldRotation()
        pov = self.driver_xbox.getPOV()
        if pov != -1:
            vX, vY = povSpeeds[pov]
        else:
            vX = self.driver_xbox.getSlewLimitedFieldForward()
            vY = self.driver_xbox.getSlewLimitedFieldLeft()

        self.drive.fieldOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.driver_xbox.getFieldThrottle(),
        )

    # for driving to objects on the field
    @state(first=True)
    def robotOriented(self, initial_call):
        self.drive.robotOrientedDrive(
            self.driver_xbox.getSlewLimitedFieldForward(),
            self.driver_xbox.getSlewLimitedFieldLeft(),
            self.driver_xbox.getSlewLimitedFieldRotation(),
            self.driver_xbox.getFieldThrottle(),
        )
