from commands2 import Subsystem
from phoenix6.hardware import TalonFX
from FROGlib.ctre import (
    FROGSlotConfig,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGFeedbackConfig,
    MOTOR_OUTPUT_CWP_COAST,
)
from phoenix6 import controls, unmanaged
import constants
import wpilib
from wpimath.system.plant import DCMotor, LinearSystemId
from wpilib.simulation import DCMotorSim
from wpimath.units import radiansToRotations
from wpiutil import SendableBuilder


feed_slot0 = FROGSlotConfig(
    k_s=constants.kFeedS,
)

feed_motor_config = FROGTalonFXConfig(
    can_bus="rio",
    parent_nt="Feeder",
    motor_output=MOTOR_OUTPUT_CWP_COAST,
    feedback=FROGFeedbackConfig(sensor_to_mechanism_ratio=5.0),
    slot0=feed_slot0,
)


class Feeder(Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            motor_config=FROGTalonFXConfig(feed_motor_config)
            .with_id(constants.kFeedMotorID)
            .with_motor_name("Feed Motor")
        )
        self._feed_speed = 4.0  # in volts for now

        if wpilib.RobotBase.isSimulation():
            self.simulationInit()

    def _runForward(self):
        self.motor.set_control(controls.VoltageOut(self._feed_speed, enable_foc=False))

    def runForward(self):
        return self.startEnd(self._runForward, self.stop)

    def stop(self):
        self.motor.stopMotor()

    def simulationInit(self):
        # Feed motor simulation
        feed_gearbox = DCMotor.falcon500(1)
        J_feed = 0.0002
        feed_gearing = 1.0

        feed_plant = LinearSystemId.DCMotorSystem(feed_gearbox, J_feed, feed_gearing)
        self.sim = DCMotorSim(feed_plant, feed_gearbox, [0.0, 0.0])
        self.sim.setState(0.0, 0.0)

    def simulationPeriodic(self):
        unmanaged.feed_enable(0.100)
        dt = 0.020
        battery_v = wpilib.RobotController.getBatteryVoltage()

        self.motor.sim_state.set_supply_voltage(battery_v)

        feed_applied_v = self.motor.get_motor_voltage().value
        self.sim.setInputVoltage(feed_applied_v)
        self.sim.update(dt)

        feed_pos_rot = self.sim.getAngularPositionRotations()
        feed_vel_rps = radiansToRotations(self.sim.getAngularVelocity())

        self.motor.sim_state.set_raw_rotor_position(feed_pos_rot)
        self.motor.sim_state.set_rotor_velocity(feed_vel_rps)

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)
        builder.setSmartDashboardType("Feeder")

        builder.addDoubleProperty(
            "Feed Speed",
            lambda: self._feed_speed,
            lambda value: setattr(self, "_feed_speed", value),
        )
