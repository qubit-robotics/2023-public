import commands2
import rev
import wpimath.controller
from wpilib import SmartDashboard
import wpilib
import enum

class Mode(enum.auto):
    CUBE = "cube"
    CONE = "cone"

class PWR(enum.auto):
    HOLD = 0.2
    SWALLOW = 0.5
    SPIT = -0.3

class ARM(commands2.PIDSubsystem):
    def __init__(self):
        super().__init__(
            wpimath.controller.PIDController(
                6,
                0,
                0
            ),
            0
        )
        self.motor_angle = rev.CANSparkMax(10, rev.CANSparkMax.MotorType.kBrushless)
        self.motor_angleEncoder = self.motor_angle.getEncoder()
        self.motor_gripper = wpilib.PWMSparkMax(3)
        self.enable()
    def topRow(self):
        self.setSetpoint(1.7)
    def midRow(self):
        self.setSetpoint(0.9)
    def retract(self):
        self.setSetpoint(0)
    def humanPlayerCube(self):
        self.setSetpoint(1.25)
    def humanPlayerCone(self):
        self.setSetpoint(1.35)
    def _getMeasurement(self) -> float:
        SmartDashboard.putNumber("armPidPos", self.motor_angleEncoder.getPosition())
        return self.motor_angleEncoder.getPosition()
    def _useOutput(self, output: float, setpoint: float) -> None:
        print("output voltage", output)
        self.motor_angle.setVoltage(output)
        SmartDashboard.putNumber("armPidOut", output)
    def disable(self) -> None:
        print("disable")
        SmartDashboard.putBoolean("ArmPIDStatus", False)
        return super().disable()
    def enable(self) -> None:
        print("enable")
        SmartDashboard.putBoolean("ArmPIDStatus", True)
        return super().enable()
    def raiseArmManual(self):
        print("manual raise")
        self.motor_angle.setVoltage(2)
    def retractArmManual(self):
        print("manual retract")
        self.motor_angle.setVoltage(-2)
    def stopArmManual(self):
        print("manual stop")
        self.motor_angle.set(0)
    def hold(self):
       self.motor_gripper.set(PWR.HOLD)
    
    def spit(self):
        print("spitting")
        self.motor_gripper.set(1)
    
    def swallow(self):
        print("swallowing")
        self.motor_gripper.set(-1)
    
    def stopIntake(self):
        print("no intake power")
        self.motor_gripper.set(0)