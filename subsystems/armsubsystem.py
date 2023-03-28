import commands2
import ctre
import rev
import enum
import wpimath.controller
import wpilib
from wpilib import SmartDashboard

class ArmSubsystem(commands2.PIDSubsystem):

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

        # self.motor_angleBackwardsLimit = wpilib.DigitalInput(0)
        
        self.enable()

        # Start with the cube as default mode
        #TODO: Change this to incorperate SendableChooser


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
    
    # #     if self.motor_angleForwardLimit.get():
    #         SmartDashboard.putBoolean("ForwardLimit", True)
    #         self.motor_angle.setSoftLimit(
    #             rev.CANSparkMax.SoftLimitDirection.kForward,
    #             self.motor_angleEncoder.getPosition()
    #         )
    #     else:
    #         SmartDashboard.putBoolean("ForwardLimit", False)

    #     if self.motor_angleBackwardsLimit.get():
    #         SmartDashboard.putBoolean("BackwardsLimit", True)
    #         self.motor_angleEncoder.setPosition(0)
    #         self.motor_angle.setSoftLimit(
    #             rev.CANSparkMax.SoftLimitDirection.kReverse,
    #             self.motor_angleEncoder.getPosition()
    #         )
    #     else:
    #         SmartDashboard.putBoolean("BackwardsLimit", False)
