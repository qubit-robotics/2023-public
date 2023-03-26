import commands2
import ctre
import rev
import enum
import wpimath.controller
import wpilib
from wpilib import SmartDashboard

class Mode:
    CUBE = "cube"
    CONE = "cone"

class PWR:
    STOP = 0
    SWALLOW = 1
    SPIT = -1

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

        self.motor_gripper = wpilib.PWMSparkMax(9)
        self.motor_gripper.setInverted(False)
        self.motor_angle = rev.CANSparkMax(10, rev.CANSparkMax.MotorType.kBrushless)
        self.motor_angleEncoder = self.motor_angle.getEncoder()

        self.motor_angleBackwardsLimit = wpilib.DigitalInput(0)

        self.enable()

        # Start with the cube as default mode
        #TODO: Change this to incorperate SendableChooser
        self.currGripperMode = Mode.CUBE
        self.currGripperPWR = PWR.STOP

    def changeMode(self):
        print("mode changed")
        if self.currGripperMode == Mode.CUBE:
            self.currGripperMode = Mode.CONE
            self.motor_gripper.setInverted(True)
            # self.motor_gripper.set(self.currGripperPWR)

        else:
            self.currGripperMode = Mode.CUBE
            self.motor_gripper.setInverted(False)
            # self.motor_gripper.set(self.currGripperPWR)

        self.updateHUD()
    
    def updateHUD(self) -> None:
        if self.currGripperMode == Mode.CONE:
            SmartDashboard.putBoolean(Mode.CONE, True)
            SmartDashboard.putBoolean(Mode.CUBE, False)
            
        elif self.currGripperMode == Mode.CUBE:
            SmartDashboard.putBoolean(Mode.CONE, False)
            SmartDashboard.putBoolean(Mode.CUBE, True)

    def topRow(self):
        self.setSetpoint(1.6)
    
    def midRow(self):
        self.setSetpoint(1)
    
    def retract(self):
        self.setSetpoint(-0.2)
    
    def humanPlayer(self):
        self.setSetpoint(1.3)

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
    
    def spit(self):
        print("spitting")
        self.currGripperPWR = PWR.SPIT
    
    def swallow(self):
        print("swallowing")
        self.currGripperPWR = PWR.SWALLOW
    
    def stopIntake(self):
        print("no intake power")
        self.currGripperPWR = PWR.STOP
    
    def periodic(self) -> None:
        self.motor_gripper.set(self.currGripperPWR)
        super().periodic()
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
