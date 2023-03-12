import commands2
import ctre
import enum
from wpilib import SmartDashboard
from ctre import ControlMode
import qlib.qsparkmax
import rev 
import wpimath.controller
import wpilib
import math

class Mode(enum.auto):
    CUBE = "cube"
    CONE = "cone"

class PWR(enum.auto):
    HOLD = 0.2
    SWALLOW = 0.5
    SPIT = -0.3

class ArmSubsystem(commands2.SubsystemBase):

    def __init__(self):
        super().__init__()
        self.arm_position = 0

        self.armKp = 0
        self.armKi = 0
        self.armKd = 0
        
        self.ArmEncoderDistPerPulse = 2.0 * math.pi / 4096.0

        self.armEncoderAChannel = 0
        self.armEncoderBChannel = 1
        
        self.motor_gripper = ctre.WPI_VictorSPX(5)
        self.motor_Arm = qlib.qsparkmax.Qubit_CANSparkMax(5 ,rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        
        self.armEncoder = wpilib.Encoder(self.armEncoderAChannel, self.armEncoderBChannel)
        self.armController = wpimath.controller.PIDController(self.armKp,self.armKi,self.armKd)
        self.armEncoder.setDistancePerPulse(self.ArmEncoderDistPerPulse)
        
        # Start with the cube as default mode
        #TODO: Change this to incorperate SendableChooser
        
        self.curr_mode = Mode.CUBE

    def changeMode(self):
        if self.curr_mode == Mode.CUBE:
            self.curr_mode = Mode.CONE
            self.motor_gripper.setInverted(True)
        else:
            self.curr_mode = Mode.CUBE
            self.motor_gripper.setInverted(False)
        self.updateHUD()
    
    def updateHUD(self) -> None:
        if self.curr_mode == Mode.CONE:
            SmartDashboard.putBoolean(Mode.CONE, True)
            SmartDashboard.putBoolean(Mode.CUBE, False)
        elif self.curr_mode == Mode.CUBE:
            SmartDashboard.putBoolean(Mode.CONE, False)
            SmartDashboard.putBoolean(Mode.CUBE, True)

    # This isn't the place for these, for experimentation purposes only!
    # TODO: Move these to commands.
    def hold(self):
        self.motor_gripper.set(PWR.HOLD)
    
    def spit(self):
        self.motor_gripper.set(PWR.SPIT)
    
    def swallow(self):
        self.motor_gripper.set(PWR.SWALLOW)
    
    def setarmposition(self):
        pidOutput = self.armController.calculate(self.armEncoder.getDistance(), math.radians(self.arm_position))
        self.motor_Arm.setVoltage(pidOutput)
