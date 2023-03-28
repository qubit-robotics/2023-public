import commands2
import commands2.cmd
import wpilib
from wpilib import SmartDashboard
import rev

class Mode:
    CUBE = "cube"
    CONE = "cone"

class PWR:
    STOP = 0
    SWALLOW = 1
    SPIT = -1

class GripperSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:

        self.motor_gripper = wpilib.PWMSparkMax(9)
        self.motor_gripper.setInverted(False)

        self.currGripperMode = Mode.CUBE
        self.currGripperPWR = PWR.STOP


        super().__init__()

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
        return super().periodic()