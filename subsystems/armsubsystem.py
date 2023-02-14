import commands2
import ctre
import enum
from wpilib import SmartDashboard

from ctre import ControlMode

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

        self.motor_gripper = ctre.WPI_VictorSPX(5)

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