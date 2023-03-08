import commands2
import wpilib
from wpilib import SmartDashboard
import wpimath.trajectory

class AutonChooser(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        self.tagchooser = wpilib.SendableChooser()
        self.tagchooser.addOption("1", 1)
        self.tagchooser.addOption("2", 2)
        self.tagchooser.addOption("3", 3)
        self.tagchooser.addOption("6", 6)
        self.tagchooser.addOption("7", 7)
        self.tagchooser.addOption("8", 8)

        self.mobilitychooser = wpilib.SendableChooser()
        self.mobilitychooser.addOption("No", "_no_mobility.wpilib.json")
        self.mobilitychooser.addOption("YES!", "_mobility.wpilib.json")

        self.tagchooser.setDefaultOption("None Selected", None)
        self.mobilitychooser.setDefaultOption("None Selected", None)
    
        SmartDashboard.putData("tagchooser", self.tagchooser)
        SmartDashboard.putData("mobilitychooser", self.mobilitychooser)
    
    def generatePath(self) -> wpimath.trajectory.Trajectory:
        """
        Returns: Trajectory object that can be passed to RamseteCommand. 
        """
        tagchoice = self.tagchooser.getSelected()
        mobilitychoice = self.mobilitychooser.getSelected()

        if ((tagchoice != None) and (mobilitychoice != None)):
            return wpimath.trajectory.TrajectoryUtil.fromPathweaverJson(f"paths/output/tagid{tagchoice}{mobilitychoice}")
        
        else:
            #TODO: make a failsafe path
            return wpimath.trajectory.Trajectory()

