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

        self.last_tagchoice = None
        self.last_mobilitychoice = None
    
        SmartDashboard.putData("tagchooser", self.tagchooser)
        SmartDashboard.putData("mobilitychooser", self.mobilitychooser)
    
    def generatePath(self) -> wpimath.trajectory.Trajectory:
        """
        Returns: Trajectory object that can be passed to RamseteCommand. 
        """
        self.tagchoice = self.tagchooser.getSelected()
        self.last_tagchoice = self.last_tagchoice

        self.mobilitychoice = self.mobilitychooser.getSelected()
        self.last_mobilitychoice = self.mobilitychoice

        if ((self.tagchoice != None) and (self.mobilitychoice != None)):
            return wpimath.trajectory.TrajectoryUtil.fromPathweaverJson(f"paths/output/tagid{self.tagchoice}{self.mobilitychoice}")
        
        else:
            if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
                return wpimath.trajectory.TrajectoryUtil.fromPathweaverJson("paths/output/test.wpilib.json")
            elif wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                return wpimath.trajectory.TrajectoryUtil.fromPathweaverJson("paths/output/test.wpilib.json")
            else:
                return wpimath.trajectory.Trajectory()
        
    def hasModeChanged(self) -> bool:
        """
        Returns: Whether the mode has been changed at some point after the last generatePath call.
        """
        if (self.tagchooser.getSelected() != self.last_tagchoice) or (self.mobilitychooser.getSelected() != self.last_mobilitychoice):
            return True
        else:
            return False