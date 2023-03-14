import wpilib
import commands2
import wpimath.geometry
from wpilib import SmartDashboard

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.armsubsystem import ArmSubsystem

class ArmVisualizer(commands2.SubsystemBase):
    """
    We don't have a complete robot yet, will have to create a dataset at some point for arm states.
    """

class InRange(commands2.SubsystemBase):

    def returnArmTranslation(self) -> wpimath.geometry.Translation2d:

        # Since we don't have a complete robot yet, we will have to write this with the robot pose
        return self.estimator.getEstimatedPosition().translation()

    def __init__(self, drive_subsystem: DriveSubsystem) -> None:
        super().__init__()

        self.estimator = drive_subsystem.estimator
        
    def periodic(self) -> None:
        # If we don't set the allience colour while testing, it will not work.
        self.allience = wpilib.DriverStation.getAlliance()

        if not (self.allience == wpilib.DriverStation.Alliance.kInvalid):
            arm_translation = self.returnArmTranslation()
            if self.allience == wpilib.DriverStation.Alliance.kBlue:
                distance_topRow = arm_translation.distance(wpimath.geometry.Translation2d(x=0.5, y=arm_translation.Y()))
                distance_bottomRow = arm_translation.distance(wpimath.geometry.Translation2d(x=0.8, y=arm_translation.Y()))
                if distance_topRow < 0.07:
                    SmartDashboard.putBoolean("TopRow", True)
                    SmartDashboard.putBoolean("BottomRow", False)
                elif distance_bottomRow < 0.07:
                    SmartDashboard.putBoolean("TopRow", False)
                    SmartDashboard.putBoolean("BottomRow", True)
                else:
                    SmartDashboard.putBoolean("TopRow", False)
                    SmartDashboard.putBoolean("BottomRow", False)

            elif self.allience == wpilib.DriverStation.Alliance.kRed:
                distance_topRow = arm_translation.distance(wpimath.geometry.Translation2d(x=16.3, y=arm_translation.Y()))
                distance_bottomRow = arm_translation.distance(wpimath.geometry.Translation2d(x=16, y=arm_translation.Y()))
                if distance_topRow < 0.07:
                    SmartDashboard.putBoolean("TopRow", True)
                    SmartDashboard.putBoolean("BottomRow", False)
                elif distance_bottomRow < 0.07:
                    SmartDashboard.putBoolean("TopRow", False)
                    SmartDashboard.putBoolean("BottomRow", True)
                else:
                    SmartDashboard.putBoolean("TopRow", False)
                    SmartDashboard.putBoolean("BottomRow", False)