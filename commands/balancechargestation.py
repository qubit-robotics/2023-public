import commands2
import commands2.cmd
import wpimath.controller
import wpilib
from subsystems.drivesubsystem import DriveSubsystem

from constants import DriveConstants

class BalanceChargeStation:

    def __init__(self, drive_subsystem: DriveSubsystem):
        self.drive_subsystem = drive_subsystem

    def balancePeriodic(self):
        if self.drive_subsystem.gyro.getGyroAngleY() < -7:
            self.drive_subsystem.voltDrive(-1.5, -1.5)
        elif self.drive_subsystem.gyro.getGyroAngleY() > 7:
            self.drive_subsystem.voltDrive(1.5, 1.5)
        else:
            self.drive_subsystem.drive(0, 0)

    def getCommand(self):
        return commands2.RepeatCommand(
            commands2.cmd.run(
            lambda: self.balancePeriodic(), [self.drive_subsystem]
            )
        )
    
        