import commands2
import wpimath.controller
import wpilib
from subsystems.drivesubsystem import DriveSubsystem

from constants import DriveConstants

class BalanceChargeStation(commands2.CommandBase):

    def __init__(self, drive_subsystem: DriveSubsystem):
        super().__init__()
        super().addRequirements([drive_subsystem])

        self.drive_subsystem = drive_subsystem

        self.gyro = self.drive_subsystem.gyro
    
    def execute(self) -> None:
        print(self.gyro.getGyroAngleY())
        if self.gyro.getGyroAngleY() > 5:
            self.drive_subsystem.drive(1, 0)
        elif self.gyro.getGyroAngleY() < -5:
            self.drive_subsystem.drive(-1, 0)
        else:
            self.end(True)
        super().execute()
        