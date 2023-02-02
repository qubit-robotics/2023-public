import commands2
import wpimath.controller
import wpilib
from subsystems.drivesubsystem import DriveSubsystem

from constants import DriveConstants

class BalanceChargeStation(commands2.PIDCommand):

    def __init__(self, drive_subsystem: DriveSubsystem):
        self.drive_subsystem = drive_subsystem
        super().__init__(
            wpimath.controller.PIDController(
                DriveConstants.kPBalance, DriveConstants.kIBalance, DriveConstants.kDBalance
            ),
            lambda: drive_subsystem.gyro.getGyroAngleY(),
            0,
            lambda volts: self.drive_subsystem.voltDrive(volts, volts),
            [drive_subsystem]
        )
        self.getController().setTolerance(5)
        wpilib.SmartDashboard.putData("BalanceChargeStationPID", self.getController())

    def atSetpoint(self) -> bool:
        return self.getController().atSetpoint()