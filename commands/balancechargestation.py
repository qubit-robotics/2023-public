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

        wpilib.Preferences.initFloat("kP", 0)
        wpilib.Preferences.initFloat("kI", 0)
        wpilib.Preferences.initFloat("kD", 0)

    def atSetpoint(self) -> bool:
        return self.getController().atSetpoint()

    def periodic(self):

        if wpilib.Preferences.getFloat("kP", 0) != self.getController().getP():
            self._controller.setP(wpilib.Preferences.getFloat("kP", 0))
        
        if wpilib.Preferences.getFloat("kI", 0) != self.getController().getI():
            self._controller.setI(wpilib.Preferences.getFloat("kI", 0))

        if wpilib.Preferences.getFloat("kD", 0) != self.getController().getD():
            self._controller.setD(wpilib.Preferences.getFloat("kD", 0))

        print("output", self.getController().calculate(self.drive_subsystem.gyro.getGyroAngleY()))           