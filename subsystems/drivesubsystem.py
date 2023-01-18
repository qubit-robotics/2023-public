import wpilib
import wpilib.drive
import commands2
import rev


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.motor_frontLeft = rev.CANSparkMax(
            1, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_rearLeft = rev.CANSparkMax(
            2, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_frontRight = rev.CANSparkMax(
            3, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_rearRight = rev.CANSparkMax(
            4, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )

        self.motor_leftGroup = wpilib.MotorControllerGroup(
            self.motor_frontLeft, self.motor_rearLeft
        )

        self.motor_rightGroup = wpilib.MotorControllerGroup(
            self.motor_frontRight, self.motor_rearRight
        )

        self.motor_rightGroup.setInverted(True)

        self.drivetrain = wpilib.drive.DifferentialDrive(
            self.motor_leftGroup, self.motor_rightGroup
        )

    def drive(self, x, z):
        self.drivetrain.arcadeDrive(x, z)
