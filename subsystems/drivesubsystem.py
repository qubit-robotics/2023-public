import wpilib
import wpilib.drive
import commands2
import rev

class DriveSubsystem(commands2.SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        self.motor_FrontLeft = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor_RearLeft = rev.CANSparkMax(2, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor_FrontRight = rev.CANSparkMax(3, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.motor_RearRight = rev.CANSparkMax(4, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.motor_LeftGroup = wpilib.MotorControllerGroup(
            self.motor_FrontLeft,
            self.motor_RearLeft
        )

        self.motor_RightGroup = wpilib.MotorControllerGroup(
            self.motor_FrontRight,
            self.motor_RearRight
        )
        
        self.motor_RightGroup.setInverted(True)

        self.drivetrain = wpilib.drive.DifferentialDrive(
            self.motor_LeftGroup,
            self.motor_RightGroup
        )
    
    def drive(self, x, z):
        self.drivetrain.arcadeDrive(
            x, z
        )






