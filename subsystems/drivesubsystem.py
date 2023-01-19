import wpilib
import wpilib.drive
import wpimath.estimator
import wpimath.geometry
import wpimath.kinematics
import commands2
import rev

from constants import DriveConstants


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

        self.motor_frontLeftEncoder = self.motor_frontLeft.getEncoder()
        self.motor_rearLeftEncoder = self.motor_rearLeft.getEncoder()
        self.motor_frontRightEncoder = self.motor_frontRight.getEncoder()
        self.motor_rearRightEncoder = self.motor_rearRight.getEncoder()

        self.gyro = wpilib.ADIS16448_IMU()
        self.gyro.calibrate()
        self.gyro.reset()

        self.kinematics = wpimath.kinematics.DifferentialDriveKinematics(
            DriveConstants.kTrackWidth
        )
        self.estimator = wpimath.estimator.DifferentialDrivePoseEstimator(
            self.kinematics,
            wpimath.geometry.Rotation2d.fromDegrees(-self.gyro.getAngle()),
            self.getLeftGroupDistance(),
            self.getRightGroupDistance(),
            DriveConstants.kStartingPose,
        )
        self.lastPose = DriveConstants.kStartingPose

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

    def resetEncoders(self):
        self.motor_frontLeftEncoder.setPosition(0)
        self.motor_rearLeftEncoder.setPosition(0)
        self.motor_frontRightEncoder.setPosition(0)
        self.motor_rearRightEncoder.setPosition(0)

    def getLeftGroupDistance(self):
        return (
            self.motor_frontLeftEncoder.getPosition()
            + self.motor_rearLeftEncoder.getPosition()
        ) / 2

    def getRightGroupDistance(self):
        return (
            self.motor_frontRightEncoder.getPosition()
            + self.motor_rearRightEncoder.getPosition()
        ) / 2

    def updateEstimator(self):
        self.lastPose = self.estimator.update(
            wpimath.geometry.Rotation2d.fromDegrees(-self.gyro.getAngle()),
            self.getLeftGroupDistance(),
            self.getRightGroupDistance(),
        )

    def getEstimatedPose(self):
        """
        self.estimator.getEstimatedPosition() doesn't work, this is a crappy workaround.
        """
        return self.lastPose

    def defCommand(self, x, z):
        self.drive(x, z)
        self.updateEstimator()
