import wpilib
import wpilib.drive
import wpimath.estimator
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
import wpimath.filter
import commands2
import rev
import qlib.qsparkmax

from subsystems.camsubsytem import CamSubsystem
from hud.autonchooser import AutonChooser

from constants import DriveConstants


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self, MyRobot: commands2.TimedCommandRobot, cam_subsystem: CamSubsystem) -> None:
        super().__init__()

        self.MyRobot = MyRobot
        self.cam_subsystem = cam_subsystem

        self.timer = wpilib.Timer()
        self.timer.start()

        self.motor_frontLeft = qlib.qsparkmax.Qubit_CANSparkMax(
            1, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_rearLeft = qlib.qsparkmax.Qubit_CANSparkMax(
            2, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_frontRight = qlib.qsparkmax.Qubit_CANSparkMax(
            3, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_rearRight = qlib.qsparkmax.Qubit_CANSparkMax(
            4, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )

        self.slewRateLimiterX = wpimath.filter.SlewRateLimiter(1)
        self.slewRateLimiterZ = wpimath.filter.SlewRateLimiter(1)

        self.motor_frontLeftEncoder = self.motor_frontLeft.getEncoder()
        self.motor_rearLeftEncoder = self.motor_rearLeft.getEncoder()
        self.motor_frontRightEncoder = self.motor_frontRight.getEncoder()
        self.motor_rearRightEncoder = self.motor_rearRight.getEncoder()

        self.motor_frontLeftEncoder.setPositionConversionFactor(0.0446812324929972)
        self.motor_frontLeftEncoder.setVelocityConversionFactor(0.0446812324929972)
        self.motor_rearLeftEncoder.setPositionConversionFactor(0.0446812324929972)
        self.motor_rearLeftEncoder.setVelocityConversionFactor(0.0446812324929972)
        self.motor_frontRightEncoder.setPositionConversionFactor(0.0446812324929972)
        self.motor_frontRightEncoder.setVelocityConversionFactor(0.0446812324929972)
        self.motor_rearRightEncoder.setPositionConversionFactor(0.0446812324929972)
        self.motor_rearRightEncoder.setVelocityConversionFactor(0.0446812324929972)


        self.gyro = wpilib.ADIS16448_IMU()
        self.gyro.calibrate()
        self.gyro.reset()

        self.kinematics = DriveConstants.kinematics
        self.estimator = wpimath.estimator.DifferentialDrivePoseEstimator(
            self.kinematics,
            wpimath.geometry.Rotation2d.fromDegrees(-self.gyro.getAngle()),
            self.getLeftGroupDistance(),
            self.getRightGroupDistance(),
            DriveConstants.kStartingPose,
        )
        self.lastPose = DriveConstants.kStartingPose
        self.last_camEstimatedPose = wpimath.geometry.Pose3d()

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("estimatorField", self.field)

        self.motor_leftGroup = wpilib.MotorControllerGroup(
            self.motor_frontLeft, self.motor_rearLeft
        )

        self.motor_rightGroup = wpilib.MotorControllerGroup(
            self.motor_frontRight, self.motor_rearRight
        )

        # self.motor_leftGroup.setInverted(True)
        # self.motor_rightGroup.setInverted(True)

        self.drivetrain = wpilib.drive.DifferentialDrive(
            self.motor_leftGroup, self.motor_rightGroup
        )

    def drive(self, x, z, noSlew= False):
        if noSlew:
            self.drivetrain.arcadeDrive(
                x / 2, z / 2
            )
        else:
            self.drivetrain.arcadeDrive(
                self.slewRateLimiterX.calculate(x), self.slewRateLimiterZ.calculate(z)
            )

    def voltDrive(self, leftVolts, rightVolts):
        self.motor_leftGroup.setVoltage(leftVolts),
        self.motor_rightGroup.setVoltage(rightVolts)
        self.drivetrain.feed()

    def resetEncodersAndGyro(self):
        self.motor_frontLeftEncoder.setPosition(0)
        self.motor_rearLeftEncoder.setPosition(0)
        self.motor_frontRightEncoder.setPosition(0)
        self.motor_rearRightEncoder.setPosition(0)

        self.gyro.reset()

    def getLeftGroupDistance(self):
        return (
            self.motor_frontLeftEncoder.getPosition() + self.motor_rearLeftEncoder.getPosition()
        ) / 2

    def getRightGroupDistance(self):
        return (
            self.motor_frontRightEncoder.getPosition() + self.motor_rearRightEncoder.getPosition()
        ) / 2

    def updateEstimator(self):
        self.lastPose = self.estimator.update(
            wpimath.geometry.Rotation2d.fromDegrees(-self.gyro.getAngle()),
            self.getLeftGroupDistance(),
            self.getRightGroupDistance(),
        )

        camEstimatedPose, timestampLatencySeconds = self.cam_subsystem.getEstimatedGlobalPose(self.lastPose)

        if self.last_camEstimatedPose != camEstimatedPose:
            self.estimator.addVisionMeasurement(
                camEstimatedPose.toPose2d(), self.timer.getFPGATimestamp() - timestampLatencySeconds
            )
            self.last_camEstimatedPose = camEstimatedPose

        self.field.setRobotPose(self.estimator.getEstimatedPosition())

    def getEstimatedPose(self):
        """
        self.estimator.getEstimatedPosition() doesn't work, this is a crappy workaround.
        """
        return self.lastPose

    def getWheelSpeeds(self):
        if self.MyRobot.isSimulation():
            speeds = wpimath.kinematics.DifferentialDriveWheelSpeeds(
                (self.motor_frontLeft.getSimVelocity() + self.motor_rearLeft.getSimVelocity()) / 2,
                (self.motor_frontRight.getSimVelocity() + self.motor_rearRight.getSimVelocity()) / 2,
            )
        else:
            speeds = wpimath.kinematics.DifferentialDriveWheelSpeeds(
                (self.motor_frontLeftEncoder.getVelocity() + self.motor_rearLeftEncoder.getVelocity()) / 2,
                (self.motor_frontRightEncoder.getVelocity() + self.motor_rearRightEncoder.getVelocity()) / 2,
            )
        return speeds

    def setStartingPose(self, auton_chooser: AutonChooser):
        """
        Set the starting position through the Shuffleboard at the start of the match.

        @param1: AutonChooser HUD element
        """
        trajectory = auton_chooser.generatePath()
        
        if trajectory != wpimath.trajectory.Trajectory():
            trajectory_initial= trajectory.initialPose()
            self.estimator.resetPosition(
                wpimath.geometry.Rotation2d.fromDegrees(-self.gyro.getAngle()),
                0,
                0,
                trajectory_initial
            )

    def periodic(self) -> None:
        """
        wpilib calls this periodically, using this is much better than the previous method
        """
        self.updateEstimator()
