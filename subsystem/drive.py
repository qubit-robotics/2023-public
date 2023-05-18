from wpilib.drive import DifferentialDrive
import wpilib
import wpimath.estimator, wpimath.geometry, wpimath.kinematics
import wpimath
import commands2
from rev import CANSparkMax
import rev
from contants import DriveConstants
import commands2
import qsparkmax
from subsystem.cam import CamSubsystem
from hud.autonchooser import AutonChooser
class drive(commands2.SubsystemBase):
    def __init__(self, MyRobot: commands2.TimedCommandRobot) -> None:
        super().__init__()
        self.Robot = MyRobot
        self.motor_frontLeft = qsparkmax.Qubit_CANSparkMax(
            1, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_rearLeft = qsparkmax.Qubit_CANSparkMax(
            2, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_frontRight = qsparkmax.Qubit_CANSparkMax(
            3, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.motor_rearRight = qsparkmax.Qubit_CANSparkMax(
            4, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )

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
            (self.motor_frontLeftEncoder.getPosition()+self.motor_rearLeftEncoder.getPosition())/2,
            (self.motor_frontRightEncoder.getPosition()+self.motor_rearRightEncoder.getPosition())/2,
            DriveConstants.kStartingPose,)
        
        self.lastPose = DriveConstants.kStartingPose
        self.last_camEstimatedPose = wpimath.geometry.Pose3d()

        self.field = wpilib.Field2d()
        self.left_group = wpilib.MotorControllerGroup(self.motor_frontLeft, self.motor_rearLeft)
        self.right_group = wpilib.MotorControllerGroup(self.motor_frontRight, self.motor_rearRight)
        
        self.drive = DifferentialDrive(self.left_group, self.right_group)
        
    def calibrate(self):
        self.motor_frontLeftEncoder.setPosition(0)
        self.motor_frontRightEncoder.setPosition(0)
        self.motor_rearLeftEncoder.setPosition(0)
        self.motor_rearRightEncoder.setPosition(0)
        self.gyro.reset()

    def getWheelSpeeds(self):
        if self.Robot.isSimulation():
            speeds = wpimath.kinematics.DifferentialDriveWheelSpeeds(
                (self.motor_frontLeft.getSimVelocity() + self.motor_rearLeft.getSimVelocity()) / 2,
                (self.motor_frontRight.getSimVelocity() + self.motor_rearRight.getSimVelocity()) / 2,)
        else:
            speeds = wpimath.kinematics.DifferentialDriveWheelSpeeds(
                (self.motor_frontLeftEncoder.getVelocity() + self.motor_rearLeftEncoder.getVelocity()) / 2,
                (self.motor_frontRightEncoder.getVelocity() + self.motor_rearRightEncoder.getVelocity()) / 2,)
        return speeds
    
    def updateEstimator(self):
        self.lastPose = self.estimator.update(
            wpimath.geometry.Rotation2d.fromDegrees(-self.gyro.getAngle()),
            (self.motor_frontLeftEncoder.getPosition()+self.motor_rearLeftEncoder.getPosition())/2,
            (self.motor_frontRightEncoder.getPosition()+self.motor_rearRightEncoder.getPosition())/2,)
        self.field.setRobotPose(self.estimator.getEstimatedPosition())

    def getEstimatedPose(self):
        return self.lastPose
    def voltDrive(self, leftVolts, rightVolts):
        self.left_group.setVoltage(leftVolts),
        self.right_group.setVoltage(rightVolts)
        self.drive.feed()
    def periodic(self) -> None:
        self.updateEstimator()
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