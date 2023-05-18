import wpilib.simulation
import wpilib
import wpimath.geometry
import wpimath.system
import wpimath.system.plant
import wpimath.trajectory
import robotpy_apriltag
import photonvision
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel, drivetrains
from pyfrc.physics.units import units
from math import pi
from contants import DriveConstants
import typing
import subsystem.drive 
if typing.TYPE_CHECKING:
    from robot import MyRobot

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.robot = robot
        self.physics_controller = physics_controller
        self.drive = robot.drive
        self.motor_frontLeft = self.drive.motor_frontLeft
        self.motor_rearLeft = self.drive.motor_rearLeft
        self.motor_frontRight = self.drive.motor_frontRight
        self.motor_rearRight = self.drive.motor_rearRight
        self.motor_frontLeftEncoder = (
            self.drive.motor_frontLeftEncoder
        )
        self.motor_rearLeftEncoder = (
            self.drive.motor_rearLeftEncoder
        )
        self.motor_frontRightEncoder = (
            self.drive.motor_frontRightEncoder
        )
        self.motor_rearRightEncoder = (
            self.drive.motor_rearRightEncoder
        )

        self.gyro = wpilib.simulation.ADIS16448_IMUSim(
            self.drive.gyro
        )
        self.system = wpimath.system.plant.LinearSystemId.identifyDrivetrainSystem(
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter,
            DriveConstants.kvVoltSecondsPerRadian,
            DriveConstants.kaVoltSecondsSquaredPerRadian,
        )
        self.drivetrain = wpilib.simulation.DifferentialDrivetrainSim(
            self.system, 0.7, wpimath.system.plant.DCMotor.NEO(2), 10.71, 0.07
        )
    def update_sim(self, now: float, tm_diff: float) -> None:

        self.drivetrain.setInputs(
                self.motor_frontLeft.get() * 12, self.motor_frontRight.get() * 12
            )
        self.drivetrain.update(tm_diff)

        self.physics_controller.field.setRobotPose(self.drivetrain.getPose())
        self.physics_controller.field.setRobotPose(self.drivetrain.getPose())

        self.motor_frontLeftEncoder.setPosition(self.drivetrain.getLeftPosition())
        self.motor_rearLeftEncoder.setPosition(self.drivetrain.getLeftPosition())
        self.motor_frontRightEncoder.setPosition(self.drivetrain.getRightPosition())
        self.motor_rearRightEncoder.setPosition(self.drivetrain.getRightPosition())

        self.motor_frontLeft.setSimVelocity(self.drivetrain.getLeftVelocity())
        self.motor_rearLeft.setSimVelocity(self.drivetrain.getLeftVelocity())
        self.motor_frontRight.setSimVelocity(self.drivetrain.getRightVelocity())
        self.motor_rearRight.setSimVelocity(self.drivetrain.getRightVelocity())

        self.gyro.setGyroAngleZ(-self.drivetrain.getHeading().degrees())