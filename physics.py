#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import wpilib.simulation
import wpilib

import wpimath.geometry
import wpimath.system
import wpimath.system.plant

import robotpy_apriltag

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel, drivetrains
from pyfrc.physics.units import units

from math import pi

from constants import DriveConstants

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):

        self.physics_controller = physics_controller

        self.motor_frontLeft = robot.container.drive_subsystem.motor_frontLeft
        self.motor_rearLeft = robot.container.drive_subsystem.motor_rearLeft
        self.motor_frontRight = robot.container.drive_subsystem.motor_frontRight
        self.motor_rearRight = robot.container.drive_subsystem.motor_rearRight

        self.motor_frontLeftEncoder = (
            robot.container.drive_subsystem.motor_frontLeftEncoder
        )
        self.motor_rearLeftEncoder = (
            robot.container.drive_subsystem.motor_rearLeftEncoder
        )
        self.motor_frontRightEncoder = (
            robot.container.drive_subsystem.motor_frontRightEncoder
        )
        self.motor_rearRightEncoder = (
            robot.container.drive_subsystem.motor_rearRightEncoder
        )

        self.gyro = wpilib.simulation.ADIS16448_IMUSim(
            robot.container.drive_subsystem.gyro
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

        self.tagz = robotpy_apriltag.loadAprilTagLayoutField(
            robotpy_apriltag.AprilTagField.k2023ChargedUp
        )

        for i in range(8):
            self.physics_controller.field.getObject(f"tag{i+1}").setPose(
                self.tagz.getTagPose(i + 1).toPose2d()
            )

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        self.drivetrain.setInputs(
            self.motor_frontLeft.get() * 12, self.motor_frontRight.get() * 12
        )
        self.drivetrain.update(tm_diff)

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