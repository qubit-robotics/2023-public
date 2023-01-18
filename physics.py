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

import robotpy_apriltag

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel, drivetrains
from pyfrc.physics.units import units

from math import pi

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
        
        self.motor_FrontLeft = robot.container.drive_subsystem.motor_FrontLeft
        self.motor_RearLeft = robot.container.drive_subsystem.motor_RearLeft
        self.motor_FrontRight = robot.container.drive_subsystem.motor_FrontRight
        self.motor_RearRight = robot.container.drive_subsystem.motor_RearRight

        self.drivetrain = drivetrains.FourMotorDrivetrain()

        self.tagz = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField.k2023ChargedUp)
        
        for i in range(8):
            self.physics_controller.field.getObject(f"tag{i+1}").setPose(self.tagz.getTagPose(i+1).toPose2d())
        
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        speeds = self.drivetrain.calculate(
            self.motor_FrontLeft.get(),
            self.motor_RearLeft.get(),
            self.motor_FrontRight.get(),
            self.motor_RearRight.get()
        )

        self.physics_controller.drive(speeds, tm_diff)