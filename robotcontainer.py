import wpilib

import commands2
import commands2.button
import commands2.cmd

from subsystems.drivesubsystem import DriveSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.drive_subsystem = DriveSubsystem()

        self.driver_controller = commands2.button.CommandJoystick(0)

        self.drive_subsystem.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.drive_subsystem.drive(
                    self.driver_controller.getX(),
                    self.driver_controller.getZ()
                ),
                [self.drive_subsystem]
            )
        )

        self.configureButtonBindings()

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        pass

    def getAutonomousCommand(self) -> commands2.Command:
        return None
