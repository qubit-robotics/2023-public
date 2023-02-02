import commands2
import commands2.button
import commands2.cmd
import wpilib
import photonvision

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.camsubsytem import CamSubsystem
from commands.ramsete import PathCommand
from commands.balancechargestation import BalanceChargeStation


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, MyRobot) -> None:

        self.cam_subsystem = CamSubsystem()
        self.drive_subsystem = DriveSubsystem(MyRobot, self.cam_subsystem)

        self.pathCommand = PathCommand(self.drive_subsystem)
        self.balanceCommand = BalanceChargeStation(self.drive_subsystem)

        self.driver_controller = commands2.button.CommandJoystick(0)

        self.drive_subsystem.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.drive_subsystem.drive(
                    self.driver_controller.getX(), self.driver_controller.getZ()
                ),
                [self.drive_subsystem],
            ),
        )

        self.configureButtonBindings()

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        self.driver_controller.button(1).whileTrue(self.balanceCommand)

    def getAutonomousCommand(self) -> commands2.Command:
        return self.pathCommand.getRamseteCommand()
