import commands2
import commands2.cmd

from hud.autonchooser import AutonChooser

from subsystem.drive import drive as DriveSubsystem
from subsystem.arm import ARM as ArmSubsystem

from commands.ramsete import PathCommand
from commands.balancechargestation import BalanceChargeStation
from commands.dropgamepieceauto import DropGamePieceAuto

class TopRowAndBalance(commands2.SequentialCommandGroup):

    def __init__(self, drive_subsystem: DriveSubsystem, arm_subsystem: ArmSubsystem, auton_chooser: AutonChooser):
        super().__init__()
        self.addCommands(
            [
            DropGamePieceAuto(arm_subsystem),
            PathCommand(drive_subsystem, auton_chooser.generatePath()).getRamseteCommand(),
            BalanceChargeStation(drive_subsystem)
            ]
        )