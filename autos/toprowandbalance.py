import commands2
import commands2.cmd

from hud.autonchooser import AutonChooser

from subsystems.drivesubsystem import DriveSubsystem

from commands.ramsete import PathCommand
from commands.balancechargestation import BalanceChargeStation

class TopRowAndBalance(commands2.SequentialCommandGroup):

    def __init__(self, drive_subsystem: DriveSubsystem, auton_chooser: AutonChooser):
        super().__init__()
        self.addCommands(
            [
            #TODO: Drop a single game piece command
            PathCommand(drive_subsystem, auton_chooser.generatePath()).getRamseteCommand(),
            BalanceChargeStation(drive_subsystem)
            ]
        )