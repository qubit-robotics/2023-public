import commands2
import commands2.cmd
import wpilib

from subsystems.armsubsystem import ArmSubsystem

class DropGamePieceAuto(commands2.SequentialCommandGroup):

    def __init__(self, arm_subsystem: ArmSubsystem):
        self.arm_subsystem = arm_subsystem

        self.extendCommand = commands2.cmd.runOnce(
            lambda: self.arm_subsystem.topRow(), [self.arm_subsystem]
        )

        self.waitForExtend = commands2.WaitCommand(1.5)

        self.spitCommand = commands2.cmd.runOnce(
            lambda: self.arm_subsystem.spit(), [self.arm_subsystem]
        )

        self.waitForSpit = commands2.WaitCommand(1)

        self.retractCommand = commands2.cmd.runOnce(
            lambda: self.arm_subsystem.retract(), [self.arm_subsystem]
        )
        
        self.waitForRetract = commands2.WaitCommand(1.5)

        super().__init__(
            self.extendCommand,
            self.waitForExtend,
            self.spitCommand,
            self.waitForSpit,
            self.retractCommand,
            self.waitForRetract
        )

