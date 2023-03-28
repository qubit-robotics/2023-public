import commands2
import commands2.cmd
import wpilib

from subsystems.armsubsystem import ArmSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.grippersubsystem import GripperSubsystem

class DropGamePieceAuto(commands2.SequentialCommandGroup):

    def __init__(self, arm_subsystem: ArmSubsystem, drive_subsystem: DriveSubsystem, gripper_subsystem: GripperSubsystem):
        self.arm_subsystem = arm_subsystem
        self.drive_subsystem = drive_subsystem
        self.gripper_subsytem = gripper_subsystem

        self.resetEverythingCommand = commands2.cmd.runOnce(
            lambda: self.drive_subsystem.resetEncodersAndGyro(), [self.drive_subsystem]
        )

        self.extendCommand = commands2.cmd.runOnce(
            lambda: self.arm_subsystem.topRow(), [self.arm_subsystem]
        )

        self.waitForExtend = commands2.WaitCommand(3)

        self.spitCommand = commands2.cmd.runOnce(
            lambda: self.gripper_subsytem.spit(), [self.gripper_subsytem]
        )

        self.waitForSpit = commands2.WaitCommand(5)

        self.backupCommand = commands2.cmd.run(lambda: self.drive_subsystem.drive(-0.5, 0), [self.drive_subsystem])

        self.waitForBackup = commands2.WaitCommand(3).andThen(
            commands2.cmd.run(
                lambda: self.drive_subsystem.drive(0, 0)
            )
        )

        self.stopIntakeCommand = commands2.cmd.runOnce(
            lambda: self.gripper_subsytem.stopIntake(), [self.gripper_subsytem]
        )

        self.retractCommand = commands2.cmd.runOnce(
            lambda: self.arm_subsystem.retract(), [self.arm_subsystem]
        )
        
        self.waitForRetract = commands2.WaitCommand(1)

        super().__init__(
            self.extendCommand,
            self.waitForExtend,
            self.spitCommand,
            self.waitForSpit,
            self.stopIntakeCommand,
            self.retractCommand,
            self.waitForRetract,
            self.backupCommand,
            self.waitForBackup,
            self.resetEverythingCommand
        )

