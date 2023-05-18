import wpilib
from wpilib.drive import DifferentialDrive
import commands2
import subsystem.drive
from container import RobotContainer
class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.container = RobotContainer(self)
        self.drive = self.container.drive_subsystem
        self.autonomousCommand = self.container.getAutonomousCommand()
    def autonomousInit(self):
        self.container.drive_subsystem.setStartingPose(self.container.auton_chooser)

        

        if self.autonomousCommand:
            print("valid autonomous command")
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous"""
        print(self.container.drive_subsystem.getWheelSpeeds())
    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
    def teleopPeriodic(self):
        self.container.balanceCommand.periodic()
if __name__ == '__main__':
    wpilib.run(MyRobot)    
