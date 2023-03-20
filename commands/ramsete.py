import commands2
import wpimath.controller
import wpimath.trajectory
import wpimath.trajectory.constraint
import wpimath.geometry
from wpilib import SmartDashboard

from constants import DriveConstants
from subsystems.drivesubsystem import DriveSubsystem


class PathCommand:
    def __init__(self, drive_subsystem: DriveSubsystem, path: wpimath.trajectory.Trajectory):
        self.autoVoltageConstraint = (
            wpimath.trajectory.constraint.DifferentialDriveVoltageConstraint(
                wpimath.controller.SimpleMotorFeedforwardMeters(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter,
                ),
                DriveConstants.kinematics,
                DriveConstants.kMaxVoltsRamsete,
            )
        )

        self.config = wpimath.trajectory.TrajectoryConfig(
            DriveConstants.kMaxSpeedMetersPerSecond,
            DriveConstants.kMaxAccelerationMetersPerSecondSquared,
        )

        self.config.setKinematics(DriveConstants.kinematics)
        self.config.addConstraint(self.autoVoltageConstraint)

        self.initialPosition = wpimath.geometry.Pose2d()

        self.movements = [
            wpimath.geometry.Translation2d(x=1, y=1),
            wpimath.geometry.Translation2d(x=2, y=-1),
        ]

        self.finalPosition = wpimath.geometry.Pose2d(
            wpimath.geometry.Translation2d(x=3, y=0), wpimath.geometry.Rotation2d(0)
        )

        self.exampleTrajectory = (
            wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
                self.initialPosition, self.movements, self.finalPosition, self.config
            )
        )
        self.trajectory = path

        self.ramseteCommand = commands2.RamseteCommand(
            self.exampleTrajectory,
            drive_subsystem.getEstimatedPose,
            wpimath.controller.RamseteController(b=2, zeta=0.7),
            wpimath.controller.SimpleMotorFeedforwardMeters(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter,
            ),
            DriveConstants.kinematics,
            drive_subsystem.getWheelSpeeds,
            wpimath.controller.PIDController(DriveConstants.kPDriveVel, 0, 0),
            wpimath.controller.PIDController(DriveConstants.kPDriveVel, 0, 0),
            drive_subsystem.voltDrive,
            [drive_subsystem],
        )

    def getRamseteCommand(self):
        return self.ramseteCommand
