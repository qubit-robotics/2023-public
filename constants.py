import wpimath.geometry
import wpimath.kinematics


class DriveConstants:

    kTrackWidth = 0.7
    kStartingPose = wpimath.geometry.Pose2d()

    kinematics = wpimath.kinematics.DifferentialDriveKinematics(kTrackWidth)

    # TODO: These are example values. When we assemble the chassis, we will need to change these. Also, the simulation will automatically adapt to these values.
    ksVolts = 0.22
    kvVoltSecondsPerMeter = 1.98
    kaVoltSecondsSquaredPerMeter = 0.2
    kvVoltSecondsPerRadian = 1.5
    kaVoltSecondsSquaredPerRadian = 0.3

    kPDriveVel = 8.5

    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3

    kMaxVoltsRamsete = 10
