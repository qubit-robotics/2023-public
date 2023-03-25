import wpimath.geometry
import wpimath.kinematics


class DriveConstants:

    kTrackWidth = 0.45
    kStartingPose = wpimath.geometry.Pose2d()

    kinematics = wpimath.kinematics.DifferentialDriveKinematics(kTrackWidth)

    #Constants related to pathplanning
    # TODO: These are example values. When we assemble the chassis, we will need to change these. Also, the simulation will automatically adapt to these values.
    ksVolts = 0.121
    kvVoltSecondsPerMeter = 2.82
    kaVoltSecondsSquaredPerMeter = 0.32
    kvVoltSecondsPerRadian = 2.89
    kaVoltSecondsSquaredPerRadian = 0.06

    kPDriveVel = 0.01

    kMaxSpeedMetersPerSecond = 2
    kMaxAccelerationMetersPerSecondSquared = 1

    kMaxVoltsRamsete = 10

    #PID Constants for auto balancing on the charge station.
    # TODO: These values should be thoroughly tested, stuff below are example values.
    kPBalance = 0.03
    kIBalance = 0
    kDBalance = 0

class SimCameraConstants:

    camDiagFOV = 67.5 #degrees
    camToRobot = wpimath.geometry.Transform3d() #camera position relative to the center of the chassis, 
    camResolutionWidth = 416 #pixels
    camResolutionHeight = 480 #pixels
    minTargetArea = 500 #pixels^2, TODO: we will tinker with this on the real robot. Don't think we need to have massive detection distance, more fps will work better.
