import photonvision
import wpimath.geometry
import robotpy_apriltag
import commands2

from constants import SimCameraConstants

class CamSubsystem(commands2.SubsystemBase):

    # Reference Pose3d for intelliSense to autocomplete
    tag = {
        "tag1": wpimath.geometry.Pose3d,
        "tag2": wpimath.geometry.Pose3d,
        "tag3": wpimath.geometry.Pose3d,
        "tag4": wpimath.geometry.Pose3d,
        "tag5": wpimath.geometry.Pose3d,
        "tag6": wpimath.geometry.Pose3d,
        "tag7": wpimath.geometry.Pose3d,
        "tag8": wpimath.geometry.Pose3d
    }

    def __init__(self):
        super().__init__()
        self.cam = photonvision.PhotonCamera("camera")

        self.tagLayout = robotpy_apriltag.loadAprilTagLayoutField(
            robotpy_apriltag.AprilTagField.k2023ChargedUp
        )

        for i in range(8):
            self.tag[f"tag{i+1}"] = self.tagLayout.getTagPose(i+1)
        
        # Python doesn't have ArrayLists, so pybind makes us provide the last param as such.
        self.photonPoseEstimator = photonvision.RobotPoseEstimator(
            self.tagLayout, photonvision.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, [(self.cam, SimCameraConstants.camToRobot)]
        )
    
    def getBestTargetTransform(self) -> int and wpimath.geometry.Transform3d:
        if self.cam.hasTargets():
            return self.cam.getLatestResult().getBestTarget().getFiducialId(), self.cam.getLatestResult().getBestTarget().getBestCameraToTarget()
    
    def getLatencyMillis(self) -> float:
        if self.cam.hasTargets():
            return self.cam.getLatestResult().getLatency()
    
    def getEstimatedGlobalPose(self, prevEstimatedGlobalPose: wpimath.geometry.Pose2d):
        self.photonPoseEstimator.setReferencePose(wpimath.geometry.Pose3d(pose=prevEstimatedGlobalPose))
        return self.photonPoseEstimator.update()