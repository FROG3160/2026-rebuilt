from photonlibpy import PhotonCamera, PhotonPoseEstimator, EstimatedRobotPose
from wpimath.geometry import Transform3d
from wpilib import SmartDashboard
from wpimath.geometry import Pose3d, Pose2d
from FROGlib.utils import PoseBuffer
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance


class FROGCameraConfig:
    def __init__(self, name: str, robotToCamera: Transform3d = Transform3d()):
        self.name = name
        self.robotToCamera = robotToCamera


class FROGPoseEstimator:
    def __init__(
        self,
        fieldTags: AprilTagFieldLayout,
        camera_name: str,
        robotToCamera: Transform3d = Transform3d(),
    ):
        self.camera = PhotonCamera(camera_name)
        self.fieldTags = fieldTags
        self.estimator = PhotonPoseEstimator(
            fieldTags,
            robotToCamera,
        )

        nt_table = f"Subsystems/Vision/{camera_name}"
        self.pose_buffer = PoseBuffer(25)
        self._latest_pose_pub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/estimated_pose", Pose2d)
            .publish()
        )

        self._stdev_x_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_x")
            .publish()
        )
        self._stdev_y_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_y")
            .publish()
        )
        self._stdev_rotation_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_rotation")
            .publish()
        )
        self._has_targets_pub = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/has_targets")
            .publish()
        )
        self._tag_id_pub = (
            NetworkTableInstance.getDefault()
            .getIntegerTopic(f"{nt_table}/target_id")
            .publish()
        )
        self._target_ambiguity_rotation_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/target_ambiguity")
            .publish()
        )

    def get_estimate(self) -> EstimatedRobotPose | None:
        estPose = None
        for result in self.camera.getAllUnreadResults():
            estPose = self.estimator.estimateCoprocMultiTagPose(result)
            if estPose is None:
                estPose = self.estimator.estimateLowestAmbiguityPose(result)

        # if we have an estimated Pose, publish it, otherwise publish a bad Pose2d
        if estPose:
            self._latest_pose_pub.set(estPose.estimatedPose.toPose2d())
        else:
            self._latest_pose_pub.set(Pose2d(-1, -1, 0))

        return estPose

        # estimated_pose = self.update()
        # if estimated_pose:
        #     targets = estimated_pose.targetsUsed
        #     SmartDashboard.putNumber(
        #         f"{self._camera.getName()} Targets Found", len(targets)
        #     )
        #     target1 = targets[0]
        #     SmartDashboard.putNumber(
        #         f"{self._camera.getName()} First Target Ambiguity",
        #         target1.getPoseAmbiguity(),
        #     )

    def get_distance_to_tag(self, pose: Pose3d, tag_num: int) -> float | None:
        if tagpose := self.getTagPose(tag_num):
            return (
                pose.toPose2d().translation().distance(tagpose.toPose2d().translation())
            )
        else:
            return None

    def getTagPose(self, tag_id: int):
        return self.fieldTags.getTagPose(tag_id)

    # def periodic(self):

    #     self.latestVisionPose = self.update()
    #     result = self._camera.getLatestResult()
    #     if result.hasTargets():
    #         target = result.getBestTarget()
    #         ambiguity = target.getPoseAmbiguity()
    #         tag_id = target.getFiducialId()
    #         self._has_targets_pub.set(result.hasTargets())
    #         self._tag_id_pub.set(tag_id)
    #         self._target_ambiguity_rotation_pub.set(ambiguity)

    #     if self.latestVisionPose:
    #         self.pose_buffer.append(self.latestVisionPose.estimatedPose.toPose2d())
    #         # target_details = []
    #         # for photon_target in self.latestVisionPose.targetsUsed():
    #         #     target_details.append(
    #         #         {
    #         #             "tag": photon_target.fiducialId,
    #         #             "ambiguity": photon_target.poseAmbiguity,
    #         #             "transform": photon_target.bestCameraToTarget,
    #         #             "distance": self.get_distance_to_tag(
    #         #                 self.latestVisionPose.estimatedPose.toPose2d(),
    #         #                 photon_target.fiducialId,
    #         #             ),
    #         #         }
    #         #     )

    #         self._latest_pose_pub.set(self.latestVisionPose.estimatedPose.toPose2d())
    #         self._stdev_x_pub.set(self.pose_buffer.x_stddev())
    #         self._stdev_y_pub.set(self.pose_buffer.y_stddev())
    #         self._stdev_rotation_pub.set(self.pose_buffer.rotation_stddev())
    #         return self.latestVisionPose
