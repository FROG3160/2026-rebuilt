from dataclasses import dataclass
import numpy as np
from typing import List, Tuple, Optional, Dict
from photonlibpy import PhotonCamera, PhotonPoseEstimator, EstimatedRobotPose
from photonlibpy.targeting import PhotonTrackedTarget
from wpimath.geometry import Transform3d
from wpilib import SmartDashboard
from wpimath.geometry import Pose3d, Pose2d
from wpiutil import Sendable, SendableBuilder
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


class DetectorTunables(Sendable):
    def __init__(self):
        super().__init__()
        self.min_confidence = 0.5
        self.cluster_radius_deg = 24.0
        self.use_weighted_centroid = True
        self.min_neighbors = 2
        self.use_area_weighting = True

    def initSendable(self, builder: SendableBuilder) -> None:
        builder.setSmartDashboardType("Detector Tunables")
        builder.addDoubleProperty(
            "Min Confidence",
            lambda: self.min_confidence,
            lambda value: setattr(self, "min_confidence", value),
        )
        builder.addDoubleProperty(
            "Cluster Radius (deg)",
            lambda: self.cluster_radius_deg,
            lambda value: setattr(self, "cluster_radius_deg", value),
        )
        builder.addBooleanProperty(
            "Use Weighted Centroid",
            lambda: self.use_weighted_centroid,
            lambda value: setattr(self, "use_weighted_centroid", value),
        )
        builder.addBooleanProperty(
            "Use Area Weighting",
            lambda: self.use_area_weighting,
            lambda value: setattr(self, "use_area_weighting", value),
        )
        builder.addDoubleProperty(
            "Min Neighbors",
            lambda: self.min_neighbors,
            lambda value: setattr(self, "min_neighbors", int(value)),
        )


@dataclass
class DetectionResult:
    """Result of detection processing"""

    target_yaw: float
    target_pitch: float
    # density_score: float  #sum of areas in cluster, weighted if enabled
    concentration_score: float  # sum of areas in cluster / concentration score
    num_detections: int  # number of targets in cluster?
    avg_confidence: float = 0.0


class FROGDetector(PhotonCamera):
    """
    Lightweight version of FROGDetection with no sklearn dependency.
    Uses only numpy for all clustering operations.
    """

    def __init__(
        self,
        camera_config: FROGCameraConfig,
    ):
        """
        Initialize the lightweight detection camera.

        Args:
            camera_name: Name of the PhotonVision camera
            use_weighted_centroid: Weight targets by area (True) or simple average (False)
            cluster_radius: Maximum distance (degrees) for targets to be in same cluster
        """
        super().__init__(camera_config.name)
        self.fuel_detector_tunables = DetectorTunables()
        SmartDashboard.putData("Fuel Detector Tunables", self.fuel_detector_tunables)
        self.targets: List[PhotonTrackedTarget] = []
        self.detection_target: Optional[DetectionResult] = None
        # self.alt_detection_target: Optional[DetectionResult] = None

    def get_targets(self):
        if results := self.getAllUnreadResults():
            result = results[-1]

            if not result.hasTargets():
                self.targets = []

            else:
                self.targets = result.getTargets()
        else:
            print("No new camera results")
            self.targets = []

        # Filter by confidence
        valid = [
            t
            for t in self.targets
            if t.objDetectConf >= self.fuel_detector_tunables.min_confidence
        ]

        # Extract to numpy for other clustering and other analysis methods
        if valid:
            self._yaws = np.fromiter((t.yaw for t in valid), dtype=float)
            self._pitches = np.fromiter((t.pitch for t in valid), dtype=float)
            self._areas = np.fromiter((t.area for t in valid), dtype=float)
        else:
            self._yaws = self._pitches = self._areas = np.array([])

    def get_detection_results(
        self,
    ) -> Optional[DetectionResult] | None:

        if self.targets:
            # self.detection_target = self._get_fuel_concentration_target()
            self.detection_target = self.get_best_cluster(
                self.targets,
                min_confidence=self.fuel_detector_tunables.min_confidence,
                cluster_radius_deg=self.fuel_detector_tunables.cluster_radius_deg,
                min_neighbors=self.fuel_detector_tunables.min_neighbors,
                use_area_weighting=self.fuel_detector_tunables.use_area_weighting,
            )
        else:
            return None
        return self.detection_target

    def get_alt_detection_results(self):
        if self.targets:
            self.detection_target = self._find_densest_ball_cluster(
                self.targets,
                radius_deg=self.fuel_detector_tunables.cluster_radius_deg,
                use_area_weighting=self.fuel_detector_tunables.use_area_weighting,
                min_neighbors=self.fuel_detector_tunables.min_neighbors,
                min_confidence=self.fuel_detector_tunables.min_confidence,
            )
        else:
            return None
        return self.detection_target

    def get_best_cluster(
        self,
        targets: List[PhotonTrackedTarget],
        min_confidence: float = 0.5,
        cluster_radius_deg: float = 12.0,
        min_neighbors: int = 2,
        use_area_weighting: bool = True,
    ) -> Optional[DetectionResult]:
        # 1. Filter by confidence
        valid = [t for t in targets if t.objDetectConf >= min_confidence]
        if not valid:
            return None

        y = np.array([t.yaw for t in valid])
        p = np.array([t.pitch for t in valid])
        a = np.array([t.area for t in valid])

        # 2. Calculate Distance Matrix (Broadcasting)
        dist_sq = (y[:, np.newaxis] - y[np.newaxis, :]) ** 2 + (
            p[:, np.newaxis] - p[np.newaxis, :]
        ) ** 2
        adj = dist_sq <= (cluster_radius_deg**2)

        # 3. Score neighborhoods based on density
        if use_area_weighting:
            scores = adj @ a
        else:
            scores = adj.sum(axis=1)

        best_idx = np.argmax(scores)

        # 4. Validate against minimum neighbors
        num_neighbors = int(adj[best_idx].sum())
        if num_neighbors < min_neighbors:
            return None

        # 5. Extract the winning cluster and calculate centroid
        cluster_mask = adj[best_idx]

        if use_area_weighting:
            weights = a[cluster_mask]
            target_yaw = np.sum(y[cluster_mask] * weights) / weights.sum()
            target_pitch = np.sum(p[cluster_mask] * weights) / weights.sum()
        else:
            target_yaw = np.mean(y[cluster_mask])
            target_pitch = np.mean(p[cluster_mask])

        return DetectionResult(
            target_yaw=float(target_yaw),
            target_pitch=float(target_pitch),
            concentration_score=float(scores[best_idx]),
            num_detections=num_neighbors,
        )

    def _get_fuel_concentration_target(self) -> Optional[DetectionResult] | None:
        """
        Get the pitch and yaw to center on the greatest concentration of fuel.

        Returns:
            DetectionResult with target angles, or None if no valid targets
        """
        if len(self.targets) == 0:
            return None

        # Extract positions
        positions = np.array([[t.getYaw(), t.getPitch()] for t in self.targets])

        # Find the largest cluster
        largest_cluster = self._find_largest_cluster(positions)
        # alt_cluster =

        if len(largest_cluster) == 0:
            return None

        # Get targets in the largest cluster
        cluster_targets = [self.targets[i] for i in largest_cluster]
        cluster_positions = positions[largest_cluster]

        # Calculate centroid
        if (
            self.fuel_detector_tunables.use_weighted_centroid
            and len(cluster_targets) > 0
        ):
            weights = np.array([t.getArea() for t in cluster_targets])
            if weights.sum() > 0:
                weights = weights / weights.sum()
                centroid = np.average(cluster_positions, axis=0, weights=weights)
            else:
                centroid = np.mean(cluster_positions, axis=0)
        else:
            centroid = np.mean(cluster_positions, axis=0)

        # Calculate concentration score
        distances = np.linalg.norm(cluster_positions - centroid, axis=1)
        avg_distance = np.mean(distances)
        concentration_score = 1.0 / (1.0 + avg_distance)

        return DetectionResult(
            target_yaw=float(centroid[0]),
            target_pitch=float(centroid[1]),
            num_detections=len(cluster_positions),
            concentration_score=float(concentration_score),
        )

    def _find_largest_cluster(self, positions: np.ndarray) -> List[int]:
        """
        Find the largest cluster using a simple distance-based algorithm.

        Returns:
            List of indices belonging to the largest cluster
        """
        n = len(positions)
        if n == 0:
            return []

        if n == 1:
            return [0]

        # Build adjacency list based on distance
        neighbors = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                distance = np.linalg.norm(positions[i] - positions[j])
                if distance <= self.fuel_detector_tunables.cluster_radius_deg:
                    neighbors[i].append(j)
                    neighbors[j].append(i)

        # Find connected components using DFS
        visited = [False] * n
        clusters = []

        def dfs(node, cluster):
            visited[node] = True
            cluster.append(node)
            for neighbor in neighbors[node]:
                if not visited[neighbor]:
                    dfs(neighbor, cluster)

        for i in range(n):
            if not visited[i]:
                cluster = []
                dfs(i, cluster)
                clusters.append(cluster)

        # Return the largest cluster
        if not clusters:
            return list(range(n))

        return max(clusters, key=len)

    def get_target_angle(self) -> Optional[Tuple[float, float]]:
        """Get just the yaw and pitch angles."""
        result = self.detection_target
        if result is None:
            return None
        return (result.target_yaw, result.target_pitch)

    def has_valid_target(self) -> bool:
        """Check if there is a valid target concentration."""
        return self.detection_target is not None

    def get_concentration_info(self) -> str:
        """Get human-readable information about the current fuel concentration."""
        result = self.detection_target

        if result is None:
            return "No fuel detected"

        return (
            f"Detected {result.num_detections} fuel objects\n"
            f"Target: Yaw={result.target_yaw:.2f}°, Pitch={result.target_pitch:.2f}°\n"
            f"Concentration score: {result.concentration_score:.3f}"
        )

    # class FROGDetectorAlt(PhotonCamera):
    #     def __init__(self, config: FROGCameraConfig):
    #         super().__init__(config.name)
    #         self.fuel_detector_tunables = DetectorTunables()
    #         SmartDashboard.putData("Fuel Detector Tunables", self.fuel_detector_tunables)

    # def get_cluster(self):
    #     result = self.getLatestResult()
    #     if result.hasTargets():
    #         cluster = self._find_densest_ball_cluster(
    #             result.getTargets(),
    #             radius_deg=self.fuel_detector_tunables.cluster_radius_deg,
    #             use_area_weighting=self.fuel_detector_tunables.use_area_weighting,
    #             min_neighbors=self.fuel_detector_tunables.min_neighbors,
    #             min_confidence=self.fuel_detector_tunables.min_confidence,
    #         )
    #         if cluster:
    #             print(
    #                 f"Best cluster: yaw={cluster['yaw_deg']:.1f}°, pitch={cluster['pitch_deg']:.1f}°"
    #             )
    #             print(
    #                 f"  → {cluster['count']} balls, density_score={cluster['density_score']:.2f}, "
    #                 f"avg conf={cluster['avg_confidence']:.2f}"
    #             )
    #         else:
    #             print("No usable ball cluster found.")
    #     else:
    #         return []

    def _find_densest_ball_cluster(
        self,
        targets: List[PhotonTrackedTarget],
        radius_deg: float = 24.0,
        use_area_weighting: bool = True,
        min_neighbors: int = 2,
        min_confidence: float = 0.50,
    ) -> Optional[DetectionResult]:
        """
        Finds the densest local cluster of detected yellow balls (or similar objects)
        and returns its center + density metrics.

        Density priority: total area-weighted sum (favors closer / bigger balls)
        Falls back to largest single target if no qualifying cluster exists.

        Args:
            targets: List of PhotonTrackedTarget detections
            radius_deg: Maximum distance (degrees) for targets to be in same cluster
            use_area_weighting: Weight targets by area (True) or simple average (False)
            min_neighbors: Minimum number of targets in cluster to be valid
            min_confidence: Minimum confidence for targets to be considered
        Returns:
            DetectionResult with target angles and density metrics, or None if no valid targets
        """
        if not targets:
            return None

        # Pre-filter low-confidence detections early
        valid_targets = [t for t in targets if t.objDetectConf >= min_confidence]
        if len(valid_targets) < min_neighbors:
            # Single fallback only if we at least have one decent detection
            if valid_targets:
                best = max(valid_targets, key=lambda t: t.area)

                return DetectionResult(
                    target_yaw=float(best.yaw),
                    target_pitch=float(best.pitch),
                    concentration_score=best.area,
                    num_detections=1,
                    avg_confidence=best.objDetectConf,
                )
            return None

        # Extract to numpy
        yaws = np.array([t.yaw for t in valid_targets])
        pitches = np.array([t.pitch for t in valid_targets])
        areas = np.array([t.area for t in valid_targets])
        confs = np.array([t.objDetectConf for t in valid_targets])

        best = {
            "yaw_deg": 0.0,
            "pitch_deg": 0.0,
            "density_score": 0.0,
            "count": 0,
            "avg_confidence": 0.0,
        }

        for i in range(len(valid_targets)):
            dy = yaws - yaws[i]
            dp = pitches - pitches[i]
            distances = np.sqrt(dy**2 + dp**2)

            mask = distances <= radius_deg
            count = np.sum(mask)

            if count < min_neighbors or count < best["count"]:
                continue

            local_areas = areas[mask]
            local_confs = confs[mask]

            if use_area_weighting and np.any(local_areas > 0):
                weights = local_areas
                total_weight = np.sum(weights)
                center_yaw = np.sum(yaws[mask] * weights) / total_weight
                center_pitch = np.sum(pitches[mask] * weights) / total_weight
                density = total_weight  # ← main density score
            else:
                center_yaw = np.mean(yaws[mask])
                center_pitch = np.mean(pitches[mask])
                density = count  # fallback unweighted

            # Update if this is better
            if density > best["density_score"] or (
                density == best["density_score"] and count > best["count"]
            ):
                best.update(
                    {
                        "yaw_deg": center_yaw,
                        "pitch_deg": center_pitch,
                        "density_score": density,
                        "count": count,
                        "avg_confidence": np.mean(local_confs),
                    }
                )

        # Final fallback if nothing beat min_neighbors
        if best["count"] == 0 and len(valid_targets) > 0:
            idx = np.argmax(areas)
            best = {
                "yaw_deg": yaws[idx],
                "pitch_deg": pitches[idx],
                "density_score": areas[idx],
                "count": 1,
                "avg_confidence": confs[idx],
            }

        # return best
        return DetectionResult(
            target_yaw=float(best["yaw_deg"]),
            target_pitch=float(best["pitch_deg"]),
            concentration_score=best["density_score"],
            num_detections=best["count"],
            avg_confidence=best["avg_confidence"],
        )
