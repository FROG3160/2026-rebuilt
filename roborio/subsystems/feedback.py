import wpilib
from wpilib import DriverStation, Timer
from FROGlib.subsystem import FROGSubsystem


class ShiftTracker(FROGSubsystem):
    """
    Helper class to easily answer:
      - Is our HUB currently active?
      - How much time is left in the current shift?
    """

    def __init__(self):
        super().__init__()
        self.game_data = None  # 'R' or 'B'
        self.our_alliance = None  # Alliance.RED / BLUE / None
        self.first_inactive_alliance = None

        # Teleop duration in seconds
        self.teleop_duration = 140.0

        # Period durations in seconds
        self.transition_duration = 10.0
        self.alliance_shift_durations = [25.0] * 4
        self.endgame_duration = 30.0

        # Calculate start times for alliance shifts (relative to teleop start)
        self.alliance_shift_starts = [self.transition_duration]
        for i in range(1, 4):
            self.alliance_shift_starts.append(
                self.alliance_shift_starts[-1] + self.alliance_shift_durations[i - 1]
            )
        self.endgame_start = (
            self.alliance_shift_starts[-1] + self.alliance_shift_durations[-1]
        )

        # Cache some values
        self.last_known_match_time = -1
        self.current_shift_start_time = -1
        self.current_shift_duration = -1
        self.current_shift_number = 0  # 0: transition, 1-4: alliance shifts, 5: endgame

    def periodic(self):
        """Automatically called by CommandScheduler"""
        self.update()
        super().periodic()

    def update(self):
        """Update internal state based on match time and game data"""
        # Get alliance once (shouldn't change during match)
        if self.our_alliance is None:
            self.our_alliance = DriverStation.getAlliance()

        # Get game data once (sent ~3s after auto)
        if self.game_data is None:
            gd = DriverStation.getGameSpecificMessage()
            if gd and gd.strip():
                self.game_data = gd.strip().upper()
                if self.game_data in ("R", "B"):
                    self.first_inactive_alliance = self.game_data

        # Only try to do shift calculations when we have everything
        if self.has_good_data():
            self.update_shift_info()

    def has_good_data(self) -> bool:
        return (
            self.our_alliance is not None and self.first_inactive_alliance is not None
        )

    def our_hub_is_active(self) -> bool:
        """Returns whether OUR HUB is currently active"""
        if not self.has_good_data():
            return False  # safest default

        # If first inactive = opponent → we are active first → our hub is active
        our_char = "R" if self.our_alliance == DriverStation.Alliance.kRed else "B"

        if self.current_shift_number == 0 or self.current_shift_number == 5:
            return True
        else:
            # If we are the first inactive alliance, inactive on odd shifts, active on even
            if self.first_inactive_alliance == our_char:
                return self.current_shift_number % 2 == 0
            else:
                return self.current_shift_number % 2 == 1

    def update_shift_info(self):
        """Update which shift we think we are in + when it started"""
        match_time_remaining = DriverStation.getMatchTime()

        # We only update when time is meaningfully changing (avoid noise)
        if abs(match_time_remaining - self.last_known_match_time) < 0.05:
            return

        self.last_known_match_time = match_time_remaining

        # Time already passed in teleop
        teleop_time_elapsed = self.teleop_duration - match_time_remaining

        # Determine current shift
        new_shift_number = 0
        new_start_time = 0.0
        new_duration = 0.0

        if teleop_time_elapsed < 0:
            # Not in teleop yet
            return

        if teleop_time_elapsed < self.transition_duration:
            new_shift_number = 0
            new_start_time = 0.0
            new_duration = self.transition_duration
        elif teleop_time_elapsed < self.endgame_start:
            for i in range(4):
                start = self.alliance_shift_starts[i]
                if teleop_time_elapsed < start + self.alliance_shift_durations[i]:
                    new_shift_number = i + 1
                    new_start_time = start
                    new_duration = self.alliance_shift_durations[i]
                    break
        else:
            new_shift_number = 5
            new_start_time = self.endgame_start
            new_duration = 999.0

        if new_shift_number != self.current_shift_number:
            self.current_shift_number = new_shift_number
            self.current_shift_start_time = new_start_time
            self.current_shift_duration = new_duration

    def time_remaining_in_current_shift(self) -> float:
        """Returns seconds remaining in current shift"""
        if not self.has_good_data():
            return -1.0

        match_time_remaining = DriverStation.getMatchTime()
        teleop_time_elapsed = self.teleop_duration - match_time_remaining

        if self.current_shift_number == 5:
            return 999.0
        else:
            return (
                self.current_shift_start_time
                + self.current_shift_duration
                - teleop_time_elapsed
            )

    # ─────────────── Telemetry methods ───────────────

    @FROGSubsystem.telemetry("Hub Status")
    def status_telem(self) -> str:
        return (
            "Received game data" if self.has_good_data() else "Waiting for game data..."
        )

    @FROGSubsystem.telemetry("Our HUB Active")
    def our_hub_active_telem(self) -> bool:
        return self.our_hub_is_active()

    @FROGSubsystem.telemetry("First Inactive Alliance")
    def first_inactive_alliance_telem(self) -> str:
        return str(self.first_inactive_alliance)

    @FROGSubsystem.telemetry("Time Left in Shift")
    def time_left_in_shift_telem(self) -> float:
        return self.time_remaining_in_current_shift()

    @FROGSubsystem.telemetry("Current Shift Number")
    def current_shift_number_telem(self) -> int:
        return self.current_shift_number


from typing import Callable, Optional
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import Field2d
from commands2.button import Trigger
import constants


class FieldZones(FROGSubsystem):
    # Covers the trench areas running along the sides of the field.
    # X bounds roughly align with the hubs (4.626 to 11.915).
    # We split this into 4 zones, leaving the very center of the field open.
    NO_SHOOT_ZONES = [
        # Blue side trenches (X from blue hub to midline ~8.25)
        {
            "x_min": 4.05,
            "x_max": 5.23,
            "y_min": 0.0,
            "y_max": 1.265,
        },  # Blue Right Trench
        {"x_min": 4.05, "x_max": 5.24, "y_min": 6.75, "y_max": 8.0},  # Blue Left Trench
        # Red side trenches (X from midline ~8.25 to red hub)
        {
            "x_min": 11.33,
            "x_max": 12.53,
            "y_min": 0.0,
            "y_max": 1.265,
        },  # Red Left Trench
        {
            "x_min": 11.33,
            "x_max": 12.53,
            "y_min": 6.75,
            "y_max": 8.0,
        },  # Red Right Trench
    ]

    # The locations of the AprilTags placed directly over the trenches
    TRENCH_TAGS = [
        Translation2d(11.8779798, 7.4247756),  # ID 1
        Translation2d(11.8779798, 0.6444996),  # ID 6
        Translation2d(11.9528844, 0.6444996),  # ID 7
        Translation2d(11.9528844, 7.4247756),  # ID 12
        Translation2d(4.6630844, 0.6444996),  # ID 17
        Translation2d(4.6630844, 7.4247756),  # ID 22
        Translation2d(4.5881798, 7.4247756),  # ID 23
        Translation2d(4.5881798, 0.6444996),  # ID 28
    ]
    SPEED_SCALAR = 0.99
    FORCEFIELD_OUTER_RADIUS = 1.5 # meters. Start slowing down here.
    FORCEFIELD_INNER_RADIUS = 0.75 # meters. Complete stop here.

    def __init__(
        self,
        pose_supplier: Callable[[], Pose2d],
        hood_state_supplier: Callable[[], bool],
        field: Field2d,
    ):
        super().__init__()
        self.pose_supplier = pose_supplier
        self.hood_state_supplier = hood_state_supplier
        self.status = "Clear"

        self.field = field
        self._setup_field2d_zones()

    def _setup_field2d_zones(self):
        for i, zone in enumerate(self.NO_SHOOT_ZONES):
            poses = [
                Pose2d(zone["x_min"], zone["y_min"], Rotation2d()),
                Pose2d(zone["x_min"], zone["y_max"], Rotation2d()),
                Pose2d(zone["x_max"], zone["y_max"], Rotation2d()),
                Pose2d(zone["x_max"], zone["y_min"], Rotation2d()),
                Pose2d(zone["x_min"], zone["y_min"], Rotation2d()),  # Close the loop
            ]
            self.field.getObject(f"NoShootZone_{i}").setPoses(poses)

    def in_restricted_zone(self, pose: Optional[Pose2d] = None) -> bool:
        pose_to_check = pose or self.pose_supplier()
        x, y = pose_to_check.x, pose_to_check.y

        for zone in self.NO_SHOOT_ZONES:
            if (
                zone["x_min"] <= x <= zone["x_max"]
                and zone["y_min"] <= y <= zone["y_max"]
            ):
                return True
        return False

    def get_trench_velocity_limit(self, intended_velocity: Translation2d) -> Translation2d:
        """Applies a proportional forcefield if the hood is deployed.
        Scales down the velocity vector component moving TOWARD the tags, leaving
        motion away from tags unaffected.
        """
        if not self.hood_state_supplier() or intended_velocity.norm() == 0:
            return intended_velocity

        pose = self.pose_supplier()
        robot_translation = pose.translation()

        closest_tag = None
        min_dist = float('inf')
        
        for tag in self.TRENCH_TAGS:
            dist = robot_translation.distance(tag)
            if dist < min_dist:
                min_dist = dist
                closest_tag = tag
                
        if closest_tag is None or min_dist >= self.FORCEFIELD_OUTER_RADIUS:
            return intended_velocity
            
        # Calculate scalar between 0.0 (at inner radius) and 1.0 (at outer radius)
        if min_dist <= self.FORCEFIELD_INNER_RADIUS:
            allowed_forward_scalar = 0.0
        else:
            range_total = self.FORCEFIELD_OUTER_RADIUS - self.FORCEFIELD_INNER_RADIUS
            allowed_forward_scalar = (min_dist - self.FORCEFIELD_INNER_RADIUS) / range_total

        # Vector pointing from robot TO the tag
        to_tag_vector = (closest_tag - robot_translation)
        to_tag_norm = to_tag_vector.norm()
        
        if to_tag_norm == 0:
             return intended_velocity * allowed_forward_scalar
             
        to_tag_unit = to_tag_vector / to_tag_norm
        
        # Calculate how much of our intended velocity is directly toward the tag
        # Dot product: V_intended • Unit_To_Tag
        forward_component = intended_velocity.x * to_tag_unit.x + intended_velocity.y * to_tag_unit.y
        
        if forward_component <= 0:
            # We are driving parallel to or away from the tag. Don't restrict.
            return intended_velocity
            
        # We are driving TOWARD the tag. Scale down only that forward component.
        restricted_forward_component = forward_component * allowed_forward_scalar
        
        # Reconstruct the final velocity:
        # Intended - original forward component + restricted forward component
        velocity_reduction = to_tag_unit * (forward_component - restricted_forward_component)
        final_velocity = intended_velocity - velocity_reduction
        
        return final_velocity

    def get_aim_target(self, pose: Optional[Pose2d] = None) -> Optional[Pose2d]:
        """Dynamically return an aim target based on field position and alliance."""
        pose_to_check = pose or self.pose_supplier()
        x = pose_to_check.x
        y = pose_to_check.y
        alliance = wpilib.DriverStation.getAlliance()

        if alliance == wpilib.DriverStation.Alliance.kRed:
            # Red Alliance: Alliance zone is X > 11.0, opponent zone is X < 5.5
            if x > 11.0:
                return constants.kRedHub
            elif 5.5 <= x <= 11.0:
                # Middle of the field -> closest Red corner (x=16.5)
                if y < 4.1:
                    return constants.kRedRightCorner
                else:
                    return constants.kRedLeftCorner
            else:
                return constants.kRedHub  # Default fallback
        else:
            # Blue Alliance (or fallback): Alliance zone is X < 5.5, opponent zone is X > 11.0
            if x < 5.5:
                return constants.kBlueHub
            elif 5.5 <= x <= 11.0:
                # Middle of the field -> closest Blue corner (x=0)
                if y < 4.1:
                    return constants.kBlueRightCorner
                else:
                    return constants.kBlueLeftCorner
            else:
                return constants.kBlueHub  # Default fallback

    def get_no_shoot_trigger(self) -> Trigger:
        return Trigger(lambda: self.in_restricted_zone())

    def get_max_speed_scalar(self) -> float:
        """Returns a scalar (0.0 to 1.0) to limit drive speed based on zone."""
        if self.in_restricted_zone():
            return self.SPEED_SCALAR / constants.kMaxMetersPerSecond
        return 1.0

    def periodic(self):
        if self.in_restricted_zone():
            self.status = "Restricted Zone!"
        else:
            self.status = "Clear"

        super().periodic()

    @FROGSubsystem.telemetry("Status")
    def status_telem(self) -> str:
        return self.status
