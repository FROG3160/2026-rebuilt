import wpilib
from wpilib import DriverStation, SmartDashboard, Timer


class ShiftTracker:
    """
    Helper class to easily answer:
      - Is our HUB currently active?
      - How much time is left in the current shift?
    """

    def __init__(self):
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

    def update(self):
        """Call this periodically (e.g. every loop in robotPeriodic / teleopPeriodic)"""
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
            new_duration = float("inf")  # or self.endgame_duration if finite preferred

        if new_shift_number != self.current_shift_number:
            self.current_shift_number = new_shift_number
            self.current_shift_start_time = new_start_time
            self.current_shift_duration = new_duration

    def time_remaining_in_current_shift(self) -> float:
        """Returns seconds remaining in current shift (or very large number on last shift)"""
        if not self.has_good_data():
            return -1.0

        match_time_remaining = DriverStation.getMatchTime()
        teleop_time_elapsed = self.teleop_duration - match_time_remaining

        if self.current_shift_number == 5:
            return float("inf")
        else:
            return (
                self.current_shift_start_time
                + self.current_shift_duration
                - teleop_time_elapsed
            )

    # ─────────────── Convenience / Dashboard methods ───────────────
    def put_to_dashboard(self):
        if self.has_good_data():
            SmartDashboard.putString("Hub/Status", "Recived game data")
            SmartDashboard.putBoolean("Hub/Our HUB Active", self.our_hub_is_active())
            SmartDashboard.putString("Hub/First Inactive", self.first_inactive_alliance)

            remaining = self.time_remaining_in_current_shift()
            SmartDashboard.putNumber("Hub/Time Left in Shift", remaining)
            SmartDashboard.putNumber("Hub/Current Shift", self.current_shift_number)
        else:
            SmartDashboard.putString("Hub/Status", "Waiting for game data...")
