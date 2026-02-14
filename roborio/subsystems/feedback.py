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

        # Shift durations in seconds
        self.shift_durations = [25, 45, float("inf")]  # shift 1, 2, last

        # Cache some values
        self.last_known_match_time = -1
        self.current_shift_start_time = -1
        self.current_shift_number = -1

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
        opponent_char = "B" if self.our_alliance == DriverStation.Alliance.kRed else "R"

        if self.first_inactive_alliance == opponent_char:
            return True
        else:
            return False

    def update_shift_info(self):
        """Update which shift we think we are in + when it started"""
        match_time_remaining = DriverStation.getMatchTime()

        # We only update when time is meaningfully changing (avoid noise)
        if abs(match_time_remaining - self.last_known_match_time) < 0.05:
            return

        self.last_known_match_time = match_time_remaining

        # Time already passed in teleop
        teleop_time_elapsed = 120 - match_time_remaining  # 2:00 teleop

        shift_start_times = [0, 25, 70]  # start of shift 1,2,3 (seconds into teleop)

        current_shift = 1
        for i, start in enumerate(shift_start_times):
            if teleop_time_elapsed >= start:
                current_shift = i + 1
            else:
                break

        if current_shift != self.current_shift_number:
            self.current_shift_number = current_shift
            self.current_shift_start_time = shift_start_times[current_shift - 1]

    def time_remaining_in_current_shift(self) -> float:
        """Returns seconds remaining in current shift (or very large number on last shift)"""
        if not self.has_good_data():
            return -1.0

        match_time_remaining = DriverStation.getMatchTime()
        teleop_time_elapsed = 120 - match_time_remaining

        if self.current_shift_number == 1:
            return 25 - teleop_time_elapsed

        elif self.current_shift_number == 2:
            return 70 - teleop_time_elapsed  # 25 + 45 = 70

        else:  # shift 3 → end of match
            return match_time_remaining

    # ─────────────── Convenience / Dashboard methods ───────────────
    def put_to_dashboard(self):
        if self.has_good_data():
            SmartDashboard.putBoolean("Hub/Our HUB Active", self.our_hub_is_active())
            SmartDashboard.putString("Hub/First Inactive", self.first_inactive_alliance)

            remaining = self.time_remaining_in_current_shift()
            SmartDashboard.putNumber("Hub/Time Left in Shift", remaining)
            SmartDashboard.putNumber("Hub/Current Shift", self.current_shift_number)
        else:
            SmartDashboard.putString("Hub/Status", "Waiting for game data...")
