# Friday 2026 OKTU Competition Changes

## Summary of Changes
- **Double Loop Autonomous Routines**:
  - Implemented `LeftSideDoubleLoopAndFire.auto` and `RightSideDoubleLoopAndFire.auto`.
  - Added duplicate/extended second loops (`LeftSideSecondLoop.path` and `RightSideSecondLoop.path`) perfectly stitched from the ending positions of the first loops.
  - Updated `mirror_paths.py` to seamlessly mirror individual paths via CLI arguments and successfully mirrored all Left side path adjustments to the Right side.
  - Implemented a `.asProxy()` fix on `NamedCommands` inside `robotcontainer.py` to prevent WPILib subsystem requirement conflicts from crashing auto routines when `Fire` triggers the `Intake Cycle`.
- **Hood Homing (Teleop)**: 
  - Added a new zeroing command for the hood using stator current detection (homing).
  - Added functionality to automatically schedule hood homing during `teleopInit`.
  - Added a manual "Hood/Zero Position" button to the `SmartDashboard`.
- **Tuning and Hardware Adjustments**:
  - Changed the hood homing current value.
  - Updated the roller minimum speed.
  - Reverted intake speed changes.
  - Updated the rotation controller P value.
  - Fixed/adjusted the camera location.
