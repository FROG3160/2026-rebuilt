# Thursday 2026 OKTU Competition Changes

## Summary of Changes
- **Feeder State Machine**: Switched to using feeder state enums and finished implementing feeder triggers.
- **Shooter & Intake Logic**: 
  - Added cycling method.
  - Added zeroing logic for the intake deploy and the hood.
  - Removed the zero hood check and changed back to the original zero hood approach.
  - Removed reverse feed from the shooting sequence.
  - Updated the flywheel tolerance.
- **PathPlanner & Autonomous**: 
  - Fixed `ZeroDivisionError` by removing redundant rotation targets at the ends of paths.
  - Removed bad rotation targets at the start of paths.
  - Fixed PathPlanner event markers so that NamedCommands (`Intake Start` and `Intake Stop`) actually trigger the command properly by passing a fully formed command object.
  - Resolved `robotcontainer` boolean fixes and general path updates.
- **NamedCommands Update**: Fixed NamedCommands registration and functionality.
