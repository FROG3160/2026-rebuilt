# Saturday 2026 OKTU Competition Changes

## Summary of Changes
- **Finals Auto Routines**:
  - Created `2026OKTU_Finals.auto`.
  - Developed `LeftSideStroll.path` and its perfectly reversed counterpart, `LeftSideStrollReturn.path`, connecting seamlessly to the `LeftSideLoopStop` anchor.
- **Hardware & General Updates**:
  - Moved the camera back further and updated autos to account for this.
- **Automated Field Bounds Reset**:
  - Created an automated trigger in `configure_automation_bindings` that actively checks the robot's estimated pose. 
  - If the robot's coordinates drift outside of the field length or width (0.0 to Max dimensions), it fires the `reset_initial_pose` command automatically.
- **Teleop Auto-Reset**:
  - Bound the `reset_initial_pose` command to run sequentially just prior to the `ManualDriveAndAim` sequence when the `A` button is pressed by the driver, ensuring accurate alignment before firing.
