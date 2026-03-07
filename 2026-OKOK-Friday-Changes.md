# Summary of Changes (OKOK-2026-Friday_changes)

This branch incorporates significant improvements to autonomous execution, unified aiming logic, and climber safety.

## 1. Core Framework (FROGlib)
- **Motor Safety**: Added default current limits (40A supply, 60A stator) to all motors using `FROGTalonFXConfig`.
- **Simulation**: Enhanced `FROGTalonFX` simulation to support an `invert_sim` flag, allowing physical gearing to remain positive while fixing simulated rotation directions.

## 2. Autonomous & PathPlanner Integration
- **Auto Chooser**: Enabled `AutoBuilder.buildAutoChooser()`, allowing autonomous routine selection from the dashboard.
- **Live Telemetry**: Integrated `PPLibTelemetry` to report the robot's estimated pose to the PathPlanner application in real-time.
- **Autonomous Execution**: Fixed `robot.py` to correctly schedule and cancel the selected autonomous command.
- **Named Commands**: Registered the `"Fire"` command, enabling scored shots during autonomous paths.

## 3. Unified Aiming & Firing Logic
- **Drive Subsystem**: Added `calculate_vT_to_target()` to centralize rotational PID and motion compensation math.
- **RobotContainer**: Implemented `get_firing_command_group()`. This unified sequence is used for both the 'A' button (Manual) and the `"Fire"` auto command.
- **Dynamic Overrides**: While firing during an autonomous path, the robot now overrides path rotation to lock onto the target while continuing its translational movement.

## 4. Climber Subsystem Enhancements
- **Safety Checks**: Added `is_deployed()` detection and `is_endgame` logic.
- **Restricted Controls**:
    - Deployment and stowing are now restricted to the last 30 seconds of the match.
    - Added manual lift controls to the D-Pad (POV Up/Down) at 1V.
    - POV lift controls are locked unless the climber is physically deployed AND the match is in endgame.

## 5. Shooter Subsystem Updates
- **Precision Tuning**: Added a `speed_multiplier` (default 1.07) to flywheel calculations to allow for fine-tuning without modifying regression data.
- **Simulation**: Fixed the hood simulation direction so it accurately reflects deployment in the simulator.
- **Telemetry**: Added hood position and velocity telemetry for better dashboard visibility.

## 6. Pathing
- Generated mirrored sweep paths (`CloseSweepRight`, `FarSweepLeft`, `FarSweepRight`) to ensure field-wide symmetry.
- Included `mirror_paths.py` in the paths folder for procedural path generation.
