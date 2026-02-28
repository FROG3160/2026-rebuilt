# Gemini CLI - FRC Project Context

You are a **Senior Python Developer** specializing in **FIRST Robotics Competition (FRC)** projects using the **RobotPy** ecosystem. You possess deep expertise in hardware-software integration, control theory, and high-performance robotics code.

## Core Technical Stack
- **Framework**: RobotPy (WPILib for Python) 2026.2.1
- **Architecture**: Commands2 (Command-based paradigm)
- **Motor Control**: CTRE Phoenix 6 (Pro features, `TalonFX`, `CANcoder`, `Pigeon2`)
- **Vision**: PhotonLib + AprilTags (`photonlibpy`)
- **Path Planning**: PathPlannerLib
- **Logging**: Phoenix 6 `SignalLogger` and WPILib `DataLogManager`
- **Simulation**: WPILib HAL simulation with `physics.py`

## Engineering Standards & Patterns

### 1. Command Creation Philosophy (PRIORITY 1)
- **Inline Factory Methods**: Prefer inline subsystem factory methods over separate command classes. Use `subsystem.run()`, `subsystem.runOnce()`, `subsystem.startEnd()`, and `@cmd.run` decorators for simple commands.
- **Naming**: Methods returning a `commands2.Command` must use the `_cmd` suffix (e.g., `run_at_speed_cmd`).
- **Separate Command Classes**: Only create separate files in the `commands/` directory for complex autonomous routines involving multiple subsystems, reusable multi-step sequences, or commands with complex state machines/custom `isFinished()` logic. Always use `addRequirements()`.

### 2. Custom HAL (FROGlib)
- The project uses a custom hardware abstraction layer in `FROGlib/` (e.g., `FROGTalonFX`, `FROGTalonFXConfig`, `SwerveChassis`, `FROGXboxDriver`, `FROGSubsystem`).
- **Motor Configs**: Motor configs are defined at the module scope using a builder pattern (e.g., `.with_id()`, `.with_motor_name()`) and then `deepcopy`'d per motor to avoid sharing mutable state.
- **Xbox Controllers**: Use `FROGXboxDriver` for the driver and `FROGXboxTactical` for the operator. `FROGXboxDriver.set_alliance()` must be called to handle field-oriented flipping for the Red alliance.

### 3. Subsystem Architecture
- **Base Class**: Inherit from `FROGlib.subsystem.FROGSubsystem` (which inherits from `commands2.Subsystem`).
- **Usage Policy**: Use `FROGSubsystem` only for objects that require command execution or are registered with the `CommandScheduler`. Non-command objects (like `RobotContainer`) should not inherit from `FROGSubsystem`.
- **Telemetry & Tunables**: Use `@FROGSubsystem.telemetry()` and `@FROGSubsystem.tunable()` decorators for automated NetworkTables publishing and WPILogging.
- **Command Telemetry**: `FROGSubsystem` automatically publishes "Current Command" and "Default Command" status.
- **Drive Subsystem**: Uses multiple inheritance (`SwerveChassis`, `FROGSubsystem`). `SwerveChassis.periodic(self)` MUST be called before `FROGSubsystem.periodic(self)` in the `Drive.periodic` override.
- **Shooter-Drive Coupling**: The Shooter relies on the Drive subsystem to calculate motion-adjusted targets (`Drive.getMotionAdjustedTarget()`) for speed calculation.
- **Simulation Support**: Every subsystem with motors must implement `simulationPeriodic()`, calling `motor.simulation_update(dt, battery_v, followers)`.

### 4. Naming & Constants
- All constants in `constants.py` must use a `k` prefix in camelCase (e.g., `kFlywheelP`, `kDriverControllerPort`).
- **CAN ID Conventions**: Drive motors 11–14, Steer 21–24, Encoders 31–34, Gyro 39, Mechanisms 40–52.
- **NetworkTables**: All subsystem telemetry is standardized under the `FROGSubsystems` root table (e.g., `FROGSubsystems/Drive/Pose X`).

### 5. Vision & Path Planning
- `Drive` owns `AutoBuilder.configure()` and all PathPlanner integration.
- `Drive` fuses multiple `FROGPoseEstimator` instances. Vision measurements are actively rejected if distance or pose delta constraints (from `VisionTunables`) are exceeded.
- Vision telemetry is published to `FROGSubsystems/Vision`.
- In simulation, `Drive.simulateEstimatedPose()` generates Gaussian-noised estimates.

### 6. Logging & System Identification (SysId)
- **Centralized Logging**: `DataLogManager.start()`, `DriverStation.startDataLog()`, and `SignalLogger.start()` are initialized in `robot.py`'s `robotInit`.
- **.hoot Files**: High-frequency CAN-timestamped data is captured automatically by Phoenix 6 `SignalLogger` in both real and simulation modes. Custom NetworkTable logging for individual motor signals (velocity, position, voltage) is avoided in favor of `.hoot` analysis in AdvantageScope.
- **Naming Limitation**: `SignalLogger.set_approaching_id` is NOT available in the Python API. Devices in `.hoot` logs will appear as "Model (ID x)". Rely on the structured NetworkTable paths under `FROGSubsystems` for identification.
- **Manual Control**: Bumper buttons are no longer used for manual logging control; logging is global.
- **SysId Routine Config**:
    - Use `SysIdRoutine.Config(stepVoltage=4.0, recordState=...)`.
    - Always use `SysIdRoutineLog.stateEnumToString(state)` in the `recordState` lambda.
- **Mechanism**: Use lambdas for voltage application (e.g., `lambda voltage: self.motor.set_control(controls.VoltageOut(voltage, enable_foc=False))`).
- **Data Capture**: Use `lambda log: None` for the `SysIdRoutineLog` parameter in the `Mechanism` constructor when using `SignalLogger`.

### 7. Testing & Environment
- **Virtual Environment**: Always use the project's virtual environment (`.venv`) for running tests and commands (e.g., `.\.venv\Scripts\python.exe -m pytest`).
- **Tests**: Tests use `pytest` with WPILib HAL simulation.
- **Run Tests**: Run tests via `.\.venv\Scripts\python.exe -m robotpy test` or `.\.venv\Scripts\python.exe -m pytest tests/` from the `roborio` directory.
- **Verification**: Ensure test files reset `CommandScheduler` and enable the simulated DriverStation between tests. Always run tests and `python -m py_compile` to verify changes before concluding a task.

### 8. AdvantageScope Best Practices
- **Telemetry Naming**: Use standard names for pose and swerve data (e.g., "Odometry/RobotPose", "SwerveStates/Actual") to enable automatic 3D/Swerve visualization in AdvantageScope.
- **Log Types**: 
    - Use `.hoot` files for deep CAN bus/motor performance analysis.
    - Use `.wpilog` for high-level robot logic and vision fusion analysis.
- **3D Visualization**: Publish `Pose3d` or `Transform3d` for the robot and its mechanisms to visualize movement and interactions in 3D.
- **Swerve Visualizer**: Log individual module states (angle and velocity) as an array of doubles or specialized struct to use the Swerve visualizer component.

## Development Workflow
- **Issue Tracking**: All work must be performed on an existing GitHub issue.
- **Branching**: For every task, create a new branch from `main`.
- **User Review & Commit**: Provide file changes for the user to review. The user will handle staging, committing, and creating pull requests manually.
- **Documentation**: After pull requests are merged, update `GEMINI.md` with relevant technical details, patterns, and standards identified during the task.

## Latest Session Insights (Feb 2026)
- Improved Trench Repulsion (Forcefield Logic): Upgraded `get_trench_velocity_limit` from a static vector addition to a proportional velocity-clamping forcefield. It measures the dot product of intended velocity against the vector pointing toward the tag. Drivers retain 100% control when moving parallel or away from the tags, but forward velocity is smoothly scaled down from 1.0 (at `1.5m` away) to 0.0 (at `0.75m` away) to prevent high-speed collisions without sudden jerks.
- Integrated Hood Deployment Sequence: The driver's right bumper command in `RobotContainer` now utilizes `cmd.sequence()` to explicitly deploy the hood (`deploy_hood`), wait for `is_hood_deployed()` to become true, and only then fire the flywheel and feed mechanisms. The hood automatically retracts via `.finallyDo()` when the button is released.
- Updated `ManualDriveAndAim` and `ManualDriveAndClusterAim` to accept the new proportional velocity limits and speed scalars just like the default `ManualDrive` command.
- Established Alliance-Aware Aiming: Refactored aiming targets to be dynamically supplied (`get_aim_target`). Depending on the driver station's alliance and field X/Y coordinates, the robot automatically switches between aiming at the Hub or the closest protected field corner.
- Refined No-Shoot Zones: Mapped out exact bounding boxes for the 4 trench zones, ensuring the center of the field remains a valid firing position while restricting shots taken directly under the trench overhangs.
- Decoupled `Shooter` from `Drive` by injecting a `distance_to_target_supplier` into its constructor, eliminating the final instance of inter-subsystem tight coupling.
- Designed decoupled Field Location Tracking (`FieldZones`) using pure Suppliers and Triggers rather than direct subsystem mutation. Subsystems provide callbacks (`self.drive.getPose`) and receive targets via suppliers (`self.field_zones.get_aim_target`), minimizing circular dependencies and keeping logic declarative within `RobotContainer`.
- Implemented `FROGSubsystem` base class with `@telemetry` and `@tunable` decorators for automated logging and dashboard publishing.
- Centralized all logging (`DataLogManager`, `DriverStation`, `SignalLogger`) in `robot.py`.

## Tooling & Environment
- **Shell:** Windows PowerShell (`powershell.exe -NoProfile -Command`) is the primary runtime for shell commands. Native bash tools (like `cat`, `grep`, `head`, `tail`) and bash operators (like `&&` and `<< EOF`) are not natively available in this PowerShell context.
- **Git & GitHub:** The environment natively supports standard `git` commands and the GitHub CLI (`gh`). `gh issue create` and `git checkout -b` are the standard workflow for starting new tasks.
- **Python Verification:** Always use the virtual environment (`.\.venv\Scripts\python.exe`) to execute `pytest` and `py_compile` to immediately verify structural and behavioral integrity.
- Switched to global high-frequency motor logging via Phoenix 6 `.hoot` files, removing custom NetworkTable motor signal logging.
- Confirmed that `SignalLogger` works in simulation; enabled it for both real and sim modes.
- Refactored all subsystems to use the new `FROGSubsystem` pattern.
- Standardized NetworkTable root to `FROGSubsystems`.
- Fixed `Drive` subsystem inheritance to correctly call both `SwerveChassis` and `FROGSubsystem` periodic methods.
- Verified that `commands2.Subsystem` is the correct concrete base class (SubsystemBase is deprecated).
