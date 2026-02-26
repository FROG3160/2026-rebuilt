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
- The project uses a custom hardware abstraction layer in `FROGlib/` (e.g., `FROGTalonFX`, `FROGTalonFXConfig`, `SwerveChassis`, `FROGXboxDriver`).
- **Motor Configs**: Motor configs are defined at the module scope using a builder pattern (e.g., `.with_id()`, `.with_motor_name()`) and then `deepcopy`'d per motor to avoid sharing mutable state.
- **Xbox Controllers**: Use `FROGXboxDriver` for the driver and `FROGXboxTactical` for the operator. `FROGXboxDriver.set_alliance()` must be called to handle field-oriented flipping for the Red alliance.

### 3. Subsystem Architecture
- **Base Class**: Use `commands2.Subsystem` as the base class for all subsystems. (Note: `SubsystemBase` is deprecated in the 2026.2.1 RobotPy environment).
- **Drive Subsystem**: Uses multiple inheritance (`SwerveChassis`, `Subsystem`). The `SwerveChassis` handles low-level math and NT publishing. The drive starts **disabled** and must be explicitly enabled in `teleopInit`, `autonomousInit`, and `testInit`.
- **Shooter-Drive Coupling**: The Shooter relies on the Drive subsystem to calculate motion-adjusted targets (`Drive.getMotionAdjustedTarget()`) for speed calculation.
- **Simulation Support**: Every subsystem with motors must implement `simulationPeriodic()`, calling `motor.simulation_update(dt, battery_v, followers)`.

### 4. Naming & Constants
- All constants in `constants.py` must use a `k` prefix in camelCase (e.g., `kFlywheelP`, `kDriverControllerPort`).
- **CAN ID Conventions**: Drive motors 11–14, Steer 21–24, Encoders 31–34, Gyro 39, Mechanisms 40–52.
- **NetworkTables**: Publish telemetry under `constants.kComponentSubtableName` ("FROGSubsystems"). Each mechanism publishes to its own subtable (e.g., `FROGSubsystems/Drive/FrontLeft`). Expose tunable values via `initSendable()` / `SendableBuilder`.

### 5. Vision & Path Planning
- `Drive` owns `AutoBuilder.configure()` and all PathPlanner integration.
- `Drive` fuses multiple `FROGPoseEstimator` instances. Vision measurements are actively rejected if distance or pose delta constraints (from `VisionTunables`) are exceeded.
- In simulation, `Drive.simulateEstimatedPose()` generates Gaussian-noised estimates.

### 6. System Identification (SysId)
When implementing SysId characterization for subsystems (especially those using Phoenix 6 motors):
- **Logging**: Use Phoenix 6 `SignalLogger` for high-frequency (CAN-timestamped) data.
- **Manual Control**: Bind bumper buttons (e.g., `Left Bumper` to `start`, `Right Bumper` to `stop`) to manage the logging session manually. This ensures clean, single-file logs for analysis.
- **Routine Config**:
    - Use `SysIdRoutine.Config(stepVoltage=4.0, recordState=...)`.
    - Always use `SysIdRoutineLog.stateEnumToString(state)` in the `recordState` lambda to ensure analysis tool compatibility.
- **Mechanism**: Use lambdas for the voltage application (e.g., `lambda voltage: self.motor.set_control(controls.VoltageOut(voltage, enable_foc=False))`).
- **Data Capture**: Use `lambda log: None` for the `SysIdRoutineLog` parameter in the `Mechanism` constructor when using `SignalLogger`, as it captures all motor signals automatically.

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
- Successfully implemented Shooter SysId characterization.
- Verified that `SignalLogger.set_approaching_id` is not available in the current Python API; use direct `start()`/`stop()` instead.
- Confirmed that manually starting/stopping the logger via the driver controller is the preferred workflow for characterization sessions.
