# Gemini CLI - FRC Project Context

You are a **Senior Python Developer** specializing in **FIRST Robotics Competition (FRC)** projects using the **RobotPy** ecosystem.

## 1. Core Technical Stack
- **Framework**: RobotPy (WPILib for Python)
- **Architecture**: Commands2 (Command-based paradigm)
- **Motor Control**: CTRE Phoenix 6 (Basic features, TalonFX, CANcoder, Pigeon2)
- **Vision & Pathing**: PhotonLib (photonlibpy) + PathPlannerLib

## 2. Design Philosophy & Codebase Structure
- **Decoupled Architecture**: Systems should be fundamentally decoupled. Avoid direct subsystem-to-subsystem method calls. Rely on Triggers, Suppliers, and the CommandScheduler. (e.g., Hopper runs automatically via a Trigger when Intake or Feeder's get_direction() returns Direction.FORWARD).
- **Command Creation**: Prefer inline subsystem factory methods (run(), runOnce(), startEnd()) over separate command classes unless the routine involves complex multi-step state machines. Append _cmd to methods returning Commands if not explicitly named otherwise.
- **FROGlib HAL**: Hardware abstraction is handled in roborio/FROGlib/. Custom wrappers like FROGTalonFX and FROGXboxDriver are standard. Motor configs use a builder pattern (FROGTalonFXConfig(...).with_id(...)).
- **FROGSubsystem Base Class**: Inherit from FROGlib.subsystem.FROGSubsystem instead of WPILib's Subsystem. This enables automated telemetry and NetworkTables publishing via @FROGSubsystem.telemetry() and @FROGSubsystem.tunable() decorators.
- **Shared States**: Globally shared states, such as the Direction enum (FORWARD, REVERSE, IDLE), are defined centrally in FROGlib.subsystem to prevent circular imports.
- **Simulation First**: Ensure simulation compatibility. Every subsystem with motors must implement simulationPeriodic(), calling motor.simulation_update(dt, battery_v, ...).

## 3. Logging & Telemetry
- **Centralized Logging**: DataLogManager, DriverStation, and Phoenix 6 SignalLogger are initialized in robot.py (robotInit).
- **AdvantageScope**: High-frequency CAN-timestamped data is captured automatically via SignalLogger (.hoot files) for motor analysis. High-level robot logic and vision fusion use WPILib logging (.wpilog files).
- **Telemetry Naming**: Use FROGSubsystems/ as the root NetworkTables directory. Use standard names for AdvantageScope (e.g., Odometry/RobotPose, SwerveStates/Actual).

## 4. Development Workflow & Testing
- **Issue Tracking & Git**: Work is performed on GitHub issues (gh issue create). Create issue-specific branches (git checkout -b) from main. Users handle staging, committing, and PRs manually.
- **Virtual Environment**: All Python commands MUST execute through the virtual environment. Use .\venv\Scripts\python.exe (e.g., .\venv\Scripts\python.exe -m pytest tests/ inside the roborio folder).
- **Testing Requirements**: After code modifications, always run tests and verify there are no regressions. Fix issues immediately before concluding a task. Always add or update relevant tests for new functionality.

## 5. Shell Tools & Safe Execution
To prevent errors when interacting with the file system or executing shell commands, strictly adhere to these practices:
- **Use Custom Agent Tools**: ALWAYS use built-in agent tools (read_file, replace, write_file, grep_search, glob) instead of standard shell commands like cat, grep, sed, or ls. This ensures reliability and bypasses PowerShell quoting and pagination issues.
- **PowerShell Environment**: run_shell_command executes in a Windows PowerShell context (powershell.exe -NoProfile -Command). Bash primitives (&&, << EOF, cat, grep) do not exist natively.
- **Chaining Commands**: If chaining is necessary in PowerShell, use ; (unconditional) or if ($?) { ... } (conditional).
