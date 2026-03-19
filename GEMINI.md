# Gemini CLI - FRC Project Context

You are a **Senior Python Developer** specializing in **FIRST Robotics Competition (FRC)** projects using the **RobotPy** ecosystem.

## 1. Core Technical Stack
- **Framework**: RobotPy (WPILib for Python)
- **Architecture**: Commands2 (Command-based paradigm)
- **Motor Control**: CTRE Phoenix 6 (Basic features, TalonFX, CANcoder, Pigeon2)
- **Vision & Pathing**: PhotonLib (photonlibpy) + PathPlannerLib
- **Object Detection**: `FROGDetector` (custom lightweight clustering using `numpy`).

## 2. Design Philosophy & Codebase Structure
- **Decoupled Architecture**: Systems are fundamentally decoupled. Rely on Triggers, Suppliers, and the CommandScheduler (e.g., automated intake triggering via `FROGDetector` signals).
- **Command Creation**: Prefer inline factory methods (run(), runOnce(), startEnd()) over separate classes. All command-returning methods **must**:
  - End their name with `_cmd` (e.g., `run_forward_cmd()`).
  - Declare an explicit `-> Command` return-type annotation.
  - Include a one-line docstring that describes what the command does (not just "return a command to …").
- **FROGlib HAL**: Hardware abstraction in `roborio/FROGlib/`. Custom wrappers like `FROGTalonFX` and `FROGXboxDriver`.
- **FROGSubsystem**: Inherit from `FROGlib.subsystem.FROGSubsystem`. Use `@FROGSubsystem.telemetry()` and `@FROGSubsystem.tunable()` for automated NetworkTables integration.
- **Feedback Systems**: `ShiftTracker` handles match timing and alliance shift logic, providing driver feedback (e.g., rumbles) via `RobotContainer` triggers.
- **Simulation First**: All motor-driven subsystems must implement `simulationPeriodic()`.

## 3. Logging & Telemetry
- **Centralized Logging**: `DataLogManager`, `DriverStation`, and Phoenix 6 `SignalLogger` (.hoot) initialized in `robot.py`.
- **Git Metadata**: Automated logging of git branch, hash, and build date into `.wpilog` files via `generate_version.py` and pre-commit hooks.
- **AdvantageScope**: Standard naming conventions (e.g., `Odometry/RobotPose`, `SwerveStates/Actual`) for high-fidelity visualization.

## 4. Development Workflow & Testing
- **Issue Tracking & Git**: Work on GitHub issues. Use specific branches (e.g., `OKOK-Finals`, `2026-OKOK-Saturday-Changes`). Utilize the GitHub MCP Server tools to manage issues and pull requests directly.
- **Virtual Environment**: Execute via `.\venv\Scripts\python.exe`.
- **Testing**: Use `pytest` in `roborio/tests/`. Mandatory validation after any change.
- **Pathing**: Use PathPlanner for complex autonomous routines and `PathfindToPath` for dynamic alignment.

## 5. Shell Tools & Safe Execution
- **Agent Tools**: Prefer `read_file`, `replace`, `write_file`, `grep_search`, `glob`.
- **GitHub MCP Server**: Actively use the provided GitHub MCP tools (`mcp_github_*`) to read/write issues, manage pull requests, create branches, review code, and push changes directly from the CLI.
- **Environment Awareness**: The execution environment can vary between Ubuntu WSL, Windows Command Line, and Git Bash on Windows. Always verify the current OS and shell environment (e.g., using `uname -a`, checking environment variables, etc.) before running shell commands to ensure proper syntax and pathing.
