---
name: frog-robotpy
description: Senior Python Developer specializing in RobotPy (WPILib for Python) for FRC. Use when creating or modifying subsystems, commands, or hardware configurations for the FROG (FIRST Robotics Organization of Grove) codebase.
---

# FROG-RobotPy Skill

Specialized procedural knowledge for the FROG RobotPy codebase.

## 1. Core Technical Stack
- **Framework**: RobotPy (WPILib for Python)
- **Architecture**: Commands2 (Command-based paradigm)
- **Motor Control**: CTRE Phoenix 6 (Basic features, TalonFX, CANcoder, Pigeon2)
- **Vision & Pathing**: PhotonLib (photonlibpy) + PathPlannerLib
- **Object Detection**: `FROGDetector` (custom lightweight clustering using `numpy`).

## 2. Command Creation Workflow
All command-returning methods in subsystems MUST follow these rules:
- **Naming**: Must end with `_cmd` (e.g., `run_forward_cmd()`).
- **Return Type**: Declare explicit `-> Command` return-type annotation.
- **Docstring**: Include a one-line docstring describing what the command does.
- **Style**: Prefer inline factory methods (`run()`, `runOnce()`, `startEnd()`) over creating separate command classes.

## 3. Subsystem Implementation
Subsystems should be designed for maximum flexibility and simulation:
- **Inheritance**: Inherit from `FROGlib.subsystem.FROGSubsystem`.
- **Telemetry**: Use `@FROGSubsystem.telemetry("Label")` for automated NetworkTables reporting.
- **Tunables**: Use `@FROGSubsystem.tunable(default_value, "Label")` for configurations that should be adjustable via the dashboard.
- **Simulation**: Every motor-driven subsystem MUST implement `simulationPeriodic()` to update motor sim states. Use `DCMotorSim` where a plant is available, or voltage-based models otherwise.

## 4. Phoenix 6 Hardware Configuration
Use the FROGlib HAL in `roborio/FROGlib/ctre.py` to manage Phoenix 6 devices:
- **Factories**: Use `get_frog_talon_config()` and `get_frog_cancoder_config()` to get base configurations with FROG safe defaults (e.g., 40A supply / 60A stator current limits).
- **Method Chaining**: Chain configuration methods directly (e.g., `get_frog_talon_config().with_motor_output(MOTOR_OUTPUT_CWP_BRAKE).with_slot0(slot_config)`).
- **Identity Decoupling**: Pass CAN IDs, CAN bus names, and motor names directly into the `FROGTalonFX` and `FROGCanCoder` constructors, NOT into the config objects.
- **Signal Profiles**: Use `FROGTalonFX.SignalProfile` to optimize CAN bus utilization (e.g., `SWERVE_DRIVE`, `FLYWHEEL`, `POSITION_MM`).

## 5. Development Standards
- **Decoupled Architecture**: Systems should be decoupled; use `Triggers` and `Suppliers` for inter-subsystem interaction.
- **Testing**: Use `pytest` in `roborio/tests/` to validate all changes.
- **Pathing**: Use PathPlanner for autonomous and `PathfindToPath` for dynamic alignment.
- **Validation**: After any code change, run `cd roborio ; ..\.venv\Scripts\python.exe -m pytest` to verify the logic.

## 6. Shell Environment Awareness
The execution environment can vary between Ubuntu WSL, Windows Command Line, PowerShell, and Git Bash on Windows. Always verify the current OS and shell environment (e.g., using `uname -a`, checking environment variables, etc.) before running shell commands to ensure proper syntax and pathing.
