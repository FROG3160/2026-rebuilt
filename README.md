# FROG 3160 - 2026 FRC Robot (Rebuilt)

This repository contains the RobotPy-based code for Team 3160's 2026 FRC robot. Built on the **Commands2** framework and the **Phoenix 6** API, this project features a custom Hardware Abstraction Layer (HAL) through subclassing common phoenix6 classes.  This gives us a design enabling rapid development of new subsystems, robust telemetry, and simulation testing.

## 🏗️ Architecture Overview

The codebase is organized into a modular, command-based structure that leverages Python's dynamic capabilities to reduce boilerplate and enforce consistent patterns across all robot mechanisms.

### Core Technical Stack
- **Framework**: [RobotPy](https://robotpy.readthedocs.io/) (WPILib 2026.2.1)
- **Motor Control**: CTRE Phoenix 6
- **Vision**: PhotonLib + AprilTags
- **Path Planning**: PathPlannerLib
- **Analysis**: AdvantageScope (.wpilog + .hoot files)

---

## 🐸 FROGlib: Custom Hardware Abstraction

The heart of our robot is `FROGlib`, our custom library that simplifies hardware interactions and automates complex tasks.

### Core Components
*   [**`FROGSubsystem`**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/FROGlib/subsystem.py): Our primary base class that uses Python descriptors and decorators (`@telemetry`, `@tunable`) to automate NetworkTables publishing and data logging without manual boilerplate.
*   [**`FROGTalonFX`**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/FROGlib/ctre.py): A specialized wrapper for Phoenix 6 `TalonFX` motors that standardizes configuration and integrates high-fidelity physics simulation.
*   [**`SwerveChassis`**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/FROGlib/swerve.py): A modular chassis controller handling kinematics, odometry, and discretized chassis speeds.
*   [**`FROGXboxDriver`**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/FROGlib/xbox.py): An enhanced driver controller class featuring built-in slew-rate limiting, alliance-aware field orientation, and standard deadband handling.
*   [**`FROGPoseEstimator`**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/FROGlib/vision.py): A vision-fusion helper that integrates multiple PhotonVision cameras with the swerve odometry.
*   [**`FROGDetector`**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/FROGlib/vision.py): A specialized class for handling object detection (e.g., game piece detection) and triggering automation based on target proximity.

---

## 🏎️ Integrated Motor Physics Simulation

A defining feature of our codebase is the integration of physics-based simulation directly into the [**`FROGTalonFX`**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/FROGlib/ctre.py) wrapper.

### Advantages of Our Implementation:
1.  **Hardware-Agnostic Validation**: We use `DCMotorSim` and `LinearSystemId` to model the physical properties (moment of inertia, gearing, motor constants) of every mechanism. 
2.  **Identical Control Paths**: The robot logic is the same in simulation and on the actual robot. It applies `VoltageOut` or `VelocityVoltage` requests, and the `FROGTalonFX` sim state updates the sensors (Encoder position/velocity) based on physics, rather than just "jumping to" the setpoint.  This has enabled us to verify control logic without testing on actual hardware.  For example, we were able to verify that the feed motors for our shooter didn't kick on until the flywheel was up to speed.
3.  **Complex Interaction Testing**: We can simulate vision-assisted shooting while moving. The simulation handles the robot's momentum and the latency of vision measurements, allowing us to tune our [**motion compensation**](https://github.com/FROG3160/2026-rebuilt/blob/main/roborio/subsystems/drive.py) logic entirely in software.
4.  **Unit Testing Rigor**: Our `pytest` suites run against these physics models, catching logic regressions that a simple mock-based test would miss.

---

## 📊 Enhanced Logging

### Global .hoot Logging
We prioritize high-fidelity data. `SignalLogger` is centralized in `robotInit`, capturing high-frequency (CAN-timestamped) data into `.hoot` files for both **real robot** and **simulation** modes. This allows for deep analysis of motor performance and control loops in AdvantageScope.

### Centralized Telemetry Root
All robot data is organized under the `FROGSubsystems` NetworkTable root. This structure ensures that AdvantageScope and custom dashboards have a predictable, clean path for all mechanism states, command statuses, and tunables.

---

## 🚀 Getting Started

### Prerequisites
- Python 3.12+
- [RobotPy](https://robotpy.readthedocs.io/en/stable/install/index.html)

### Running Tests
```powershell
# From the project root
.\.venv\Scripts\python.exe -m robotpy test
```

### Starting Simulation
```powershell
cd roborio
..\.venv\Scripts\python.exe -m robotpy sim
```

---
**Team 3160 - FROG** | *Figure It Out*
