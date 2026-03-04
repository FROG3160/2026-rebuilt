# FROG 3160 - Driver Controls

This document outlines the control mapping for the driver's Xbox controller during tele-operated mode.

## 🕹️ Primary Drive Controls

| Control | Action |
| --- | --- |
| **Left Stick** | Controls the robot's translation (forward, backward, strafe). This is field-oriented. |
| **Right Stick (X-Axis)** | Controls the robot's rotation. |
| **Start Button** | Resets the robot's odometry and pose to its initial starting position. |
| **Right Bumper (Hold)** | Runs a pathfinding routine to a scoring position, dynamically selected based on the robot's current field zone. |

## 🎯 Shooting & Scoring

| Control | Action |
| --- | --- |
| **A Button (Hold)** | Activates the "Aim and Fire" sequence. The robot will: <br> 1. Aim at the target. <br> 2. Deploy the hood. <br> 3. Spin up the flywheel to the correct speed based on distance. <br> 4. Feed the game piece into the shooter. <br> Releasing the button retracts the hood and stops the sequence. <br> *Note: This only works in designated "safe-to-shoot" zones on the field.* |

## 📦 Intake & Cargo Handling

| Control | Action |
| --- | --- |
| **Y Button (Toggle)** | Toggles the intake on/off (runs forward to collect game pieces). |
| **X Button (Hold)** | Ejects all cargo. Runs the intake, feeder, and hopper in reverse. |
| **Automatic Intake** | The intake will automatically run forward when the front-facing sensor detects a game piece is close. |
| **Automatic Hopper** | The hopper motor runs automatically: <br> - **Forward** when the intake or feeder is running forward. <br> - **Backward** when the intake or feeder is running backward. |

## 🧗 Climber

| Control | Action |
| --- | --- |
| **Left Bumper (Press)** | Deploys the climber mechanism to its set position. |
| **Left Trigger (Hold)** | Stows the climber mechanism. |