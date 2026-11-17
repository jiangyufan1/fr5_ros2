# AGENTS.md

# FR5 ROS 2 AI Development Guidelines

This document defines mandatory rules for AI agents working on this repository.

## 1. Project Environment

- OS: Ubuntu 22.04 LTS
- ROS: ROS 2 Humble
- Robot: Fairino FR5 V6.0
- Controller software: V3.9.12-QX
- C++ standard: C++17
- Python: 3.10
- Build system: colcon + CMake

Do not change the ROS distribution or system Python version without authorization.

## 2. Project Packages

The workspace contains seven ROS 2 packages:

| Package | Responsibility |
|---|---|
| fairino_hardware | Hardware interfaces and SDK integration |
| fairino_msgs | Custom ROS messages and services |
| fairino_description | Robot URDF and meshes |
| fairino5_v6_moveit2_config | MoveIt 2 configuration |
| fairino5_ikfast_plugin | IKFast and KDL hybrid solver |
| fr5_python | Python utilities |
| fr5_rviz_sync | Robot state visualization synchronization |

Understand package dependencies before modifying interfaces.

## 3. Hardware Architecture

Current hardware components:

- FairinoMoveItHardware: Primary real-robot motion interface.
- FairinoReadOnlyProbe: Read-only diagnostic interface.
- ros2_cmd_server: Independent vendor command and state feedback service.

The legacy FairinoHardwareInterface has been removed.

Do not reintroduce it without an explicit architecture decision.

## 4. Robot Safety — Mandatory

The FR5 is physical equipment.

AI agents must never execute real-robot motion without explicit user authorization.

Do not automatically:

- Enable robot motors.
- Clear robot alarms.
- Change robot operating modes.
- Execute ServoJ commands.
- Execute motion trajectories.
- Execute IO output commands.
- Replay failed trajectories.
- Modify emergency-stop settings.

Before an authorized hardware test:

1. Confirm robot IP and network interface.
2. Confirm robot operating state.
3. Confirm the workspace is clear.
4. Confirm emergency stop accessibility.
5. Confirm only one motion-control process owns the robot.
6. Confirm explicit user authorization.

Software protection does not replace physical safety systems.

## 5. Single Motion Ownership

Only one motion-control program may own the FR5 at a time.

Never run MoveIt motion control simultaneously with:

- ros2_cmd_server motion commands.
- Other ros2_control hardware instances.
- Vendor SDK motion programs.
- External robot jogging software.

Read-only diagnostic processes must not send motion commands.

## 6. SDK Management

Fairino SDK files are maintained outside this repository.

Expected location:

/data/SDKs/fairino/

Do not:

- Modify vendor SDK source without authorization.
- Replace SDK binaries silently.
- Commit private credentials.
- Commit local SDK symlinks as substitutes for SDK installation.
- Assume a fresh Git clone contains the SDK.

Document SDK versions, sources and compatibility requirements.

## 7. Network Configuration

The robot has two network addresses:

- Wired: 192.168.57.2
- Wireless: 192.168.58.2

Do not hardcode machine-specific network settings in multiple source files.

Do not confuse robot IP addresses with Ubuntu SSH addresses.

Never modify network interfaces or routing without explicit authorization.

## 8. Git Workflow

Before modifying any file, execute:

```bash
git status
git branch --show-current
git log --oneline -5
Rules:
- Never overwrite uncommitted user changes.
- Never execute git reset --hard without authorization.
- Never force-push without authorization.
- Never delete branches or tags without authorization.
- Keep main stable.
- Use independent development branches.
- Use Pull Requests for meaningful changes.
## 9. Code Modification Rules
Before modifying code:
1. Read the target file.
2. Inspect related dependencies.
3. Search all references to affected symbols.
4. Inspect related launch files.
5. Identify hardware and runtime implications.
Prefer small, isolated changes.

Do not rename ROS topics, services, actions, parameters or package names without checking all consumers.

Preserve original vendor attribution and license information.
## 10. Build Requirements
Load ROS 2 Humble:source /opt/ros/humble/setup.bashBuild using colcon.

After modifying a package:
1. Build affected packages.
2. Inspect compiler warnings and errors.
3. Check package discovery.
4. Check launch configurations.
5. Run applicable tests.
Never claim a build passed without observing its actual result.

Never claim a hardware test passed based only on compilation.
## 11. Motion Control Constraints
Current hardware conventions:
- Six ordered joints: j1 through j6.
- ROS joint positions use radians.
- Fairino SDK joint positions use degrees.
- Nominal ServoJ period: 8 ms.
- Controller update frequency: 125 Hz.
Do not change joint order, units, timing, velocity limits or fault behavior without reviewing dependent components.

A nominal 8 ms period does not guarantee deterministic real-time execution.

Changes to ServoJ, feedback handling, fault handling and stopping behavior require dedicated review.
## 12. MoveIt and IKFast
The project uses IKFast as the primary inverse kinematics solver and KDL as fallback.

Do not:
- Remove KDL fallback without validation.
- Accept approximate solutions silently.
- Change joint limits without checking URDF and MoveIt configurations.
- Change planning groups without checking all consumers.
Preserve original URDF residual validation.
## 13. ROS 2 Interfaces
Before changing a topic, service or message:
- Search all publishers and subscribers.
- Search service clients and servers.
- Check message definitions.
- Check launch files.
- Check external consumers.
The /nonrt_state_data topic is part of the current state synchronization workflow.

Do not silently break existing interfaces.
## 14. Configuration and Security
Never commit:
- Passwords.
- GitHub tokens.
- Private keys.
- Personal credentials.
- Machine-specific secrets.
Keep example configuration files separate from actual local configuration.

Do not place tokens in source code, scripts or README files.
## 15. Testing Order
Use this validation order:
1. Static source inspection.
2. Syntax and formatting checks.
3. Unit tests.
4. Package compilation.
5. Plugin loading.
6. Launch validation.
7. Read-only hardware testing.
8. Explicitly authorized real-robot motion testing.
Do not skip directly to physical motion testing.
## 16. Documentation
When architecture or behavior changes, update relevant documentation.

Keep implemented features separate from planned features.

Document:
- Package responsibilities.
- SDK versions.
- Build dependencies.
- Launch procedures.
- Known limitations.
- Validation results.
## 17. Completion Report
Every task must report:
1. Files changed.
2. Reasons for changes.
3. Commands executed.
4. Build and test results.
5. Remaining risks.
6. Git branch and commit status.
Do not report unverified success.
