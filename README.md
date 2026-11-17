# FR5 ROS 2 Development Platform

基于 Ubuntu 22.04、ROS 2 Humble 和 Fairino FR5 V6.0 的机器人研发平台。

项目包含机器人模型、硬件驱动、MoveIt 2、IKFast、RViz 状态同步和 Python 工具。

## 1. Development Environment

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| Robot | Fairino FR5 V6.0 |
| Controller | V3.9.12-QX |
| Web | v3.9.2 |
| C++ | C++17 |
| Python | 3.10 |
| Build | colcon + CMake |
| Motion Planning | MoveIt 2 |
| IK | IKFast + KDL |

## 2. Architecture

```text
Application Layer
       |
       v
Task Planning
       |
       v
MoveIt 2
       |
       v
IKFast + KDL
       |
       v
ros2_control
       |
       v
FairinoMoveItHardware
       |
       v
Fairino C++ SDK
       |
       v
FR5 Robot
```

### State feedback flow

```text
FR5
 |
 v
Fairino SDK
 |
 v
ros2_control
 |
 +----> Joint State Broadcaster
 |              |
 |              v
 |          MoveIt / RViz
 |
 +----> /nonrt_state_data
                |
                v
           fr5_rviz_sync
                |
                v
               RViz
```


## 3. ROS 2 Packages

| Package | Responsibility |
|---|---|
| fairino_hardware | Hardware interfaces and command server |
| fairino_msgs | Custom messages and services |
| fairino_description | URDF and robot meshes |
| fairino5_v6_moveit2_config | MoveIt configuration |
| fairino5_ikfast_plugin | IKFast and KDL hybrid solver |
| fr5_python | Python utilities |
| fr5_rviz_sync | Real robot state synchronization |

## 4. Hardware Interfaces
- FairinoMoveItHardware: Primary real-robot motion interface.
- FairinoReadOnlyProbe: Read-only diagnostic interface.
- ros2_cmd_server: Independent vendor command and state feedback service.
Legacy FairinoHardwareInterface has been removed.

Only one motion-control program may control the robot at a time.
## 5. Repository Structure
fr5_ros2/
├── README.md
├── AGENTS.md
├── CONTRIBUTING.md
├── CHANGELOG.md
├── configs/
├── docs/
├── src/
├── scripts/
├── tests/
├── tools/
├── vendor/
└── .github/


## 6. SDK Dependencies
Fairino SDK files are maintained outside this repository.

Expected location:
/data/SDKs/fairino/
├── cpp/
│   ├── include/
│   └── lib/
└── python/
    └── linux/


SDK versions and dependencies must be documented.

Do not commit private credentials or local SDK symlinks as substitutes for dependency management.
## 7. Build
Load ROS 2:
source /opt/ros/humble/setup.bash


Build current baseline packages:
bash scripts/02_build.sh


Load workspace:
source scripts/03_source.bash


Check environment:
bash scripts/04_check.sh


Note: The current build script builds basic packages only. Full MoveIt and IKFast build support will be standardized separately.
## 8. Launch
MoveIt real-robot control:
bash scripts/start_moveit.sh


Independent state and command server:
bash scripts/start_state.sh


Read-only diagnostic:
source scripts/03_source.bash
ros2 launch scripts/fr5_readonly_probe.launch.py


Never run multiple motion-control programs against the same robot simultaneously.
## 9. Network
The robot has separate wired and wireless network addresses.
- Wired: 192.168.57.2
- Wireless: 192.168.58.2
Keep machine-specific configuration outside reusable source code.

Do not confuse robot network addresses with Ubuntu SSH addresses.
## 10. Development Roadmap
Planned extensions:
- FR5 task planning.
- Gripper integration.
- Tactile sensing.
- MuJoCo simulation.
- SAC-based control.
- Automated testing.
- Multi-computer development.
These are planned features, not claims of completed implementation.
## 11. Documentation
- Architecture
- IKFast
- AI Agent Rules
- Contributing
## 12. Official References
- https://github.com/FAIR-INNOVATION/frcobot_ros2
- https://github.com/FAIR-INNOVATION/fairino-python-sdk
- https://docs.ros.org/en/humble/
- https://moveit.picknik.ai/
