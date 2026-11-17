# Changelog

All notable changes to this project are documented here.

The project is under active development.

## [Unreleased]

### Planned

- Standardize workspace documentation.
- Improve SDK dependency management.
- Introduce separated build scripts.
- Standardize robot network configuration.
- Add automated ROS 2 Humble CI.
- Improve package-level testing.
- Prepare MuJoCo and SAC integration.
- Prepare gripper and tactile sensing integration.

## [0.1.0] - 2026-11-17

### Added

- Initial FR5 ROS 2 workspace.
- FR5 V6.0 robot description.
- MoveIt 2 configuration.
- Fairino hardware package.
- Custom ROS 2 messages and services.
- Python state utility.
- RViz real-time state synchronization.
- IKFast and KDL hybrid kinematics plugin.
- IKFast generation and validation tools.
- Installation, build and startup scripts.

### Changed

- Removed legacy FairinoHardwareInterface implementation.
- Removed legacy hardware plugin registration.
- Retained FairinoMoveItHardware as the primary real-robot control interface.
- Retained FairinoReadOnlyProbe for read-only diagnostics.

### Validation

- ROS 2 Humble builds completed for fairino_msgs and fairino_hardware.
- A MoveIt real-robot trajectory was executed successfully.
- MoveIt shutdown lifecycle issues remain under investigation.

### Notes

- Original workspace snapshot is preserved by the Git tag:
  pre-workspace-standardization.
- Planned features are not considered implemented until separately validated.
