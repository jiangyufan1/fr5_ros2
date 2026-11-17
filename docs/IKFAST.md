# FR5 inverse kinematics

MoveIt uses `fr5_kinematics/IKFastWithKDL` from `fairino5_ikfast_plugin`.
IKFast is tried first. When no candidate passes bounds, consistency, original
URDF residual checks and the caller callback, KDL receives the remaining query
time. The configured query budget is 0.05 seconds total. No approximate solutions
are accepted. Collision rejection callbacks apply to both backends.

The raw IKFast solver has no solutions for the validated wrist-zero target set;
KDL fallback is intentional. The generation-only model snaps near-quarter-turn
angles; the original URDF remains unchanged. Both backends are checked against
the original model with 20 micrometre / 20 microradian residual thresholds.

Validation loads the actual installed plugin and tests 1001 ordinary targets,
101 wrist-zero targets and 108 near-limit targets with perturbed seeds. It also
checks callback rejection, consistency, invalid inputs and unreachable targets.
These tests do not establish full workspace coverage or trajectory continuity.
No hardware connection or motion test was performed during installation.

Run records and the previous kinematics configurations: /data/Projects/fr5_ros2/outputs/ikfast_hybrid_20261113_194021
