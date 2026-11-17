# FR5 当前项目与 ACT 接入交接

## 给接手 AI 的任务

基于包内当前源码，将 ACT 项目接入 FR5。这里假定 ACT 指 Action Chunking with Transformers；ACT 仓库、版本、模型权重和配置尚未提供，请先确认这些输入，再选择适配接口。不要把尚未实现的接口当作现有功能。

首先阅读 `Projects/fr5_ros2/docs/ARCHITECTURE.md`、`docs/IKFAST.md`、`scripts/start_moveit.sh`、对应 launch、控制器配置与 `src/fairino_hardware`。以实际源码和配置为准。列出 ACT 观测、动作与机器人接口的映射，再实现独立适配包和离线测试。

## 已完成与待验证

- Ubuntu、ROS 2 Humble，工程实体 `/data/Projects/fr5_ros2`；官方 SDK `/data/SDKs/fairino`。
- FR5 V6.0，控制器地址 `192.168.57.2`。有线通信、约 100 Hz 状态反馈、RViz 姿态同步和一条真机 MoveIt 轨迹此前成功。
- MoveIt 当前求解器为 `fr5_kinematics/IKFastWithKDL`，IKFast 优先，无有效解时 KDL 兜底。用户明确选择此方案。
- 2026-11-13 离线测试：普通目标 1001/1001、近限位目标 108/108、腕部零角度目标 101/101；IKFast 成功 1109 次，KDL 成功 101 次，错误解 0。回调拒绝、动作一致性限制、非法输入及不可达目标检查通过。
- 裸 IKFast 对生成模型的腕部零角度目标 0/101；不得移除兜底或将普通目标通过率视为全工作空间保证。
- 生成模型对接近 π/2 的角度做了归整，原 URDF 未改。组合插件按原 URDF 验证结果，误差阈值位置 2e-5 m、旋转 2e-5 rad。
- IKFast 接入后尚未验证真机运动。最近一次启动在硬件激活阶段 RPC code=-2 退出，不能将其认定为 IKFast 故障，也不能据此认定真机已可运行。
- 空闲伺服结束/再次启动及 watchdog 逻辑已有测试；连续两次真机轨迹执行仍需复测。

## 当前控制链路

RViz → MoveIt → FollowJointTrajectory → fairino5_controller → FairinoMoveItHardware → 官方 C++ SDK → FR5。

ROS 关节状态和动作单位为弧度，SDK 边界转换为度。按实际 joint_names 映射 j1..j6，不依赖反馈数组的偶然顺序。已配置 ServoJ 周期 8 ms / 125 Hz。ACT 推理频率与该硬件周期不是同一个概念。

接入时只保留一个硬件控制者。ACT 适配层应使用现有控制接口；不要并行创建另一条 SDK/ServoJ 连接。原 watchdog、跟踪误差、限位和故障锁定不因推理变慢而直接关闭。

## ACT 适配需要明确的契约

1. 数据输入：摄像头数量/标识、RGB 或 BGR、分辨率、裁剪、时间戳；关节反馈及夹爪状态的维度、单位与同步方法。包内不代表相机和夹爪已接通。
2. 动作输出：绝对关节角、关节增量或末端位姿；关节顺序、弧度/角度、夹爪通道、归一化统计、训练频率及坐标系。
3. 时序：chunk 长度、执行步数、时间聚合、旧动作丢弃、推理超时、启动实际姿态、暂停/取消与恢复。禁止在失联后继续重放过期 chunk。
4. 接口选择：高层目标可经 MoveIt 规划；高频关节 chunk 若直接发 FollowJointTrajectory，会绕过 MoveIt 规划阶段的碰撞检查，必须明确相应验证策略。不能假定每个预测点都应独立调用 MoveIt 规划。
5. 测试顺序：录制/合成观测驱动的离线推理 → 只记录动作、不连接硬件的适配测试 → 验证维度、单位、时间与限位 → 经用户安排后实机测试。

建议新增独立 `fr5_act_bridge` 包（名称为建议，尚未实现），将模型推理、观测同步、动作调度与 ROS 驱动分开。ACT/PyTorch 环境独立管理，不替换系统 ROS Python。模型和数据分别放 `/data/Models`、`/data/Datasets`，通过配置引用。

## 包结构与使用

- `Projects/fr5_ros2`：打包时的源码、脚本、配置、工具、测试和当前文档。
- `SDKs/fairino`：官方 SDK 文件及许可证；用于当前运行的动态库保留。
- `verification`：选择性保留已完成安装的报告；不是实机运行证明。
- `MANIFEST.json`：文件列表、大小、SHA-256，以及符号链接的原始目标记录。

不含 build/install/log、outputs 大型中间产物、旧备份、历史迁移资料、Git 历史、Python 环境、模型权重和数据集。项目的 SDK 符号链接在 ZIP 内展开为实际文件，便于跨平台查看；原文件和链接不会被修改。恢复时将 `Projects` 和 `SDKs` 放到 `/data` 下，再按项目构建脚本重建。路径变化需要相应调整脚本。

构建入口：`bash /data/Projects/fr5_ros2/scripts/build.sh`。
真机启动入口：`bash /data/Projects/fr5_ros2/scripts/start_moveit.sh`，不是离线仿真入口。

请向用户索取 ACT 仓库地址、模型配置/权重位置、相机/夹爪配置和动作数据定义；这些信息不能从机械臂工程中凭空推断。
