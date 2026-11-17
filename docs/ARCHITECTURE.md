# FR5 项目目录与后续开发架构

## 当前事实与边界

- Ubuntu / ROS 2 Humble；FR5 V6.0；Web 版本 v3.9.2，控制器版本 V3.9.12-QX。
- 真机 IP：192.168.57.2；此前有线网卡地址：192.168.57.10。
- 已验证非实时状态反馈约 100 Hz、RViz 姿态同步，以及一条 MoveIt 真机轨迹成功执行。
- 空闲时持续 ServoJ 导致超时的问题已加入伺服结束/重新启动逻辑；对应 C++ 测试通过，但连续两条真机轨迹仍待复测。
- 最近启动出现 RPC 返回 -2，原因尚未查清。文件迁移和离线编译通过不代表连接问题已经解决。

## 实际存放位置

```text
/data/
├── Projects/fr5_ros2/           # colcon 工作空间实体
│   ├── src/                    # 驱动、接口、模型、MoveIt、状态显示
│   ├── scripts/                # 构建、环境加载、启动入口
│   ├── configs/legacy_fr5/      # 迁移保留的 ROS 隔离配置
│   ├── vendor/                 # SDK 的相对引用，COLCON_IGNORE 保留
│   ├── docs/
│   │   ├── ARCHITECTURE.md
│   │   └── migration/          # 哈希清单、清理记录、原始资料、验证日志
│   ├── tests/
│   ├── outputs/
│   └── build、install、log/     # 在新路径生成，可重新构建
└── SDKs/fairino/
    ├── cpp/                    # robot.h、类型/错误定义、2.2.7 和 2.3.2 动态库
    └── python/                 # 原官方 Python SDK、示例、扩展库、许可证
```

工作空间是实际目录，不是指回 home 的链接。只有工作空间内部的 SDK 引用使用相对链接。官方 SDK 文件尽量保持原样，不在 vendor 源码里放业务逻辑。Python SDK 的完整分发与原项目运行时内容经过比对，使用同一份 `linux/fairino/Robot.py`。

## 当前运行链路

RViz 目标交互 → MoveIt 规划/碰撞检查 → FollowJointTrajectory → fairino5_controller → FairinoMoveItHardware → 官方 C++ SDK → FR5。

实际关节反馈沿 SDK → ros2_control → joint_state_broadcaster → robot_state_publisher/MoveIt 返回。ROS 侧使用弧度；SDK 边界转换为度。轨迹控制周期为 125 Hz，ServoJ 标称周期为 8 ms。

同一时间只允许一套运动控制程序持有真机控制权。原 `ros2_cmd_server` 用于独立状态/命令调试，不与 MoveIt 真机入口或其他软件点动并行运行。启动入口不自动清报警、切模式或上使能；使用前确认自动、使能、无报警。

## 后续开发分层（设计，尚未实现）

1. **任务层**：增加 `fr5_tasks`，组织抓取、搬运、触觉实验等状态机；通过 MoveIt 执行动作，不直接调用 ServoJ。
2. **感知层**：相机、标定、检测、触觉处理作为独立包；只发布观测与目标，不绕过任务层控制机械臂。
3. **规划层**：继续使用 MoveIt 配置，统一机器人模型、TCP/工具、负载、工作坐标系、障碍物和速度/加速度约束。
4. **硬件层**：`fairino_hardware` 负责 SDK 会话、关节接口、反馈校验和故障上报；SDK 单独保留版本，不在业务包里分散创建机器人连接。
5. **记录与回放**：实验参数放 configs，运行输出放 outputs；大型数据集、模型分别使用 /data/Datasets、/data/Models，并记录版本及来源。
6. **测试**：先做不连接机械臂的单位测试、模型/配置检查，再做受控实机验证。仿真如后续需要，应独立工作空间和启动入口，不覆盖真机配置。

故障处理应明确区分连接故障、控制周期超时、SDK 拒绝、关节越限、路径跟踪误差。禁止自动重放已经失败的轨迹；恢复后重新读取实际姿态并规划。软件停止受通信和线程调度约束，不能替代现场急停。

## 常用入口

重新构建（停止驱动后，在干净终端使用）：

```bash
bash /data/Projects/fr5_ros2/scripts/build.sh
```

Ubuntu 桌面启动 MoveIt 真机：

```bash
bash /data/Projects/fr5_ros2/scripts/start_moveit.sh
```

独立状态/命令服务（先关闭 MoveIt）：

```bash
bash /data/Projects/fr5_ros2/scripts/start_state.sh
```

新终端会提供 `fr5moveit` 和 `fr5state` 别名。关闭时在启动终端 Ctrl+C 并等待后台进程退出，关闭 RViz 窗口本身不等于关闭控制器。

## 迁移清理原则

源码、官方 SDK、现有说明文档以及旧桌面/压缩包等尚可能包含设计信息的资料保留在 /data。旧 build/install/log、字节码缓存和历史补丁备份不作为活动工程迁移。旧 home 中的对应副本、过期入口和 Gazebo 用户缓存，仅在文件校验、干净环境编译、模型加载及动态库检查通过后删除。

不卸载系统 ROS/MoveIt，也不删除其他用户项目。`resources.py` 继续用于资源目录管理；它不承担本次真实迁移。迁移脚本不会连接机械臂或执行轨迹。

## Inverse kinematics

IKFast is primary with KDL fallback. See [IKFAST.md](IKFAST.md) for validation, limits and installation records.
