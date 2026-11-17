# FR5 实物 → RViz 只读同步

目标是让 RViz 模型跟随真实机械臂的六轴反馈。这个包不运行物理仿真，不下发使能、运动、IO、模式或速度指令，不启动 SDK 或官方驱动，不依赖 MoveIt、Gazebo、MuJoCo。

```text
真实机械臂 192.168.58.2
  → 已有官方 ros2_cmd_server
  → /nonrt_state_data（关节单位 deg）
  → fr5_rviz_sync/joint_state_bridge
  → /fr5_live/joint_states（关节单位 rad）
  → robot_state_publisher
  → /fr5_live/tf、/fr5_live/tf_static
  → RViz RobotModel（固定坐标 base_link）
```

## 网络与远程桌面

Ubuntu 的 Tailscale 地址由用户确认为 100.80.160.79，用户 dj306。Tailscale 用于 Windows 与 Ubuntu 的管理连接，不是机器人控制器的新地址；机器人仍使用 192.168.58.2。

所有 ROS 节点和 RViz 均在 Ubuntu 上运行，不需要 DDS 跨 Tailscale 通信，也不需要在 Windows 安装 ROS。保持 Ubuntu 到机器人网段的原有网络路径，确认 `ip route get 192.168.58.2` 和 ping 可用。

VS Code Remote SSH 能显示代码和终端，但普通 SSH 会话没有图形桌面。通过 Ubuntu 本机显示器或现有远程桌面（如向日葵）查看 RViz。不要盲目设置 DISPLAY=:0；需要由已登录桌面的会话提供显示和认证环境。

## 上传与编译

Windows PowerShell：

```powershell
scp "C:\Users\Administrator\Documents\ChatGPT\真机\outputs\fr5_rviz_sync.tar.gz" dj306@100.80.160.79:~/
```

也可把压缩包通过已连接的 VS Code 传到 Ubuntu 的 /home/dj306。Ubuntu 终端：

```bash
cd /data/Projects/fr5_ros2
tar -xzf ~/fr5_rviz_sync.tar.gz -C src
bash src/fr5_rviz_sync/scripts/build.sh
```

原工作空间脚本 scripts/02_build.sh 固定选择原四包，不会编译本包。已有依赖为 fairino_msgs、fairino_description、robot_state_publisher、rviz2。缺少后两者时通过 Ubuntu APT 安装 ros-humble-robot-state-publisher、ros-humble-rviz2。

## 启动

先确认没有旧的离线仿真或重复本包实例在运行。已有官方驱动正常运行时不要重复启动第二个连接实例。

终端 A：若官方驱动尚未运行：

```bash
cd /data/Projects/fr5_ros2
source scripts/03_source.bash
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
ros2 run fairino_hardware ros2_cmd_server
```

终端 B（VS Code 或桌面终端均可）：

```bash
cd /data/Projects/fr5_ros2
bash src/fr5_rviz_sync/scripts/start_live.sh
```

脚本在有图形显示时启动 RViz；无图形显示时只启动同步和 TF，不会因缺 DISPLAY 导致整套流程报错。

如果 B 是 VS Code 的 SSH 终端，再到 Ubuntu 本机/远程桌面打开终端 C：

```bash
cd /data/Projects/fr5_ros2
bash src/fr5_rviz_sync/scripts/view_rviz.sh
```

这条命令仅启动 RViz，不重复启动同步桥。RViz 显示 FR5 Live Robot 与 TF，Fixed Frame 是 base_link，Description Topic 是 /fr5_live/robot_description；已配置独立 TF 话题，因此不会混用 fr5_sim 的模型变换。

## 验证

在另一个 Ubuntu 终端：

```bash
cd /data/Projects/fr5_ros2
source src/fr5_rviz_sync/scripts/source_live.bash
ros2 topic info /nonrt_state_data -v
ros2 topic echo /fr5_live/joint_states --once
timeout 10s ros2 topic hz /fr5_live/joint_states
```

预期 joint_states 中 name 为 j1..j6，position 是弧度（例如 90°→1.5707963），velocity/effort 留空。桥接节点在有效数据到达时最多发布 50 Hz；输入更慢时输出相应更慢。timeout 到 10 秒返回 124 属于正常超时终止。

桥接日志首次收到数据会打印 Live feedback received；每约 5 秒显示 LIVE、收发计数、六轴角度及错误码。超过 2 秒没有有效数据会警告 NO LIVE FEEDBACK，并停止发布新数据；RViz 可能保留旧姿态，不能把静止画面当成连接正常。恢复反馈后自动继续。

先在机械臂静止时对照真实姿态和日志。若已在现场通过正常方式产生小幅运动，可观察相应关节是否同步；本包自身不触发运动。若只有关节数值对而形状不一致，继续校准 URDF 零位、方向和模型，不盲目增大运动幅度。

## 范围与精度

这是关节反馈驱动的数字模型显示，既没有 Gazebo/MuJoCo 接触动力学，也没有 MoveIt 执行控制。沿用当前匹配版本官方 URDF，修正 wrist2 碰撞元素 origins 拼写时只改内存，不改厂商源文件。

接收时间作为 JointState 时间戳，不提供控制器硬件时钟对齐或端到端延迟保证。不伪造速度/力矩，不夹紧或环绕真实关节角。未添加工具、相机、力传感器模型；基坐标、零位和 wrist3_link 到实际法兰/TCP 的差异仍需后续核验。

## 验证记录

本地通过 4 项独立测试：真实状态样本和 1°变化的单位转换、符号/多圈保留、非法数据拒绝、含硬件插件 URDF 拒绝。另静态检查官方消息字段、URDF 网格引用、RViz 配置和启动节点。

本地 Windows 未安装 ROS2 Humble，暂不能声称完整 ROS2/RViz 运行已通过。当前自动 SSH 认证失败，服务器部署及图形显示需用户通过已连接的 VS Code/桌面终端完成并回传验证。

资料：https://github.com/ros/robot_state_publisher/tree/humble 与 https://github.com/ros2/rviz/tree/humble 。
