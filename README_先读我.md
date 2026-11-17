# FR5 ROS2 Humble 标准工作空间

目标：Ubuntu 22.04 x86_64、ROS2 Humble、FR5 V6.0、机器人软件 v3.9.2。
src/ 下四个 fairino 包复制自用户提供的官方 ROS2 版本源码；fr5_python 是本次新增的 ament_python 包。
vendor/ 内为用户提供的匹配版本 Python SDK 源码，保留 Robot.py 及支持文件，不包含 Windows 包或编译缓存。
官方代码与其现有许可声明保持原样；新增包的许可证为 Apache-2.0。

## 目录格式

```text
fr5_ros2_ws/
├── src/
│   ├── fairino_msgs/
│   ├── fairino_hardware/
│   ├── fairino_description/
│   ├── fairino5_v6_moveit2_config/
│   └── fr5_python/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/fr5_python
│       ├── fr5_python/__init__.py
│       ├── fr5_python/state_echo.py
│       ├── launch/state.launch.py
│       └── config/state.yaml
├── vendor/fairino_python_sdk/fairino/
├── scripts/
├── build/       # colcon 编译生成，不预建
├── install/     # colcon 编译生成，不预建
└── log/         # colcon 编译生成，不预建
```

## 安装与编译（在 Ubuntu 上）

不覆盖已有 ~/fr5_ros2_ws。若已有该目录，先解压到另一个目录检查；工作空间脚本使用自身实际路径。

```bash
cd ~/fr5_ros2_ws
bash scripts/01_setup_ubuntu.sh
bash scripts/02_build.sh
source scripts/03_source.bash
bash scripts/04_check.sh
```

01 会安装系统软件包，可能下载较多文件并要求 sudo 密码；需要 Ubuntu 访问软件源与 GitHub。
未预装 Humble 时，按官方 ros2-apt-source 方法配置软件源。安装使用 --no-remove，若依赖需要删除现有包会停止，不自动处理。
不运行全系统升级，不修改网卡、SSH、.bashrc，不自动启动机械臂或 MoveIt2。
02 默认只编译基础包；FR5 MoveIt2 配置保留在 src 中，尚不编译或启动。后续安装 MoveIt2 相关依赖后单独配置。
只使用 /opt/ros/humble 的 ros2_control 二进制依赖；若已有自编译 ros2_control，先另开干净终端，避免混用不匹配的覆盖层。

## Python 状态订阅

```bash
source ~/fr5_ros2_ws/scripts/03_source.bash
ros2 launch fr5_python state.launch.py
```

这个节点只订阅 /nonrt_state_data，不会启动机器人驱动或发送指令；无驱动时会提示等待反馈。
仅在机器人网络已核验后，另一个终端手动启动官方驱动：

```bash
source ~/fr5_ros2_ws/scripts/03_source.bash
ros2 run fairino_hardware ros2_cmd_server
```

官方驱动含可接收控制指令的 ROS2 服务。初次连通测试先运行状态订阅，不调用运动、使能和 IO 输出指令。
机器人 IP 是 192.168.58.2；Ubuntu SSH IP 当前为 192.168.10.159，两者不同。不要为机器人网络盲目修改当前 SSH 网卡。

## 验证范围

本地检查 Python 语法、包目录、XML、消息字段、打包和基础示例逻辑；未在服务器执行安装、colcon 编译或真机测试。
服务器仍要求交互式密码，本次未能进行自动 SSH 操作。

## 官方来源

- https://github.com/FAIR-INNOVATION/frcobot_ros2/releases/tag/V3.0.0_RobotV3.9.2
- https://github.com/FAIR-INNOVATION/fairino-python-sdk/releases/tag/v2.2.2_robot3.9.2
- https://fairino-doc-zhs.readthedocs.io/latest/ROSGuide/ros2guide.html
- https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst
