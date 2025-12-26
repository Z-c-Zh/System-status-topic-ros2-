📊 ROS2 系统状态监控工具
一个基于 ROS2 和 PySide2 的实时系统资源监控工具，可发布并可视化 CPU、内存、网络等关键指标。
（用于学习ros2节点与话题应用）

✨ 功能特性
多维度监控：实时采集并发布 CPU 使用率、内存占用、网络流量等系统状态。

松耦合架构：采用 ROS2 话题通信，发布者与 GUI 可视化界面分离，便于扩展和复用。

友好图形界面：基于 PySide2 开发的桌面 GUI，直观展示各项指标数据。

即插即用：提供完整的 ROS2 接口定义、发布者节点和即用型 GUI 程序。
示例界面：
<img width="741" height="618" alt="73ef93c76338a4f186e2153c87b1e68f" src="https://github.com/user-attachments/assets/05e906c0-f738-49b0-b5c8-a1e7a246a407" />

🚀 快速开始
前提条件
ROS2 Humble （已安装并配置好工作空间）

Python 3.8+

pip 包管理器

安装步骤
克隆仓库到 ROS2 工作空间的 src 目录

bash
cd ~/ros2_ws/src
git clone https://github.com/你的用户名/ros2_system_monitor.git
安装 Python 依赖

bash
cd ~/ros2_ws
pip install psutil PySide2
构建工作空间

bash
colcon build --packages-select system_monitor_pkg
激活工作空间

bash
source install/setup.bash
🖥️ 使用方法
1. 启动状态发布者
在新终端中运行以下命令，启动数据采集与发布节点：

bash
source ~/ros2_ws/install/setup.bash
ros2 run system_monitor_pkg status_publisher
该节点将开始每秒发布一次系统状态到 /sys_status 话题。

2. 启动 GUI 可视化界面
在另一个新终端中，启动图形化监控界面：

bash
source ~/ros2_ws/install/setup.bash
ros2 run system_monitor_pkg status_gui
启动后，GUI 窗口将显示并实时更新所有系统指标。

🔧 自定义接口
项目的核心消息接口定义在 status_interfaces/msg/SystemStatus.msg：

plaintext
builtin_interfaces/Time stamp
string host_name
float32 cpu_percent
float32 memory_percent
float32 memory_total
float32 memory_available
float64 net_sent #MB
float64 net_recv #MB
你可以根据需要修改此文件（修改后需重新 colcon build），以添加或删减监控字段。

🐛 故障排除
GUI 显示白框/无数据：

请确保已正确安装 PySide2：pip install PySide2

确认发布者节点正在运行：ros2 topic echo /sys_status

检查 GUI 节点是否有错误输出。

colcon build 失败：

确认功能包结构正确，特别是 package.xml 和 setup.py 中已声明对 rclpy、psutil 等依赖。

无法导入自定义接口：

确保工作空间已重新构建并激活：source install/setup.bash

📄 许可证
本项目采用 Apache License 2.0 开源许可证。详见 LICENSE 文件。

👥 贡献
欢迎提交 Issue 和 Pull Request 来报告问题或贡献代码。

☕ 致谢
感谢 ROS2 社区提供的出色机器人中间件框架。
感谢 psutil 库提供跨平台的系统信息获取能力。
