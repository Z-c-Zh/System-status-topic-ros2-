# Copyright 2025 Zechang Zhou

#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import rclpy
from status_interfaces.msg import SystemStatus
from rclpy.node import Node
import psutil
import platform
from PySide2.QtWidgets import QApplication,QMainWindow,QPushButton,QPlainTextEdit,QMessageBox
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile,QTimer
# builtin_interfaces/Time stamp
# string host_name
# float32 cpu_percent
# float32 memory_percent
# float32 memory_total
# float32 memory_available
# float64 net_sent #MB
# float64 net_recv #MB
class SysStatusPub(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.status_publisher_=self.create_publisher(
            SystemStatus,"sys_status",10
        )
        self.timer_=self.create_timer(1.0,self.timer_callback)
    def timer_callback(self):
        cpu_percent=psutil.cpu_percent()
        memory_info=psutil.virtual_memory()
        net_io_counters=psutil.net_io_counters()

        msg=SystemStatus()
        msg.stamp=self.get_clock().now().to_msg()
        msg.host_name=platform.node()
        msg.cpu_percent = float(cpu_percent)  # 确保是浮点数
        msg.memory_percent = float(memory_info.percent)  # 确保是浮点数
        msg.memory_total = float(memory_info.total)  # 将整数转换为浮点数
        msg.memory_available = float(memory_info.available)  # 将整数转换为浮点数
        msg.net_sent = float(net_io_counters.bytes_sent / 1024 / 1024)  # 转换为MB/s
        msg.net_recv = float(net_io_counters.bytes_recv / 1024 / 1024)  # 转换为MB/s

        self.get_logger().info(f"publish:{str(msg)}")
        self.status_publisher_.publish(msg)
def main():
    rclpy.init()
    node=SysStatusPub("sys_status_pub")
    rclpy.spin(node)
    rclpy.shutdown()