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


#!/usr/bin/env python3
"""
系统状态监控GUI - 彻底解决白框问题
"""

import sys
import os

# 设置Qt属性，避免WebEngine警告
from PySide2.QtCore import Qt
if hasattr(Qt, 'AA_ShareOpenGLContexts'):
    from PySide2.QtWidgets import QApplication
    QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)

from PySide2.QtWidgets import QApplication, QTextBrowser, QLabel
from PySide2.QtUiTools import QUiLoader
from PySide2.QtCore import QFile, QIODevice, QTimer
import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus

def load_ui_file(ui_file_path):
    """加载UI文件"""
    ui_file = QFile(ui_file_path)
    if not ui_file.open(QIODevice.ReadOnly):
        print(f"错误: 无法打开UI文件 {ui_file_path}")
        return None
    
    loader = QUiLoader()
    window = loader.load(ui_file)
    ui_file.close()
    
    if not window:
        print(f"错误: 加载UI文件失败")
        return None
    
    return window

class SimpleStatusNode(Node):
    """ROS2节点，订阅系统状态"""
    
    def __init__(self, update_callback):
        super().__init__('simple_gui_node')
        self.update_callback = update_callback
        
        # 订阅系统状态话题
        self.subscription = self.create_subscription(
            SystemStatus,"sys_status",
            self.callback,
            10
        )
        
        self.get_logger().info("GUI节点已启动")
    
    def callback(self, msg):
        """收到消息时的回调"""
        try:
            # 格式化数据 - 确保键名与UI控件名对应
            data = {
                'cpu': f"{msg.cpu_percent:.1f}%",
                'memory': f"{msg.memory_percent:.1f}%",
                'memory_total': f"{msg.memory_total/1e9:.2f} GB",
                'memory_available': f"{msg.memory_available/1e9:.2f} GB",
                'net_sent': f"{msg.net_sent:.2f} MB",
                'net_recv': f"{msg.net_recv:.2f} MB",
                'host': msg.host_name,
            }
            
            # 调用UI更新函数
            self.update_callback(data)
            
        except Exception as e:
            self.get_logger().error(f"处理数据时出错: {e}")

def initialize_textbrowsers(window):
    """初始化所有QTextBrowser，确保它们能正常显示"""
    from PySide2.QtWidgets import QTextBrowser
    import time
    
    textbrowsers = window.findChildren(QTextBrowser)
    
    # 为每个TextBrowser设置明显的样式和初始文本
    for browser in textbrowsers:
        name = browser.objectName()
        
        # 设置清晰的样式，确保可见
        browser.setStyleSheet("""
            QTextBrowser {
                background-color: #FFFFFF;
                color: #000000;
                font-size: 16px;
                font-weight: bold;
                border: 2px solid #336699;
                border-radius: 5px;
                padding: 5px;
                min-height: 40px;
                min-width: 300px;
            }
        """)
        
        # 设置初始文本
        browser.setText(f"{name}: 等待数据...")
        
        # 设置只读（虽然默认就是只读）
        browser.setReadOnly(True)
        
        # 设置文本居中对齐
        from PySide2.QtCore import Qt
        browser.setAlignment(Qt.AlignCenter)
        
        print(f"初始化 {name}: 大小={browser.width()}x{browser.height()}")
    
    return len(textbrowsers)

def create_ui_updater(window):
    """创建UI更新器，返回更新函数"""
    from PySide2.QtWidgets import QTextBrowser
    
    # 预先查找所有需要的控件
    controls = {
        'cpu': window.findChild(QTextBrowser, "cpu_percent"),
        'memory': window.findChild(QTextBrowser, "memory_percent"),
        'memory_total': window.findChild(QTextBrowser, "memory_total"),
        'memory_available': window.findChild(QTextBrowser, "memory_available"),
        'net_sent': window.findChild(QTextBrowser, "net_sent"),
        'net_recv': window.findChild(QTextBrowser, "net_recv"),
    }
    
    # 如果控件没找到，尝试其他查找方式
    for key in controls:
        if not controls[key]:
            controls[key] = window.findChild(QTextBrowser, key)
    
    def update_ui(data):
        """更新UI数据"""
        import time
        from PySide2.QtCore import Qt
        
        print(f"[{time.strftime('%H:%M:%S')}] 更新UI数据")
        
        # 更新CPU
        if controls['cpu'] and 'cpu' in data:
            controls['cpu'].setText(data['cpu'])
            print(f"  CPU: {data['cpu']}")
        
        # 更新内存百分比
        if controls['memory'] and 'memory' in data:
            controls['memory'].setText(data['memory'])
            print(f"  内存: {data['memory']}")
        
        # 更新总内存
        if controls['memory_total'] and 'memory_total' in data:
            controls['memory_total'].setText(data['memory_total'])
        
        # 更新可用内存
        if controls['memory_available'] and 'memory_available' in data:
            controls['memory_available'].setText(data['memory_available'])
        
        # 更新网络发送
        if controls['net_sent'] and 'net_sent' in data:
            controls['net_sent'].setText(data['net_sent'])
        
        # 更新网络接收
        if controls['net_recv'] and 'net_recv' in data:
            controls['net_recv'].setText(data['net_recv'])
        
        # 强制处理所有待处理事件
        QApplication.processEvents()
        
        # 检查控件是否可见
        for name, widget in controls.items():
            if widget:
                if not widget.isVisible():
                    print(f"警告: {name} 控件不可见")
                if widget.height() == 0 or widget.width() == 0:
                    print(f"警告: {name} 控件大小为0")

    return update_ui

def main():
    """主函数"""
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    
    # 设置应用程序样式
    app.setStyle("Fusion")
    
    # 加载UI文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ui_file_path = os.path.join(script_dir, "sys_status.ui")
    print(f"加载UI文件: {ui_file_path}")
    
    if not os.path.exists(ui_file_path):
        print(f"UI文件不存在: {ui_file_path}")
        print("当前目录:", os.getcwd())
        print("尝试查找文件...")
        for root, dirs, files in os.walk('.'):
            if 'sys_status.ui' in files:
                ui_file_path = os.path.join(root, 'sys_status.ui')
                print(f"找到UI文件: {ui_file_path}")
                break
    
    window = load_ui_file(ui_file_path)
    
    if not window:
        print("加载UI文件失败")
        return 1
    
    window.setWindowTitle("系统状态监控")
    
    # 初始化所有TextBrowser
    count = initialize_textbrowsers(window)
    print(f"初始化了 {count} 个QTextBrowser")
    
    # 创建UI更新器
    update_ui = create_ui_updater(window)
    
    # 显示窗口
    window.show()
    
    # 先更新一次，显示初始化数据
    init_data = {
        'cpu': '0.0%',
        'memory': '0.0%',
        'memory_total': '0.0 GB',
        'memory_available': '0.0 GB',
        'net_sent': '0.0 MB',
        'net_recv': '0.0 MB',
        'host': '等待数据...'
    }
    update_ui(init_data)
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建ROS2节点
    node = SimpleStatusNode(update_ui)
    
    # 使用定时器处理ROS2事件（非阻塞）
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)  # 每10ms处理一次
    
    print("窗口已显示，等待ROS2数据...")
    
    # 运行应用
    result = app.exec_()
    
    # 清理
    node.destroy_node()
    rclpy.shutdown()
    
    return result

if __name__ == '__main__':
    sys.exit(main())