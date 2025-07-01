import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication
from pyqt_ros_gui.logic.joint_sender import JointSender
from pyqt_ros_gui.ui.joint_gui import JointGUI
import json

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_joint_publisher')
        self.publisher = self.create_publisher(String, 'processed_joint_states', 10)
        self.subscription = self.create_subscription(String, 'processed_joint_states', self.listener_callback, 10)
        self.gui = None

    def send_joint_commands(self, joint_data):
        msg = String()
        msg.data = json.dumps(joint_data)
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent joint data: {msg.data}")

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if self.gui:
                self.gui.update_joint_states({k: v['position'] for k, v in data.items()})
        except Exception as e:
            self.get_logger().error(f"Failed to parse incoming JSON: {e}")

def main():
    rclpy.init()
    node = GUINode()
    logic = JointSender(node)
    
    app = QApplication([])
    logic = JointSender(node)


    gui = JointGUI(sender=node, logic=logic)
    node.gui = gui  # 💡关键绑定 GUI 实例回传
    logic.ui = gui  # 必须有这行！！
    gui.show()

    # ROS2 和 PyQt5 协同运行
    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    app.exec()
    node.destroy_node()
    rclpy.shutdown()
