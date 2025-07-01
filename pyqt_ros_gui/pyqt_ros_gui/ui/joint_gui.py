from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QScrollArea, QFormLayout, QGroupBox, QTabWidget, QComboBox
)
from PyQt5.QtCore import QTimer
from datetime import datetime
import json, os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointGUI(QWidget):
    def __init__(self, sender, logic):
        super().__init__()
        self.sender = sender
        self.logic = logic
        self.joint_inputs = {}
        self.joint_state_labels = {}
        self.latest_joint_state = {}
        self.last_update_label = QLabel("Last update: N/A")
        self.actions = self.load_actions()

        self.joint_map = {
            "l_thumb_joint": "Left Thumb", "l_index_joint": "Left Index", "l_middle_joint": "Left Middle",
            "l_ring_joint": "Left Ring", "l_pinky_joint": "Left Pinky",
            "l_thumb_tip_joint": "Left Thumb Tip", "l_index_tip_joint": "Left Index Tip", "l_middle_tip_joint": "Left Middle Tip",
            "l_ring_tip_joint": "Left Ring Tip", "l_pinky_tip_joint": "Left Pinky Tip",
            "l_wrist_joint": "Left Wrist", "l_elbow_joint": "Left Elbow", "l_shoulder_joint": "Left Shoulder",
            "neck_joint": "Neck", "head_joint": "Head",
            "r_thumb_joint": "Right Thumb", "r_index_joint": "Right Index", "r_middle_joint": "Right Middle",
            "r_ring_joint": "Right Ring", "r_pinky_joint": "Right Pinky",
            "r_thumb_tip_joint": "Right Thumb Tip", "r_index_tip_joint": "Right Index Tip", "r_middle_tip_joint": "Right Middle Tip",
            "r_ring_tip_joint": "Right Ring Tip", "r_pinky_tip_joint": "Right Pinky Tip",
            "r_wrist_joint": "Right Wrist", "r_elbow_joint": "Right Elbow", "r_shoulder_joint": "Right Shoulder",
        }

        self.left_joints = [k for k in self.joint_map if k.startswith("l_")]
        self.center_joints = [k for k in self.joint_map if k.startswith("neck") or k.startswith("head")]
        self.right_joints = [k for k in self.joint_map if k.startswith("r_")]

        self.setStyleSheet("""
            QWidget {
                background-color: #1e1e2f;
                color: #f0f0f0;
                font-family: 'Segoe UI', 'Helvetica Neue', sans-serif;
                font-size: 13px;
            }
            QLineEdit {
                background-color: #2a2a3d;
                border: 1px solid #444;
                padding: 6px;
                color: #fff;
                border-radius: 4px;
            }
            QPushButton {
                background-color: #3f51b5;
                color: white;
                padding: 6px 12px;
                border: none;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #5c6bc0;
            }
            QComboBox {
                background-color: #2a2a3d;
                color: white;
                padding: 4px;
            }
        """)

        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_action_combo)
        self.timer.start(1000)

    def load_actions(self):
        try:
            path = os.path.join(get_package_share_directory('pyqt_ros_gui'), 'actions', 'actions.json')
            with open(path, 'r') as f:
                return json.load(f)
        except:
            return []

    def init_ui(self):
        self.setWindowTitle("Joint Controller GUI")
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        self.init_joint_tab()
        self.init_monitor_tab()
        self.init_actions_tab()

    def init_joint_tab(self):
        widget = QWidget()
        outer_layout = QVBoxLayout(widget)
        columns_layout = QHBoxLayout()

        for group, joints in [("Left Arm", self.left_joints), ("Head / Neck", self.center_joints), ("Right Arm", self.right_joints)]:
            box = QGroupBox(group)
            form = QFormLayout()
            for joint in joints:
                label = QLabel(self.joint_map[joint])
                edit = QLineEdit()
                self.joint_inputs[joint] = edit
                form.addRow(label, edit)
            box.setLayout(form)
            columns_layout.addWidget(box)

        outer_layout.addLayout(columns_layout)

        btn_row = QHBoxLayout()
        send_btn = QPushButton("Send")
        clear_btn = QPushButton("Clear")
        send_btn.clicked.connect(lambda: self.logic.send_data(self.joint_inputs, self.sender))
        clear_btn.clicked.connect(lambda: self.logic.clear_inputs(self.joint_inputs))
        btn_row.addWidget(send_btn)
        btn_row.addWidget(clear_btn)
        outer_layout.addLayout(btn_row)

        self.tabs.addTab(widget, "Joint Control")

    def init_monitor_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        columns = QHBoxLayout()

        for title, joint_list in [("Left Arm", self.left_joints), ("Head / Neck", self.center_joints), ("Right Arm", self.right_joints)]:
            box = QGroupBox(title)
            vbox = QVBoxLayout()
            for joint in joint_list:
                label = QLabel(f"{self.joint_map[joint]}: N/A")
                self.joint_state_labels[joint] = label
                vbox.addWidget(label)
            box.setLayout(vbox)
            columns.addWidget(box)

        layout.addLayout(columns)
        layout.addWidget(self.last_update_label)

        btns = QHBoxLayout()
        record = QPushButton("Record")
        stop = QPushButton("Stop and Send")
        record.clicked.connect(lambda: self.logic.start_recording_placeholder(self.latest_joint_state))
        stop.clicked.connect(lambda: self.logic.send_latest_state(self.latest_joint_state, self.sender))
        btns.addWidget(record)
        btns.addWidget(stop)
        layout.addLayout(btns)

        self.tabs.addTab(widget, "Monitor")

    def init_actions_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        layout.addStretch(1)
        center = QVBoxLayout()
        label = QLabel("Select Action:")
        label.setStyleSheet("font-size: 14px;")
        self.action_combo = QComboBox()
        self.action_combo.addItems([a['name'] for a in self.actions])
        run_btn = QPushButton("Run Action")
        run_btn.clicked.connect(lambda: self.logic.run_action(
            self.action_combo.currentText(), self.actions, self.sender))

        center.addWidget(label)
        center.addWidget(self.action_combo)
        center.addWidget(run_btn)
        center.setSpacing(12)
        center.setContentsMargins(50, 0, 50, 0)

        layout.addLayout(center)
        layout.addStretch(2)

        self.tabs.addTab(widget, "Actions")

    def update_joint_states(self, joint_state_dict):
        self.latest_joint_state = joint_state_dict
        for joint, label in self.joint_state_labels.items():
            pos = joint_state_dict.get(joint, None)
            label.setText(f"{self.joint_map[joint]}: {pos:.2f}" if pos is not None else f"{self.joint_map[joint]}: N/A")
        self.last_update_label.setText("Last update: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    def update_action_combo(self):
        new_actions = self.load_actions()
        if new_actions != self.actions:
            current = self.action_combo.currentText()
            self.actions = new_actions
            self.action_combo.clear()
            self.action_combo.addItems([a['name'] for a in self.actions])
            if current in [a['name'] for a in self.actions]:
                self.action_combo.setCurrentText(current)
