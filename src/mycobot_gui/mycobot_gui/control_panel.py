#!/usr/bin/env python3
import sys
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QGridLayout, QDoubleSpinBox
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject

ARM_JOINTS = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]
GRIPPER_JOINT = "gripper_joint"
BASE_FRAME = "g_base"
TOOL_LINK = "gripper_base"
OBJECT_ID = "pick_cube"
OBJECT_SIZE = 0.04

ARM_LIMIT_DEG = 165
GRIPPER_MIN = -0.7
GRIPPER_MAX = 0.15
APPROACH_OFFSET_Z = 0.08


class RosBridge(QObject):
    joint_state_received = Signal(dict)
    status_message = Signal(str)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('mycobot_gui_node')

        self.arm_client = ActionClient(self.node, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self.node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.auto_pick_place_client = self.node.create_client(Trigger, '/auto_pick_place')

        self.latest_joint_state = {}
        self.node.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.object_pose = {"x": 0.15, "y": -0.10, "z": 0.02}
        self.object_attached = False

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        rclpy.spin(self.node)

    def _joint_state_cb(self, msg: JointState):
        data = dict(zip(msg.name, msg.position))
        self.latest_joint_state = data
        self.joint_state_received.emit(data)

    def trigger_pick_place(self):
        if not self.auto_pick_place_client.wait_for_service(timeout_sec=1.0):
            self.status_message.emit('Status: Pick & Place service unavailable')
            return False

        req = Trigger.Request()
        future = self.auto_pick_place_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        result = future.result()
        if result is None:
            self.status_message.emit('Status: Pick & Place failed: request timed out')
            return False
        if result.success:
            self.status_message.emit(f'Status: {result.message}')
            return True
        self.status_message.emit(f'Status: Pick & Place failed: {result.message}')
        return False

    def send_arm_goal(self, joint_names, positions, duration_sec=1.5):
        if not self.arm_client.wait_for_server(timeout_sec=0.5):
            self.status_message.emit('arm_group_controller not available')
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        goal.trajectory.points = [point]
        self.arm_client.send_goal_async(goal)

    def send_gripper_goal(self, position, duration_sec=0.8):
        if not self.gripper_client.wait_for_server(timeout_sec=0.5):
            self.status_message.emit('gripper_controller not available')
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        goal.trajectory.points = [point]
        self.gripper_client.send_goal_async(goal)


class JointSlider(QWidget):
    def __init__(self, label, min_deg, max_deg, on_change):
        super().__init__()
        self.on_change = on_change
        layout = QHBoxLayout()
        self.label = QLabel(f"{label}:")
        self.label.setFixedWidth(90)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_deg))
        self.slider.setMaximum(int(max_deg))
        self.slider.setValue(0)
        self.value_label = QLabel("0.0°")
        self.value_label.setFixedWidth(60)
        self.slider.valueChanged.connect(self._changed)
        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value_label)
        self.setLayout(layout)

    def _changed(self, val):
        self.value_label.setText(f"{val:.1f}°")
        self.on_change()

    def value_deg(self):
        return self.slider.value()

    def set_value_deg(self, val):
        self.slider.blockSignals(True)
        self.slider.setValue(int(val))
        self.value_label.setText(f"{val:.1f}°")
        self.slider.blockSignals(False)


class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("myCobot 280 — Control Panel")
        self.resize(700, 750)

        central = QWidget()
        main_layout = QVBoxLayout()

        self.status_label = QLabel("Status: connecting...")
        main_layout.addWidget(self.status_label)

        arm_box = QGroupBox("Joint Control (target)")
        arm_layout = QVBoxLayout()
        self.sliders = []
        for i, _ in enumerate(ARM_JOINTS):
            s = JointSlider(f"J{i+1}", -ARM_LIMIT_DEG, ARM_LIMIT_DEG, self._send_arm)
            arm_layout.addWidget(s)
            self.sliders.append(s)
        arm_box.setLayout(arm_layout)
        main_layout.addWidget(arm_box)

        live_box = QGroupBox("Live Joint State (actual, from robot)")
        live_grid = QGridLayout()
        self.live_labels = {}
        for i, jname in enumerate(ARM_JOINTS + [GRIPPER_JOINT]):
            name_label = QLabel(f"J{i+1}" if i < 6 else "Gripper")
            val_label = QLabel("--")
            live_grid.addWidget(name_label, i, 0)
            live_grid.addWidget(val_label, i, 1)
            self.live_labels[jname] = val_label
        live_box.setLayout(live_grid)
        main_layout.addWidget(live_box)

        grip_box = QGroupBox("Gripper")
        grip_layout = QHBoxLayout()
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setMinimum(0)
        self.gripper_slider.setMaximum(100)
        self.gripper_slider.setValue(100)
        self.gripper_slider.valueChanged.connect(self._send_gripper)
        grip_layout.addWidget(QLabel("Closed"))
        grip_layout.addWidget(self.gripper_slider)
        grip_layout.addWidget(QLabel("Open"))
        grip_box.setLayout(grip_layout)
        main_layout.addWidget(grip_box)

        obj_box = QGroupBox("Object Position (pick_cube)")
        obj_layout = QGridLayout()
        self.obj_current_label = QLabel("Current: (0.150, -0.100, 0.020)")
        obj_layout.addWidget(self.obj_current_label, 0, 0, 1, 3)

        obj_layout.addWidget(QLabel("Place X:"), 1, 0)
        self.place_x = QDoubleSpinBox(); self.place_x.setRange(-0.3, 0.3); self.place_x.setSingleStep(0.01); self.place_x.setValue(0.15)
        obj_layout.addWidget(self.place_x, 1, 1)

        obj_layout.addWidget(QLabel("Place Y:"), 2, 0)
        self.place_y = QDoubleSpinBox(); self.place_y.setRange(-0.3, 0.3); self.place_y.setSingleStep(0.01); self.place_y.setValue(0.10)
        obj_layout.addWidget(self.place_y, 2, 1)

        obj_layout.addWidget(QLabel("Place Z:"), 3, 0)
        self.place_z = QDoubleSpinBox(); self.place_z.setRange(0.0, 0.3); self.place_z.setSingleStep(0.01); self.place_z.setValue(0.02)
        obj_layout.addWidget(self.place_z, 3, 1)

        obj_box.setLayout(obj_layout)
        main_layout.addWidget(obj_box)

        btn_layout = QHBoxLayout()
        home_btn = QPushButton("HOME")
        home_btn.clicked.connect(self._go_home)
        open_btn = QPushButton("Open Gripper")
        open_btn.clicked.connect(lambda: self._set_gripper_preset(100))
        close_btn = QPushButton("Close Gripper")
        close_btn.clicked.connect(lambda: self._set_gripper_preset(0))
        pnp_btn = QPushButton("PICK && PLACE")
        pnp_btn.clicked.connect(self._pick_and_place)
        btn_layout.addWidget(home_btn)
        btn_layout.addWidget(open_btn)
        btn_layout.addWidget(close_btn)
        btn_layout.addWidget(pnp_btn)
        main_layout.addLayout(btn_layout)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

        self.bridge.joint_state_received.connect(self._on_joint_state)
        self.bridge.status_message.connect(self._on_status_message)

        self.timer = QTimer()
        self.timer.timeout.connect(self._update_status)
        self.timer.start(1000)

    def _update_status(self):
        arm_ok = self.bridge.arm_client.server_is_ready()
        grip_ok = self.bridge.gripper_client.server_is_ready()
        self.status_label.setText(
            f"Status: arm={'OK' if arm_ok else 'waiting'} | gripper={'OK' if grip_ok else 'waiting'}"
        )

    def _on_status_message(self, msg):
        self.status_label.setText(msg)

    def _send_arm(self):
        positions = [math.radians(s.value_deg()) for s in self.sliders]
        self.bridge.send_arm_goal(ARM_JOINTS, positions)

    def _send_gripper(self, val):
        frac = val / 100.0
        pos = GRIPPER_MIN + frac * (GRIPPER_MAX - GRIPPER_MIN)
        self.bridge.send_gripper_goal(pos)

    def _set_gripper_preset(self, val):
        self.gripper_slider.setValue(val)

    def _go_home(self):
        for s in self.sliders:
            s.set_value_deg(0)
        self.bridge.send_arm_goal(ARM_JOINTS, [0.0] * len(ARM_JOINTS), duration_sec=2.0)

    def _on_joint_state(self, data: dict):
        for jname, label in self.live_labels.items():
            if jname in data:
                deg = math.degrees(data[jname])
                label.setText(f"{deg:.1f}°  ({data[jname]:.3f} rad)")

    def _pick_and_place(self):
        threading.Thread(target=self._pick_and_place_seq, daemon=True).start()

    def _pick_and_place_seq(self):
        self.bridge.status_message.emit('Status: Pick & Place started')
        self.bridge.trigger_pick_place()

    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()


def main():
    bridge = RosBridge()
    app = QApplication(sys.argv)
    window = MainWindow(bridge)
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
