import sys
import rclpy
from rclpy.node import Node
from PySide6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                               QLabel, QDoubleSpinBox, QPushButton, QTabWidget, 
                               QSlider)
from PySide6.QtCore import QTimer, Qt
from std_msgs.msg import Float64MultiArray
from poker_interfaces.msg import TargetPose, MotorFeedback, TargetJoints

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.pub_pose = self.create_publisher(TargetPose, '/target_pose', 10)
        self.pub_joints = self.create_publisher(TargetJoints, '/target_joints', 10)
        self.sub_feedback = self.create_subscription(MotorFeedback, '/motor_feedback', self.feedback_cb, 10)
        self.sub_ik = self.create_subscription(Float64MultiArray, '/ik_solution', self.ik_cb, 10)
        
        self.feedback_data = [0.0]*6
        self.latest_ik = None

    def feedback_cb(self, msg):
        self.feedback_data = msg.positions

    def ik_cb(self, msg):
        # Callback when controller publishes calculated angles
        self.latest_ik = msg.data

    def send_pose(self, x, y, z, p, r, t):
        msg = TargetPose()
        msg.x, msg.y, msg.z = x, y, z
        msg.pitch, msg.roll = p, r
        msg.duration = t
        self.pub_pose.publish(msg)

    def send_joints(self, joints, duration):
        msg = TargetJoints()
        msg.joints = [float(j) for j in joints]
        msg.duration = duration
        self.pub_joints.publish(msg)

class DashboardWidget(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("Poker Arm Dashboard")
        self.resize(450, 550) # Made slightly wider
        
        main_layout = QVBoxLayout()
        
        # --- Tabs ---
        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_cartesian_tab(), "Cartesian Control")
        self.tabs.addTab(self.create_joint_tab(), "Joint Control")
        main_layout.addWidget(self.tabs)

        # --- Status ---
        self.lbl_status = QLabel("Status: Ready")
        main_layout.addWidget(self.lbl_status)

        self.setLayout(main_layout)
        
        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)

    def create_cartesian_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        
        self.cart_inputs = {}
        labels = ['X (m)', 'Y (m)', 'Z (m)', 'Pitch (rad)', 'Roll (rad)', 'Duration (s)']
        defaults = [0.3, 0.0, 0.2, 0.0, 0.0, 2.0]
        
        for lbl, default in zip(labels, defaults):
            h = QHBoxLayout()
            h.addWidget(QLabel(lbl))
            spin = QDoubleSpinBox()
            
            # --- FIX: Custom Range for Duration ---
            if 'Duration' in lbl:
                spin.setRange(0.1, 60.0)
            else:
                spin.setRange(-2.0, 2.0)
                
            spin.setSingleStep(0.01)
            spin.setValue(default)
            self.cart_inputs[lbl] = spin
            h.addWidget(spin)
            layout.addLayout(h)
        
        # --- NEW: IK Result Label ---
        self.lbl_ik_result = QLabel("Calculated IK: [None]")
        self.lbl_ik_result.setWordWrap(True)
        layout.addWidget(self.lbl_ik_result)
        
        btn = QPushButton("Move to Pose")
        btn.clicked.connect(self.on_move_cartesian)
        layout.addWidget(btn)
        
        widget.setLayout(layout)
        return widget

    def create_joint_tab(self):
        widget = QWidget()
        layout = QVBoxLayout()
        
        self.sliders = []
        self.spinboxes = []
        
        for i in range(6):
            h = QHBoxLayout()
            h.addWidget(QLabel(f"J{i+1}"))
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-314, 314) 
            slider.setValue(0)
            
            spin = QDoubleSpinBox()
            spin.setRange(-3.14, 3.14)
            spin.setSingleStep(0.1)
            
            slider.valueChanged.connect(lambda val, s=spin: s.setValue(val/100.0))
            spin.valueChanged.connect(lambda val, s=slider: s.setValue(int(val*100)))
            
            self.sliders.append(slider)
            self.spinboxes.append(spin)
            
            h.addWidget(slider)
            h.addWidget(spin)
            layout.addLayout(h)
            
        btn = QPushButton("Move Joints")
        btn.clicked.connect(self.on_move_joints)
        layout.addWidget(btn)
        widget.setLayout(layout)
        return widget

    def on_move_cartesian(self):
        vals = [s.value() for s in self.cart_inputs.values()]
        self.node.send_pose(*vals)
        self.lbl_status.setText(f"Status: Sending Cartesian Target...")

    def on_move_joints(self):
        joints = [s.value() for s in self.spinboxes]
        self.node.send_joints(joints, 2.0)
        self.lbl_status.setText("Status: Sending Joint Command...")

    def spin_ros(self):
        if not rclpy.ok():
            self.timer.stop()
            self.close()
            return
        
        try:
            rclpy.spin_once(self.node, timeout_sec=0)
            
            # Update IK Display
            if self.node.latest_ik is not None:
                ik_str = ", ".join([f"{x:.2f}" for x in self.node.latest_ik])
                self.lbl_ik_result.setText(f"Calculated IK:\n[{ik_str}]")
                
        except KeyboardInterrupt:
            self.close()
        except Exception:
            pass

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

def main():
    rclpy.init()
    ros_node = DashboardNode()
    app = QApplication(sys.argv)
    win = DashboardWidget(ros_node)
    win.show()
    try:
        exit_code = app.exec()
    except KeyboardInterrupt:
        exit_code = 0
    finally:
        win.timer.stop()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()