#!/usr/bin/env python3
import sys
import rospy
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QFont, QColor, QIcon, QPixmap
from std_msgs.msg import Int32

class DrivingModeUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.current_mode = 0  # 0: Standard, 1: Max
        self.init_ros()
        self.init_ui()
        
    def init_ros(self):
        """Initialize ROS node and publisher"""
        try:
            rospy.init_node('driving_mode_ui', anonymous=True)
            self.mode_publisher = rospy.Publisher('/driving_mode', Int32, queue_size=10)
            rospy.loginfo("Driving Mode UI initialized")
            self.ros_enabled = True
        except Exception as e:
            print(f"ROS initialization failed: {e}")
            print("Running in standalone mode (no ROS)")
            self.mode_publisher = None
            self.ros_enabled = False
        
    def init_ui(self):
        """Initialize UI components"""
        self.setWindowTitle("AIM Driving Mode")
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet(self.get_stylesheet())
        
        # Get script directory for images
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Title
        title_label = QLabel("Autonomous Driving Mode Selection")
        title_label.setFont(QFont("Arial", 24, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # Description
        desc_label = QLabel("Select lane change and overtaking mode")
        desc_label.setFont(QFont("Arial", 12))
        desc_label.setAlignment(Qt.AlignCenter)
        desc_label.setStyleSheet("color: #888;")
        main_layout.addWidget(desc_label)
        
        # Mode selection buttons layout
        buttons_layout = QHBoxLayout()
        buttons_layout.setSpacing(30)
        
        # Standard Mode Button
        self.standard_btn = self.create_mode_button(
            "Standard Mode",
            "Standard Mode - Safe and conservative driving",
            0,
            "standard_.png"
        )
        buttons_layout.addWidget(self.standard_btn)
        
        # Max Mode Button
        self.max_btn = self.create_mode_button(
            "Max Mode",
            "Max Mode - Aggressive overtaking",
            1,
            "mad_max_.png"
        )
        buttons_layout.addWidget(self.max_btn)
        
        main_layout.addLayout(buttons_layout)
        
        # Current mode display
        self.status_label = QLabel("Current Mode: Standard")
        self.status_label.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: #0066CC; padding: 10px;")
        main_layout.addWidget(self.status_label)
        
        # Information frame
        info_frame = QFrame()
        info_frame.setStyleSheet("background-color: #F5F5F5; border-radius: 10px; padding: 15px;")
        info_layout = QVBoxLayout()
        
        info_text = QLabel(
            " Standard Mode : Safe and conservative path following\n"
            " Max Mode      : Aggressive lane change and overtaking"
        )
        info_text.setFont(QFont("Arial", 11))
        info_text.setAlignment(Qt.AlignLeft)
        info_layout.addWidget(info_text)
        info_frame.setLayout(info_layout)
        main_layout.addWidget(info_frame)
        
        main_layout.addStretch()
        main_widget.setLayout(main_layout)
        
        # Set initial mode to Standard
        self.set_mode(0)
    
    def create_mode_button(self, text, description, mode_value, image_filename):
        """Create a mode selection button with image"""
        btn = QPushButton()
        btn.setMinimumSize(300, 200)
        btn.setToolTip(description)
        btn.clicked.connect(lambda: self.set_mode(mode_value))
        
        # Try to load image from script directory
        image_path = os.path.join(self.script_dir, image_filename)
        if os.path.exists(image_path):
            pixmap = QPixmap(image_path)
            # Scale image to fit button
            scaled_pixmap = pixmap.scaledToHeight(150, Qt.SmoothTransformation)
            icon = QIcon(scaled_pixmap)
            btn.setIcon(icon)
            btn.setIconSize(QSize(280, 150))
            btn.setText(f"\n{text}")
            btn.setLayoutDirection(Qt.LeftToRight)
        else:
            # Fallback: show text if image not found
            btn.setText(text)
            btn.setFont(QFont("Arial", 14, QFont.Bold))
            print(f"[Warning] Image not found: {image_path}")
        
        btn.setStyleSheet(self.get_button_style(False))
        return btn
    
    def set_mode(self, mode):
        """Set driving mode and publish"""
        self.current_mode = mode
        
        # Update button styles
        standard_active = mode == 0
        max_active = mode == 1
        
        self.standard_btn.setStyleSheet(self.get_button_style(standard_active))
        self.max_btn.setStyleSheet(self.get_button_style(max_active))
        
        # Update status label
        mode_text = "Standard" if mode == 0 else "Max"
        self.status_label.setText(f"Current Mode: {mode_text}")
        
        # Always publish mode to ROS
        msg = Int32()
        msg.data = mode
        self.mode_publisher.publish(msg)
        
        # Console output
        rospy.loginfo(f"✓ Publishing to /driving_mode: {mode} ({mode_text})")
        print(f"\n{'='*50}")
        print(f"[PUBLISHED] Mode = {mode} | {mode_text}")
        print(f"{'='*50}\n")
    
    def get_button_style(self, active):
        """Return button stylesheet based on active state"""
        if active:
            return """
                QPushButton {
                    background-color: #0066CC;
                    color: white;
                    border: 3px solid #0066CC;
                    border-radius: 15px;
                    font-weight: bold;
                    padding: 15px;
                }
                QPushButton:hover {
                    background-color: #0052A3;
                    border: 3px solid #0052A3;
                }
                QPushButton:pressed {
                    background-color: #003D7A;
                    border: 3px solid #003D7A;
                }
            """
        else:
            return """
                QPushButton {
                    background-color: #E8E8E8;
                    color: #333;
                    border: 3px solid #CCCCCC;
                    border-radius: 15px;
                    font-weight: bold;
                    padding: 15px;
                }
                QPushButton:hover {
                    background-color: #D8D8D8;
                    border: 3px solid #BBBBBB;
                }
                QPushButton:pressed {
                    background-color: #C8C8C8;
                    border: 3px solid #AAAAAA;
                }
            """
    
    def get_stylesheet(self):
        """Return main window stylesheet"""
        return """
            QMainWindow {
                background-color: #FFFFFF;
            }
            QLabel {
                color: #333333;
            }
        """
    
    def closeEvent(self, event):
        """Handle window close event"""
        if self.ros_enabled:
            rospy.loginfo("Driving Mode UI closed")
            rospy.signal_shutdown("Window closed")
        else:
            print("[UI] Driving Mode UI closed")
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = DrivingModeUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
