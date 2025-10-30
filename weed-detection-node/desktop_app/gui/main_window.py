#!/usr/bin/env python3
"""
Main Window for Weed Detection Desktop Application
"""

from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QTextEdit, QSplitter, QGroupBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QImage
import cv2
import numpy as np

from .ros_thread import ROS2Thread


class MainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.ros_thread = None
        self.init_ui()
        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Weed Detection Monitor - ArUco Marker Detection")
        self.setGeometry(100, 100, 1400, 800)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        
        # Top: Control panel
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)
        
        # Middle: Split view for images and coordinates
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Left: Camera feeds
        camera_widget = self.create_camera_widget()
        splitter.addWidget(camera_widget)
        
        # Right: Coordinates panel
        coord_widget = self.create_coordinates_widget()
        splitter.addWidget(coord_widget)
        
        splitter.setStretchFactor(0, 2)  # Camera feeds take 2/3
        splitter.setStretchFactor(1, 1)  # Coordinates take 1/3
        
        main_layout.addWidget(splitter)
        
        # Status bar
        self.statusBar().showMessage("Ready - Click 'Start' to begin")
        
    def create_control_panel(self):
        """Create the control panel with buttons"""
        group = QGroupBox("Controls")
        layout = QHBoxLayout()
        
        # Start button
        self.start_btn = QPushButton("🚀 Start")
        self.start_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 14px; padding: 10px; }")
        self.start_btn.clicked.connect(self.start_ros)
        layout.addWidget(self.start_btn)
        
        # Stop button
        self.stop_btn = QPushButton("⏹ Stop")
        self.stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-size: 14px; padding: 10px; }")
        self.stop_btn.clicked.connect(self.stop_ros)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)
        
        # Status label
        self.status_label = QLabel("Status: Stopped")
        self.status_label.setStyleSheet("QLabel { font-size: 14px; font-weight: bold; }")
        layout.addWidget(self.status_label)
        
        layout.addStretch()
        group.setLayout(layout)
        return group
        
    def create_camera_widget(self):
        """Create widget for camera feeds"""
        group = QGroupBox("Camera Feeds")
        layout = QVBoxLayout()
        
        # Raw camera feed
        raw_label = QLabel("📷 Raw Camera:")
        raw_label.setStyleSheet("QLabel { font-size: 12px; font-weight: bold; }")
        layout.addWidget(raw_label)
        
        self.raw_image_label = QLabel()
        self.raw_image_label.setMinimumSize(640, 360)
        self.raw_image_label.setStyleSheet("QLabel { background-color: #2b2b2b; border: 2px solid #555; }")
        self.raw_image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.raw_image_label.setText("No image")
        layout.addWidget(self.raw_image_label)
        
        # Annotated camera feed
        annotated_label = QLabel("🎯 Annotated (with markers):")
        annotated_label.setStyleSheet("QLabel { font-size: 12px; font-weight: bold; }")
        layout.addWidget(annotated_label)
        
        self.annotated_image_label = QLabel()
        self.annotated_image_label.setMinimumSize(640, 360)
        self.annotated_image_label.setStyleSheet("QLabel { background-color: #2b2b2b; border: 2px solid #555; }")
        self.annotated_image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.annotated_image_label.setText("No image")
        layout.addWidget(self.annotated_image_label)
        
        group.setLayout(layout)
        return group
        
    def create_coordinates_widget(self):
        """Create widget for coordinates display"""
        group = QGroupBox("Detected Markers")
        layout = QVBoxLayout()
        
        # Markers count
        self.markers_count_label = QLabel("Markers: 0")
        self.markers_count_label.setStyleSheet("QLabel { font-size: 16px; font-weight: bold; color: #4CAF50; }")
        layout.addWidget(self.markers_count_label)
        
        # Coordinates text area
        self.coordinates_text = QTextEdit()
        self.coordinates_text.setReadOnly(True)
        self.coordinates_text.setStyleSheet("""
            QTextEdit { 
                background-color: #1e1e1e; 
                color: #00ff00; 
                font-family: 'Courier New', monospace; 
                font-size: 11px;
                border: 2px solid #555;
            }
        """)
        self.coordinates_text.setText("Waiting for marker data...")
        layout.addWidget(self.coordinates_text)
        
        group.setLayout(layout)
        return group
        
    def start_ros(self):
        """Start ROS2 connection"""
        try:
            self.status_label.setText("Status: Starting...")
            self.statusBar().showMessage("Connecting to ROS2 nodes...")
            
            # Create and start ROS2 thread
            self.ros_thread = ROS2Thread()
            self.ros_thread.raw_image_signal.connect(self.update_raw_image)
            self.ros_thread.annotated_image_signal.connect(self.update_annotated_image)
            self.ros_thread.coordinates_signal.connect(self.update_coordinates)
            self.ros_thread.start()
            
            # Update UI
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.status_label.setText("Status: Running ✓")
            self.status_label.setStyleSheet("QLabel { font-size: 14px; font-weight: bold; color: #4CAF50; }")
            self.statusBar().showMessage("Connected - Receiving data...")
            
        except Exception as e:
            self.status_label.setText(f"Status: Error - {str(e)}")
            self.statusBar().showMessage(f"Error: {str(e)}")
            
    def stop_ros(self):
        """Stop ROS2 connection"""
        if self.ros_thread:
            self.ros_thread.stop()
            self.ros_thread.wait()
            self.ros_thread = None
            
        # Update UI
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("Status: Stopped")
        self.status_label.setStyleSheet("QLabel { font-size: 14px; font-weight: bold; }")
        self.statusBar().showMessage("Stopped")
        
        # Clear displays
        self.raw_image_label.setText("No image")
        self.annotated_image_label.setText("No image")
        self.coordinates_text.setText("Waiting for marker data...")
        self.markers_count_label.setText("Markers: 0")
        
    def update_raw_image(self, cv_image):
        """Update raw camera image display"""
        self.display_image(cv_image, self.raw_image_label)
        
    def update_annotated_image(self, cv_image):
        """Update annotated camera image display"""
        self.display_image(cv_image, self.annotated_image_label)
        
    def display_image(self, cv_image, label):
        """Display OpenCV image in QLabel"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Create QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            
            # Scale to fit label while maintaining aspect ratio
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(label.size(), Qt.AspectRatioMode.KeepAspectRatio, 
                                         Qt.TransformationMode.SmoothTransformation)
            
            label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            print(f"Error displaying image: {e}")
            
    def update_coordinates(self, coords_data):
        """Update coordinates display"""
        try:
            markers = coords_data.get('markers', [])
            marker_count = len(markers)
            
            # Update count
            self.markers_count_label.setText(f"Markers: {marker_count}")
            
            # Format coordinates text
            text = f"Timestamp: {coords_data.get('timestamp', 'N/A')}\n\n"
            
            for marker in markers:
                marker_id = marker.get('id', '?')
                center = marker.get('center', {})
                
                text += f"═══ Marker ID {marker_id} ═══\n"
                text += f"  Center: ({center.get('x', 0)}, {center.get('y', 0)})\n"
                
                corners = marker.get('corners', [])
                if corners:
                    text += f"  Corners:\n"
                    for i, corner in enumerate(corners, 1):
                        text += f"    {i}. ({corner.get('x', 0)}, {corner.get('y', 0)})\n"
                
                text += "\n"
            
            self.coordinates_text.setText(text)
            
        except Exception as e:
            self.coordinates_text.setText(f"Error parsing coordinates: {e}")
            
    def closeEvent(self, event):
        """Handle window close event"""
        if self.ros_thread:
            self.ros_thread.stop()
            self.ros_thread.wait()
        event.accept()

