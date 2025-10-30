#!/usr/bin/env python3
"""
PyQt6 Desktop Application for Weed Detection
Main entry point
"""

import sys
from PyQt6.QtWidgets import QApplication
from gui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    
    # Set application properties
    app.setApplicationName("Weed Detection Monitor")
    app.setOrganizationName("Precision Farming Robot")
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    # Run application
    sys.exit(app.exec())


if __name__ == '__main__':
    main()

