#!/usr/bin/env python3
"""
6D Signal USB Serial Master Controller with Keyboard Control
Sends 6-byte signal to Arduino for motor control via USB Serial
Control using arrow keys: UP, DOWN, LEFT, RIGHT
Format: [Dir1, Speed1, Dir2, Speed2, Dir3, Speed3]
"""

import serial
import time
import sys
import threading
from pynput import keyboard
import curses
from time import sleep

# Direction constants
DIR_FORWARD = 0
DIR_BACKWARD = 1
DIR_STOP = 2

class MotorController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """Initialize USB Serial connection"""
        print("=" * 50)
        print("6D Signal Motor Controller - USB Serial Master")
        print("=" * 50)
        
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset after serial connection
            print(f"DEBUG: Serial initialized on port {port}")
            print(f"DEBUG: Baudrate: {baudrate}")
            # Clear any startup messages from Arduino
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            print("DEBUG: Serial buffer cleared")
        except Exception as e:
            print(f"ERROR: Failed to initialize Serial: {e}")
            print("\nMake sure:")
            print("  1. Arduino is connected via USB")
            print("  2. Port is correct (check with 'ls /dev/ttyUSB*' or 'ls /dev/ttyACM*')")
            print("  3. You have permissions to access the port")
            sys.exit(1)
    
    def send_6d_signal(self, motor1_dir, motor1_speed, motor2_dir, motor2_speed, motor3_dir, motor3_speed):
        """
        Send 6D signal to Arduino
        
        Args:
            motor1_dir: Direction for motor 1 (0=Forward, 1=Backward, 2=Stop)
            motor1_speed: Speed for motor 1 (0-255)
            motor2_dir: Direction for motor 2
            motor2_speed: Speed for motor 2 (0-255)
            motor3_dir: Direction for motor 3
            motor3_speed: Speed for motor 3 (0-255)
        """
        signal = [
            motor1_dir & 0xFF,
            motor1_speed & 0xFF,
            motor2_dir & 0xFF,
            motor2_speed & 0xFF,
            motor3_dir & 0xFF,
            motor3_speed & 0xFF
        ]
        
        print("\n--- Sending 6D Signal ---")
        print(f"Motor 1: Dir={self._dir_name(motor1_dir)}, Speed={motor1_speed}")
        print(f"Motor 2: Dir={self._dir_name(motor2_dir)}, Speed={motor2_speed}")
        print(f"Motor 3: Dir={self._dir_name(motor3_dir)}, Speed={motor3_speed}")
        print(f"RAW DATA: {' '.join(f'0x{b:02X}' for b in signal)}")
        
        try:
            # Send 6 bytes over serial
            self.serial.write(bytes(signal))
            self.serial.flush()
            print(f"DEBUG: Data sent successfully")
            
            # Read Arduino's debug output
            time.sleep(0.2)  # Give Arduino time to process and respond
            while self.serial.in_waiting > 0:
                response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"  Arduino: {response}")
            
            return True
        except Exception as e:
            print(f"ERROR: Failed to send data: {e}")
            return False
    
    def _dir_name(self, direction):
        """Convert direction code to readable name"""
        if direction == DIR_FORWARD:
            return "FORWARD"
        elif direction == DIR_BACKWARD:
            return "BACKWARD"
        elif direction == DIR_STOP:
            return "STOP"
        else:
            return f"UNKNOWN({direction})"
    
    def stop_all(self):
        """Send stop command to all motors"""
        print("\nDEBUG: Stopping all motors...")
        return self.send_6d_signal(DIR_STOP, 0, DIR_STOP, 0, DIR_STOP, 0)
    
    def move_forward(self, speed=200):
        """Move all motors forward"""
        print(f"\nDEBUG: Moving forward at speed {speed}...")
        return self.send_6d_signal(DIR_FORWARD, speed, DIR_FORWARD, speed, DIR_FORWARD, speed)
    
    def move_backward(self, speed=200):
        """Move all motors backward"""
        print(f"\nDEBUG: Moving backward at speed {speed}...")
        return self.send_6d_signal(DIR_BACKWARD, speed, DIR_BACKWARD, speed, DIR_BACKWARD, speed)
    
    def turn_left(self, speed=150):
        """Turn left (left motors backward, right motors forward)"""
        print(f"\nDEBUG: Turning left at speed {speed}...")
        return self.send_6d_signal(DIR_BACKWARD, speed, DIR_FORWARD, speed, DIR_BACKWARD, speed)
    
    def turn_right(self, speed=150):
        """Turn right (left motors forward, right motors backward)"""
        print(f"\nDEBUG: Turning right at speed {speed}...")
        return self.send_6d_signal(DIR_FORWARD, speed, DIR_BACKWARD, speed, DIR_FORWARD, speed)
    
    def close(self):
        """Close Serial connection"""
        print("\nDEBUG: Closing Serial connection...")
        if self.serial.is_open:
            self.serial.close()
        print("DEBUG: Serial closed")


class KeyboardController:
    """Keyboard control handler for arrow keys"""
    
    def __init__(self, motor_controller):
        self.controller = motor_controller
        self.current_speed = 180
        self.turn_speed = 150
        self.is_moving = False
        self.current_direction = None
        
    def on_press(self, key):
        """Handle key press events"""
        try:
            if key == keyboard.Key.up:
                if not self.is_moving or self.current_direction != 'forward':
                    print("\n[KEYBOARD] UP pressed - Moving FORWARD")
                    self.controller.move_forward(speed=self.current_speed)
                    self.is_moving = True
                    self.current_direction = 'forward'
                    
            elif key == keyboard.Key.down:
                if not self.is_moving or self.current_direction != 'backward':
                    print("\n[KEYBOARD] DOWN pressed - Moving BACKWARD")
                    self.controller.move_backward(speed=self.current_speed)
                    self.is_moving = True
                    self.current_direction = 'backward'
                    
            elif key == keyboard.Key.left:
                if not self.is_moving or self.current_direction != 'left':
                    print("\n[KEYBOARD] LEFT pressed - Turning LEFT")
                    self.controller.turn_left(speed=self.turn_speed)
                    self.is_moving = True
                    self.current_direction = 'left'
                    
            elif key == keyboard.Key.right:
                if not self.is_moving or self.current_direction != 'right':
                    print("\n[KEYBOARD] RIGHT pressed - Turning RIGHT")
                    self.controller.turn_right(speed=self.turn_speed)
                    self.is_moving = True
                    self.current_direction = 'right'
                    
            elif key == keyboard.Key.space:
                print("\n[KEYBOARD] SPACE pressed - STOP")
                self.controller.stop_all()
                self.is_moving = False
                self.current_direction = None
                
            elif key == keyboard.Key.esc:
                print("\n[KEYBOARD] ESC pressed - Exiting...")
                self.controller.stop_all()
                return False  # Stop listener
                
        except Exception as e:
            print(f"Error handling key press: {e}")
    
    def on_release(self, key):
        """Handle key release events"""
        try:
            if key in [keyboard.Key.up, keyboard.Key.down, 
                      keyboard.Key.left, keyboard.Key.right]:
                print(f"\n[KEYBOARD] Key released - STOP")
                self.controller.stop_all()
                self.is_moving = False
                self.current_direction = None
                
        except Exception as e:
            print(f"Error handling key release: {e}")


def main(stdscr):
    # Clear screen
    stdscr.clear()
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.keypad(True)   # Enable arrow keys

    print("Ready! Use arrow keys to control the robot...")
    
    # Initialize your motor controller
    controller = MotorController(port="/dev/ttyUSB0", baudrate=115200)
    
    try:
        while True:
            key = stdscr.getch()
            
            if key == curses.KEY_UP:
                controller.move_forward()
            elif key == curses.KEY_DOWN:
                controller.move_backward()
            elif key == curses.KEY_LEFT:
                controller.turn_left()
            elif key == curses.KEY_RIGHT:
                controller.turn_right()
            elif key == ord(' '):
                controller.stop_all()
            elif key == 27:  # ESC
                break

            sleep(0.05)  # small delay to reduce CPU usage
    finally:
        controller.stop_all()
        controller.close()

if __name__ == "__main__":
    curses.wrapper(main)
