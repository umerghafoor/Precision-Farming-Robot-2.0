#!/usr/bin/env python3
"""
6D Signal USB Serial Master Controller with Keyboard Control
Sends 6-byte signal to Arduino for motor control via USB Serial
Control keys: Arrow keys for drive, Q/A for Servo 1, W/S for Servo 2
Format: [Dir1, Speed1, Dir2, Speed2, Dir3, Speed3]
"""

import serial
import time
import sys
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

    def send_text_command(self, command):
        """Send newline-delimited text command to Arduino command parser."""
        try:
            payload = f"{command.strip()}\n".encode("utf-8")
            self.serial.write(payload)
            self.serial.flush()
            print(f"DEBUG: Text command sent -> {command.strip().upper()}")
            return True
        except Exception as e:
            print(f"ERROR: Failed to send text command '{command}': {e}")
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
    """Keyboard control handler for motor + dual-servo keys"""
    
    def __init__(self, motor_controller):
        self.controller = motor_controller
        self.current_speed = 180
        self.min_speed = 0
        self.max_speed = 255
        self.speed_step = 10

    def _adjust_speed(self, delta):
        self.current_speed = max(self.min_speed, min(self.max_speed, self.current_speed + delta))
        print(f"\n[KEYBOARD] SPEED -> {self.current_speed}")

    def handle_key(self, key):
        """Handle a curses key code. Returns False when exit is requested."""
        if key == curses.KEY_UP:
            print("\n[KEYBOARD] UP -> FORWARD")
            self.controller.move_forward(speed=self.current_speed)
        elif key == curses.KEY_DOWN:
            print("\n[KEYBOARD] DOWN -> BACKWARD")
            self.controller.move_backward(speed=self.current_speed)
        elif key == curses.KEY_LEFT:
            print("\n[KEYBOARD] LEFT -> TURN LEFT")
            self.controller.turn_left(speed=150)
        elif key == curses.KEY_RIGHT:
            print("\n[KEYBOARD] RIGHT -> TURN RIGHT")
            self.controller.turn_right(speed=150)
        elif key in (ord('q'), ord('Q')):
            print("\n[KEYBOARD] Q -> SERVO 1 LEFT")
            self.controller.send_text_command("Q")
        elif key in (ord('a'), ord('A')):
            print("\n[KEYBOARD] A -> SERVO 1 RIGHT")
            self.controller.send_text_command("A")
        elif key in (ord('w'), ord('W')):
            print("\n[KEYBOARD] W -> SERVO 2 LEFT")
            self.controller.send_text_command("W")
        elif key in (ord('s'), ord('S')):
            print("\n[KEYBOARD] S -> SERVO 2 RIGHT")
            self.controller.send_text_command("S")
        elif key in (ord('e'), ord('E')):
            self._adjust_speed(self.speed_step)
        elif key in (ord('d'), ord('D')):
            self._adjust_speed(-self.speed_step)
        elif key in (ord(' '), ord('x'), ord('X')):
            print("\n[KEYBOARD] STOP")
            self.controller.stop_all()
        elif key in (ord('r'), ord('R')):
            print("\n[KEYBOARD] R -> CENTER SERVOS")
            self.controller.send_text_command("CENTER")
        elif key == 27:  # ESC
            print("\n[KEYBOARD] ESC -> EXIT")
            self.controller.stop_all()
            return False

        return True


def main(stdscr):
    # Clear screen
    stdscr.clear()
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.keypad(True)

    print("Ready! Controls: Arrow keys=drive, E/D=speed +/- , Q/A=servo1 left/right, W/S=servo2 left/right, R=center servos, SPACE/X=stop, ESC=exit")
    
    # Initialize your motor controller
    controller = MotorController(port="/dev/ttyUSB0", baudrate=115200)
    keyboard_controller = KeyboardController(controller)
    
    try:
        while True:
            key = stdscr.getch()

            if key != -1:
                should_continue = keyboard_controller.handle_key(key)
                if not should_continue:
                    break

            sleep(0.05)  # small delay to reduce CPU usage
    finally:
        controller.stop_all()
        controller.close()

if __name__ == "__main__":
    curses.wrapper(main)
