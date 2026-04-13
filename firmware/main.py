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
        self.port = port
        self.baudrate = baudrate
        self.timeout = 1
        self.serial = None

        if not self._connect_serial(is_initial=True):
            print("\nMake sure:")
            print("  1. Arduino is connected via USB")
            print("  2. Port is correct (check with 'ls /dev/ttyUSB*' or 'ls /dev/ttyACM*')")
            print("  3. You have permissions to access the port")
            sys.exit(1)

    def _connect_serial(self, is_initial=False):
        """Open serial port and clear startup noise from MCU reset."""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout, write_timeout=1)
            time.sleep(2)  # Wait for Arduino to reset after serial connection
            print(f"DEBUG: Serial initialized on port {self.port}")
            print(f"DEBUG: Baudrate: {self.baudrate}")
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            print("DEBUG: Serial buffer cleared")
            return True
        except Exception as e:
            phase = "initialize" if is_initial else "reconnect"
            print(f"ERROR: Failed to {phase} serial: {e}")
            self.serial = None
            return False

    def _ensure_serial_connection(self):
        """Verify serial is usable; attempt reconnect if it is not."""
        if self.serial is not None and self.serial.is_open:
            return True

        print("WARNING: Serial link is closed. Attempting reconnect...")
        return self._connect_serial(is_initial=False)

    def _handle_write_failure(self, err):
        """Handle a write failure and attempt one reconnect for retry."""
        print(f"ERROR: Serial write failed: {err}")
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except Exception:
                pass
            self.serial = None

        print("WARNING: Attempting serial reconnect after write failure...")
        return self._connect_serial(is_initial=False)

    def _read_arduino_output(self, wait_s=0.2):
        """Read and print any pending Arduino serial lines for a short window."""
        if self.serial is None or not self.serial.is_open:
            return

        time.sleep(wait_s)
        while self.serial.in_waiting > 0:
            response = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if response:
                print(f"  Arduino: {response}")
    
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
        
        if not self._ensure_serial_connection():
            print("ERROR: Cannot send 6D signal because serial is unavailable")
            return False

        for attempt in range(2):
            try:
                # Send 6 bytes over serial
                self.serial.write(bytes(signal))
                self.serial.flush()
                print("DEBUG: Data sent successfully")

                # Read Arduino's debug output
                self._read_arduino_output(wait_s=0.2)

                return True
            except Exception as e:
                if attempt == 0 and self._handle_write_failure(e):
                    print("INFO: Serial recovered, retrying 6D signal...")
                    continue
                print(f"ERROR: Failed to send data: {e}")
                return False

        return False

    def send_text_command(self, command):
        """Send newline-delimited text command to Arduino command parser."""
        if not self._ensure_serial_connection():
            print(f"ERROR: Cannot send text command '{command}' because serial is unavailable")
            return False

        payload = f"{command.strip()}\n".encode("utf-8")
        for attempt in range(2):
            try:
                self.serial.write(payload)
                self.serial.flush()
                print(f"DEBUG: Text command sent -> {command.strip().upper()}")
                self._read_arduino_output(wait_s=0.12)
                return True
            except Exception as e:
                if attempt == 0 and self._handle_write_failure(e):
                    print(f"INFO: Serial recovered, retrying text command '{command.strip().upper()}'...")
                    continue
                print(f"ERROR: Failed to send text command '{command}': {e}")
                return False

        return False

    def set_servo_angle(self, servo_id, angle):
        """Send absolute angle command to a servo (expects Arduino S1:/S2: parser support)."""
        if servo_id not in (1, 2):
            print(f"ERROR: Invalid servo id {servo_id}. Expected 1 or 2.")
            return False
        return self.send_text_command(f"S{servo_id}:{angle}")
    
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
        if self.serial is not None and self.serial.is_open:
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

    def _prompt_for_angle(self, stdscr, servo_label):
        """Prompt user for an absolute servo angle in curses mode."""
        if stdscr is None:
            print("\n[KEYBOARD] ERROR: Prompt requires curses screen context")
            return None

        height, width = stdscr.getmaxyx()
        prompt = f"Enter {servo_label} angle (0-180): "
        prompt_row = max(0, height - 1)
        max_input_len = 4

        try:
            stdscr.nodelay(False)
            curses.echo()

            stdscr.move(prompt_row, 0)
            stdscr.clrtoeol()
            stdscr.addstr(prompt_row, 0, prompt[: max(0, width - 1)])
            stdscr.refresh()

            input_col = min(len(prompt), max(0, width - 1))
            raw = stdscr.getstr(prompt_row, input_col, max_input_len).decode("utf-8", errors="ignore").strip()
        finally:
            curses.noecho()
            stdscr.nodelay(True)
            stdscr.move(prompt_row, 0)
            stdscr.clrtoeol()
            stdscr.refresh()

        if not raw:
            print(f"\n[KEYBOARD] {servo_label}: input cancelled")
            return None

        try:
            angle = int(raw)
        except ValueError:
            print(f"\n[KEYBOARD] Invalid angle '{raw}'. Enter an integer between 0 and 180.")
            return None

        if not 0 <= angle <= 180:
            print(f"\n[KEYBOARD] Angle {angle} out of range. Valid range is 0-180.")
            return None

        return angle

    def handle_key(self, key, stdscr=None):
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
        elif key in (ord('m'), ord('M')):
            angle = self._prompt_for_angle(stdscr, "Servo 1")
            if angle is not None:
                print(f"\n[KEYBOARD] M -> SERVO 1 ANGLE {angle}")
                self.controller.set_servo_angle(1, angle)
        elif key in (ord('n'), ord('N')):
            angle = self._prompt_for_angle(stdscr, "Servo 2")
            if angle is not None:
                print(f"\n[KEYBOARD] N -> SERVO 2 ANGLE {angle}")
                self.controller.set_servo_angle(2, angle)
        elif key in (ord('u'), ord('U')):
            print("\n[KEYBOARD] U -> SERVO 1 STOP")
            self.controller.send_text_command("S1STOP")
        elif key in (ord('i'), ord('I')):
            print("\n[KEYBOARD] I -> SERVO 2 STOP")
            self.controller.send_text_command("S2STOP")
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

    print("Ready! Controls: Arrow keys=drive, E/D=speed +/- , Q/A=servo1 left/right, W/S=servo2 left/right, M=servo1 absolute value, N=servo2 absolute value, U=servo1 stop, I=servo2 stop, R=center servos, SPACE/X=stop, ESC=exit")
    
    # Initialize your motor controller
    controller = MotorController(port="/dev/ttyUSB0", baudrate=115200)
    keyboard_controller = KeyboardController(controller)
    
    try:
        while True:
            key = stdscr.getch()

            if key != -1:
                should_continue = keyboard_controller.handle_key(key, stdscr=stdscr)
                if not should_continue:
                    break

            sleep(0.05)  # small delay to reduce CPU usage
    finally:
        controller.stop_all()
        controller.close()

if __name__ == "__main__":
    curses.wrapper(main)
