#!/usr/bin/env python3
"""
6D Signal USB Serial Master Controller
Sends 6-byte signal to Arduino for motor control via USB Serial
Format: [Dir1, Speed1, Dir2, Speed2, Dir3, Speed3]
"""

import serial
import time
import sys

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


def main():
    """Main demonstration program"""
    print("\nInitializing motor controller...")
    
    # Auto-detect serial port
    import glob
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    
    if not ports:
        print("ERROR: No Arduino found. Please connect Arduino via USB.")
        return
    
    port = ports[0]
    print(f"DEBUG: Using port {port}")
    
    try:
        controller = MotorController(port=port, baudrate=115200)
    except Exception as e:
        print(f"FATAL: Could not initialize controller: {e}")
        return
    
    try:
        print("\n" + "=" * 50)
        print("Starting test sequence...")
        print("=" * 50)
        
        # Test 1: Stop all motors
        print("\n[TEST 1] Stopping all motors")
        controller.stop_all()
        time.sleep(2)
        
        # Test 2: Move forward
        print("\n[TEST 2] Moving forward")
        controller.move_forward(speed=180)
        time.sleep(3)
        
        # Test 3: Stop
        print("\n[TEST 3] Stopping")
        controller.stop_all()
        time.sleep(1)
        
        # Test 4: Move backward
        print("\n[TEST 4] Moving backward")
        controller.move_backward(speed=180)
        time.sleep(3)
        
        # Test 5: Stop
        print("\n[TEST 5] Stopping")
        controller.stop_all()
        time.sleep(1)
        
        # Test 6: Turn left
        print("\n[TEST 6] Turning left")
        controller.turn_left(speed=150)
        time.sleep(2)
        
        # Test 7: Stop
        print("\n[TEST 7] Stopping")
        controller.stop_all()
        time.sleep(1)
        
        # Test 8: Turn right
        print("\n[TEST 8] Turning right")
        controller.turn_right(speed=150)
        time.sleep(2)
        
        # Test 9: Final stop
        print("\n[TEST 9] Final stop")
        controller.stop_all()
        
        print("\n" + "=" * 50)
        print("Test sequence complete!")
        print("=" * 50)
        
    except KeyboardInterrupt:
        print("\n\nDEBUG: Interrupted by user!")
        controller.stop_all()
    except Exception as e:
        print(f"\nERROR: {e}")
        controller.stop_all()
    finally:
        controller.close()
        print("\nProgram terminated")


if __name__ == "__main__":
    main()
