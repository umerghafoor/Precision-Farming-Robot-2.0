# Example Sketches

This directory contains example Arduino sketches to help you test and learn about the Precision Farming Robot components.

## Available Examples

### 1. sensor_test.ino

**Purpose**: Test all sensors independently

**What it does**:
- Reads DHT22 (temperature/humidity)
- Reads soil moisture sensor
- Reads ultrasonic distance sensor
- Displays readings on Serial Monitor every 2 seconds

**How to use**:
1. Wire up all sensors (see hardware_setup.md)
2. Upload this sketch
3. Open Serial Monitor (115200 baud)
4. Observe sensor readings

**Good for**:
- Initial sensor testing
- Troubleshooting sensor issues
- Calibration reference

---

### 2. motor_test.ino

**Purpose**: Test motor movement patterns

**What it does**:
- Tests forward movement
- Tests backward movement
- Tests left turn
- Tests right turn
- Repeats sequence continuously

**How to use**:
1. Wire up motors and L298N driver
2. **IMPORTANT**: Lift robot so wheels don't touch ground
3. Upload this sketch
4. Watch motors run through test sequence

**Good for**:
- Verifying motor connections
- Testing motor driver
- Checking motor direction (swap wires if needed)

**Safety Warning**: Always test motors with robot elevated first!

---

## Running Examples

### Basic Steps

1. **Open Example**
   ```
   File → Open → Navigate to examples folder → Select sketch
   ```

2. **Select Board**
   ```
   Tools → Board → Arduino Mega 2560 (or your board)
   ```

3. **Select Port**
   ```
   Tools → Port → (your COM/USB port)
   ```

4. **Upload**
   ```
   Click Upload button (→)
   ```

5. **Monitor Output**
   ```
   Tools → Serial Monitor (Ctrl+Shift+M)
   Set baud rate to 115200
   ```

## Customizing Examples

Feel free to modify these examples:

### Change Update Rate
```cpp
// Change from 2 seconds to 5 seconds
delay(5000);
```

### Modify Pin Numbers
```cpp
// If using different pins
#define DHT_PIN 8  // Changed from 7
```

### Adjust Motor Speed
```cpp
// Change speed (0-255)
#define MOTOR_SPEED 150  // Slower than default 200
```

## Creating Your Own Examples

Want to create a new example? Great!

1. Create new .ino file in `examples/` folder
2. Include descriptive header comment
3. Document pin connections
4. Add clear serial output
5. Submit via pull request

### Example Template

```cpp
/*
 * Example Name
 * 
 * Description of what this example does
 * 
 * Hardware Required:
 * - Component 1
 * - Component 2
 * 
 * Connections:
 * Component → Arduino
 * Pin1    → Pin7
 */

void setup() {
  Serial.begin(115200);
  Serial.println("Example Name");
  // Setup code
}

void loop() {
  // Main code
  delay(1000);
}
```

## Troubleshooting Examples

### Compilation Errors

**"Library not found"**
- Install required libraries (see README.md)

**"Pin X already defined"**
- Check for pin conflicts in your code

### Runtime Issues

**No output in Serial Monitor**
- Check baud rate (must be 115200)
- Verify board is connected
- Press reset button

**Sensor readings are 0 or NaN**
- Check wiring connections
- Verify power supply
- Test sensor with multimeter

**Motors don't move**
- Check battery charge
- Verify driver connections
- Test with direct battery connection

## Learning Path

Recommended order for beginners:

1. **Start with sensor_test.ino**
   - Learn how sensors work
   - Understand serial output
   - Practice troubleshooting

2. **Try motor_test.ino**
   - Understand motor control
   - Learn about PWM
   - Test movement patterns

3. **Explore main program**
   - See how everything integrates
   - Study the control logic
   - Understand state management

4. **Create your own**
   - Combine different features
   - Add new sensors
   - Experiment with algorithms

## Additional Resources

- Main documentation: `../docs/`
- API reference: `../docs/api_reference.md`
- Hardware guide: `../docs/hardware_setup.md`
- User manual: `../docs/user_guide.md`

## Need Help?

- Check existing documentation
- Review code comments
- Test components individually
- Ask in GitHub discussions

Happy coding! 🚀
