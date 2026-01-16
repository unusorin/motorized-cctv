# Motorized CCTV - ESP32 Motor Controller

A dual DC motor controller system using ESP32-C3 with optical endstops and PWM speed control. Designed for precise motorized focus and zoom control with binary protocol communication.

## Hardware Components

### Microcontroller
- **ESP32-C3 SuperMini**
  - Compact development board with USB-C
  - Built-in USB CDC for serial communication
  - Sufficient GPIO pins for dual motor control with endstops

### Motor Driver
- **MX1508 Dual H-Bridge Motor Driver**
  - Controls two DC motors independently
  - Low-voltage operation (2-9.6V)
  - PWM speed control support
  - Compact size suitable for small projects

### Motors
- **2x DC Motors** (Focus and Zoom)
  - Controlled via PWM for variable speed (35%-100% duty cycle)
  - Minimum 35% duty cycle required to overcome static friction

### Endstops
- **4x Optical Endstops**
  - 2 per motor (MIN and MAX positions)
  - Normally interrupted (HIGH), triggered when optical contact (LOW)
  - Used for position limiting and safety

## Pin Configuration

### Motor Control Pins (MX1508)

| Motor | Function | GPIO Pin | MX1508 Pin |
|-------|----------|----------|------------|
| Focus | IN1 | GPIO 5 | IN1 |
| Focus | IN2 | GPIO 4 | IN2 |
| Zoom | IN3 | GPIO 7 | IN3 |
| Zoom | IN4 | GPIO 6 | IN4 |

### Endstop Pins

| Motor | Position | GPIO Pin | Pull-up |
|-------|----------|----------|---------|
| Focus | MIN | GPIO 2 | Internal |
| Focus | MAX | GPIO 3 | Internal |
| Zoom | MIN | GPIO 1 | Internal |
| Zoom | MAX | GPIO 0 | Internal |

## Wiring Diagram

```
ESP32-C3 SuperMini          MX1508 H-Bridge
┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │
│ GPIO 7 ─────────┼────────▶│ IN3   Motor A+ │───┐
│ GPIO 6 ─────────┼────────▶│ IN4   Motor A- │───┤ Zoom Motor
│                 │         │                 │   │
│ GPIO 5 ─────────┼────────▶│ IN1   Motor B+ │───┐
│ GPIO 4 ─────────┼────────▶│ IN2   Motor B- │───┤ Focus Motor
│                 │         │                 │   │
│ GND ────────────┼─────────│ GND             │
│ 5V ─────────────┼─────────│ VCC             │
└─────────────────┘         └─────────────────┘

Optical Endstops (x4)
┌──────────────┐
│ VCC ────── 5V (from ESP32 or external)
│ GND ────── GND
│ OUT ────── GPIO 0/1/2/3 (respective endstop)
└──────────────┘
```

## Communication Protocol

The system supports two communication protocols: **Binary** (recommended) and **Text** (legacy/debugging).

### Binary Protocol (Recommended)

The binary protocol uses a 2-byte struct for efficient, atomic control of both motors simultaneously.

#### Struct Definition

```c
struct MotorCommand {
  int8_t focus;  // -127 to +127
  int8_t zoom;   // -127 to +127
} __attribute__((packed));
```

#### Value Interpretation

- **Sign**: Direction
  - Positive (+1 to +127): Forward
  - Negative (-1 to -127): Backward
  - Zero (0): Stop

- **Magnitude**: Speed (mapped to 35%-100% PWM duty cycle)
  - Value 1: Minimum speed (35% duty cycle, ~89/255 PWM)
  - Value 127: Maximum speed (100% duty cycle, 255/255 PWM)
  - The firmware automatically maps input range to ensure motors always have enough torque

#### Python Example

```python
import serial
import struct

# Connect to ESP32
ser = serial.Serial('/dev/ttyACM0', 115200)

# Example 1: Focus forward at 50%, Zoom backward at 100%
command = struct.pack('bb', 64, -127)
ser.write(command)

# Example 2: Both motors forward at different speeds
command = struct.pack('bb', 25, 100)  # Focus 20%, Zoom 80%
ser.write(command)

# Example 3: Stop all motors
command = struct.pack('bb', 0, 0)
ser.write(command)
```

#### Advantages of Binary Protocol

- **Compact**: Only 2 bytes per command
- **Fast**: No parsing overhead
- **Atomic**: Both motors controlled simultaneously
- **Efficient**: Ideal for real-time control applications

### Text Protocol (Legacy)

Text commands are human-readable and useful for debugging via serial monitor.

#### Available Commands

```
focus_stop              - Stop focus motor
focus_forward           - Move focus forward (at current speed)
focus_back              - Move focus backward (at current speed)
zoom_stop               - Stop zoom motor
zoom_forward            - Move zoom forward (at current speed)
zoom_back               - Move zoom backward (at current speed)
focus_speed <0-255>     - Set focus motor speed (PWM value)
zoom_speed <0-255>      - Set zoom motor speed (PWM value)
```

#### Examples

```
focus_forward
zoom_speed 128
zoom_back
focus_stop
```

### Protocol Auto-Detection

The ESP32 firmware automatically detects which protocol is being used:
- **Binary**: First byte ≤127 or ≥129 (valid signed int8 range)
- **Text**: First byte is ASCII printable character (0x20-0x7E)

## Motor Control Details

### Speed Mapping

The firmware maps input speed values to ensure reliable motor operation:

```
Input Value (±1 to ±127) → PWM Duty Cycle (35% to 100%)

Examples:
  ±1   → 35% duty cycle (89/255 PWM)
  ±25  → ~40% duty cycle
  ±64  → ~50% duty cycle
  ±127 → 100% duty cycle (255/255 PWM)
```

**Why 35% minimum?**
DC motors require a minimum voltage/duty cycle to overcome static friction and back-EMF. Below ~35%, motors will buzz but not rotate. The firmware ensures all non-zero commands produce enough torque for smooth rotation.

### Stop Modes

The system uses **active brake mode** when stopping:
- Both H-bridge inputs set to HIGH
- This shorts the motor windings through the driver
- Provides immediate stopping and prevents coasting
- Eliminates buzzing/humming when stopped

### Endstop Protection

Motors automatically stop when endstops are triggered:
- **MIN endstop triggered**: Prevents backward movement, allows forward
- **MAX endstop triggered**: Prevents forward movement, allows backward
- Commands that would move into an active endstop are rejected with error messages
- 200ms debounce prevents false triggers

## Python Control Application

A Pygame-based GUI application (`motor_control.py`) provides keyboard control:

### Controls

- **UP Arrow**: Zoom forward
- **DOWN Arrow**: Zoom backward
- **LEFT Arrow**: Focus backward
- **RIGHT Arrow**: Focus forward
- **SHIFT**: Full speed (100%)
- **Default**: 20% speed (maps to ~40% actual motor speed)
- **ESC**: Exit

### Features

- Real-time visual feedback
- Speed indicator (20% / Full Speed)
- Connection status display
- Motor state visualization
- Clean shutdown (stops motors before disconnect)

### Building and Flashing

```bash
# Build firmware
pio run

# Upload to ESP32-C3
pio run --target upload

# Monitor serial output
pio device monitor
```

### Running the Application

```bash
# Install dependencies
pip install pygame pyserial

# Run the controller
python3 motor_control.py
```


## Serial Configuration

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## License

This project is open source. Feel free to modify and adapt for your needs.

