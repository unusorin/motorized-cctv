#!/usr/bin/env python3
"""
Motorized CCTV - ESP32 Motor Controller - Pygame Keyboard Interface
Controls focus and zoom motors via serial commands using arrow keys.

Controls:
- UP Arrow: Zoom Forward
- DOWN Arrow: Zoom Backward
- LEFT Arrow: Focus Backward
- RIGHT Arrow: Focus Forward
- ESC: Exit

Communication Protocol:
- Binary struct mode: Sends 2-byte MotorCommand struct for efficient control
- Text mode (fallback): Sends text commands for debugging
"""

import pygame
import serial
import serial.tools.list_ports
import sys
import time
import struct

# Serial configuration
BAUD_RATE = 115200
SERIAL_PORT = None  # Will be auto-detected or set manually

# Communication mode
USE_BINARY_PROTOCOL = True  # Set to False for text commands (debugging)

# Colors for UI
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 100, 255)
GRAY = (100, 100, 100)

class MotorController:
    def __init__(self):
        self.serial_connection = None
        self.focus_state = "STOPPED"
        self.zoom_state = "STOPPED"
        self.connected = False
        self.focus_direction = 0  # -1, 0, 1
        self.zoom_direction = 0   # -1, 0, 1

    def find_esp32_port(self):
        """Auto-detect ESP32-C3 serial port"""
        ports = serial.tools.list_ports.comports()

        print("Available serial ports:")
        for i, port in enumerate(ports):
            print(f"  {i}: {port.device} - {port.description}")
            # ESP32-C3 often shows up with these identifiers
            if "USB" in port.description.upper() or "CP210" in port.description or "CH340" in port.description or "SERIAL" in port.description:
                print(f"  -> Potential ESP32 device detected")

        if not ports:
            print("No serial ports found!")
            return None

        # Try to auto-select
        for port in ports:
            if "USB" in port.description.upper() or "SERIAL" in port.description.upper():
                print(f"\nAuto-selecting: {port.device}")
                return port.device

        # If no auto-detection, ask user
        if len(ports) == 1:
            print(f"\nUsing only available port: {ports[0].device}")
            return ports[0].device
        else:
            try:
                choice = int(input("\nEnter port number to use: "))
                return ports[choice].device
            except (ValueError, IndexError):
                print("Invalid selection")
                return None

    def connect(self, port=None):
        """Connect to ESP32 via serial"""
        try:
            if port is None:
                port = self.find_esp32_port()

            if port is None:
                return False

            self.serial_connection = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for connection to stabilize
            self.connected = True
            print(f"Connected to {port} at {BAUD_RATE} baud")

            # Read any initial messages from ESP32
            time.sleep(0.5)
            while self.serial_connection.in_waiting:
                line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"ESP32: {line}")

            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial_connection and self.serial_connection.is_open:
            # Send stop command using binary protocol
            print("Stopping all motors before disconnect...")
            if USE_BINARY_PROTOCOL:
                self.send_motor_command_binary(0, 0)
            else:
                self.send_command("focus_stop")
                self.send_command("zoom_stop")
            # Give time for command to be processed
            time.sleep(0.2)
            self.serial_connection.close()
            self.connected = False
            print("Disconnected from ESP32")

    def send_motor_command_binary(self, focus, zoom):
        """Send binary motor command struct to ESP32

        Args:
            focus: -127 to +127 (sign=direction, magnitude=speed, 0=stop)
            zoom: -127 to +127 (sign=direction, magnitude=speed, 0=stop)
        """
        if not self.connected or not self.serial_connection:
            print(f"Not connected! Cannot send binary command")
            return False

        try:
            # Pack as binary struct: 2 bytes (int8, int8)
            # Format: 'bb' = signed char, signed char
            packed_data = struct.pack('bb', focus, zoom)
            self.serial_connection.write(packed_data)

            # Update internal state
            if focus > 0:
                self.focus_state = "FORWARD"
                self.focus_direction = 1
            elif focus < 0:
                self.focus_state = "BACKWARD"
                self.focus_direction = -1
            else:
                self.focus_state = "STOPPED"
                self.focus_direction = 0

            if zoom > 0:
                self.zoom_state = "FORWARD"
                self.zoom_direction = 1
            elif zoom < 0:
                self.zoom_state = "BACKWARD"
                self.zoom_direction = -1
            else:
                self.zoom_state = "STOPPED"
                self.zoom_direction = 0

            print(f"Sent binary: Focus={focus:4d}, Zoom={zoom:4d}")
            return True
        except Exception as e:
            print(f"Error sending binary command: {e}")
            return False

    def send_command(self, command):
        """Send text command to ESP32 (backward compatibility)"""
        if not self.connected or not self.serial_connection:
            print(f"Not connected! Cannot send: {command}")
            return False

        try:
            self.serial_connection.write(f"{command}\n".encode())
            print(f"Sent: {command}")

            # Update state tracking
            if "focus" in command:
                if "stop" in command:
                    self.focus_state = "STOPPED"
                    self.focus_direction = 0
                elif "forward" in command:
                    self.focus_state = "FORWARD"
                    self.focus_direction = 1
                elif "back" in command:
                    self.focus_state = "BACKWARD"
                    self.focus_direction = -1
            elif "zoom" in command:
                if "stop" in command:
                    self.zoom_state = "STOPPED"
                    self.zoom_direction = 0
                elif "forward" in command:
                    self.zoom_state = "FORWARD"
                    self.zoom_direction = 1
                elif "back" in command:
                    self.zoom_state = "BACKWARD"
                    self.zoom_direction = -1

            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

    def update_motors(self, speed_multiplier=0.5):
        """Send current motor state using binary protocol

        Args:
            speed_multiplier: Speed multiplier (0.5 = half speed, 1.0 = full speed)
        """
        if USE_BINARY_PROTOCOL:
            # Map direction to signed value with speed multiplier
            # Base speed is 127 (full), multiply by speed_multiplier
            base_speed = int(127 * speed_multiplier)
            focus = self.focus_direction * base_speed
            zoom = self.zoom_direction * base_speed
            self.send_motor_command_binary(focus, zoom)
        else:
            # Use text commands
            if self.focus_direction == 1:
                self.send_command("focus_forward")
            elif self.focus_direction == -1:
                self.send_command("focus_back")
            elif self.focus_direction == 0:
                self.send_command("focus_stop")

            if self.zoom_direction == 1:
                self.send_command("zoom_forward")
            elif self.zoom_direction == -1:
                self.send_command("zoom_back")
            elif self.zoom_direction == 0:
                self.send_command("zoom_stop")

    def read_serial(self):
        """Read and print any incoming serial data"""
        if self.connected and self.serial_connection and self.serial_connection.in_waiting:
            try:
                line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"ESP32: {line}")
            except Exception as e:
                print(f"Error reading serial: {e}")


def main():
    # Initialize Pygame
    pygame.init()

    # Set up display
    screen_width = 600
    screen_height = 400
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Motorized CCTV Controller")

    # Font
    font_large = pygame.font.Font(None, 48)
    font_medium = pygame.font.Font(None, 32)
    font_small = pygame.font.Font(None, 24)

    # Initialize motor controller
    controller = MotorController()

    print("\n" + "="*50)
    print("Motorized CCTV Controller - Pygame Interface")
    print("="*50)

    if not controller.connect():
        print("Failed to connect to ESP32. Exiting...")
        pygame.quit()
        sys.exit(1)

    # Track key states
    keys_pressed = {
        pygame.K_UP: False,
        pygame.K_DOWN: False,
        pygame.K_LEFT: False,
        pygame.K_RIGHT: False
    }

    # Track shift key for speed control
    shift_pressed = False

    clock = pygame.time.Clock()
    running = True

    print("\nControls:")
    print("  UP Arrow    = Zoom Forward")
    print("  DOWN Arrow  = Zoom Backward")
    print("  LEFT Arrow  = Focus Backward")
    print("  RIGHT Arrow = Focus Forward")
    print("  SHIFT       = Full Speed (default is 20% speed)")
    print("  ESC         = Exit")
    print(f"\nProtocol: {'Binary struct' if USE_BINARY_PROTOCOL else 'Text commands'}")
    print("Press and hold arrow keys to control motors...")

    while running:
        # Check shift key state
        keys = pygame.key.get_pressed()
        shift_pressed = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        speed_multiplier = 1.0 if shift_pressed else 0.2

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                # Exit on ESC
                if event.key == pygame.K_ESCAPE:
                    running = False

                # Zoom controls (UP/DOWN)
                elif event.key == pygame.K_UP and not keys_pressed[pygame.K_UP]:
                    keys_pressed[pygame.K_UP] = True
                    controller.zoom_direction = 1
                    controller.update_motors(speed_multiplier)

                elif event.key == pygame.K_DOWN and not keys_pressed[pygame.K_DOWN]:
                    keys_pressed[pygame.K_DOWN] = True
                    controller.zoom_direction = -1
                    controller.update_motors(speed_multiplier)

                # Focus controls (LEFT/RIGHT)
                elif event.key == pygame.K_LEFT and not keys_pressed[pygame.K_LEFT]:
                    keys_pressed[pygame.K_LEFT] = True
                    controller.focus_direction = -1
                    controller.update_motors(speed_multiplier)

                elif event.key == pygame.K_RIGHT and not keys_pressed[pygame.K_RIGHT]:
                    keys_pressed[pygame.K_RIGHT] = True
                    controller.focus_direction = 1
                    controller.update_motors(speed_multiplier)

                # Update speed when shift is pressed while motor is running
                elif event.key in [pygame.K_LSHIFT, pygame.K_RSHIFT]:
                    if controller.focus_direction != 0 or controller.zoom_direction != 0:
                        controller.update_motors(1.0)

            elif event.type == pygame.KEYUP:
                # Stop motors when keys released
                if event.key == pygame.K_UP and keys_pressed[pygame.K_UP]:
                    keys_pressed[pygame.K_UP] = False
                    controller.zoom_direction = 0
                    controller.update_motors(speed_multiplier)

                elif event.key == pygame.K_DOWN and keys_pressed[pygame.K_DOWN]:
                    keys_pressed[pygame.K_DOWN] = False
                    controller.zoom_direction = 0
                    controller.update_motors(speed_multiplier)

                elif event.key == pygame.K_LEFT and keys_pressed[pygame.K_LEFT]:
                    keys_pressed[pygame.K_LEFT] = False
                    controller.focus_direction = 0
                    controller.update_motors(speed_multiplier)

                elif event.key == pygame.K_RIGHT and keys_pressed[pygame.K_RIGHT]:
                    keys_pressed[pygame.K_RIGHT] = False
                    controller.focus_direction = 0
                    controller.update_motors(speed_multiplier)

                # Update speed when shift is released while motor is running
                elif event.key in [pygame.K_LSHIFT, pygame.K_RSHIFT]:
                    if controller.focus_direction != 0 or controller.zoom_direction != 0:
                        controller.update_motors(0.2)

        # Read any serial feedback
        controller.read_serial()

        # Draw UI
        screen.fill(BLACK)

        # Title
        title = font_large.render("CCTV Controller", True, WHITE)
        screen.blit(title, (screen_width//2 - title.get_width()//2, 20))

        # Connection status
        status_text = "Connected" if controller.connected else "Disconnected"
        status_color = GREEN if controller.connected else RED
        status = font_small.render(f"Status: {status_text}", True, status_color)
        screen.blit(status, (20, 80))

        # Speed indicator
        speed_text = "FULL SPEED" if shift_pressed else "20% SPEED"
        speed_color = RED if shift_pressed else BLUE
        speed_display = font_small.render(f"Speed: {speed_text}", True, speed_color)
        screen.blit(speed_display, (20, 110))

        # Focus motor status
        focus_label = font_medium.render("FOCUS:", True, WHITE)
        screen.blit(focus_label, (50, 150))

        focus_color = GREEN if controller.focus_state != "STOPPED" else GRAY
        focus_status = font_medium.render(controller.focus_state, True, focus_color)
        screen.blit(focus_status, (180, 150))

        # Draw LEFT/RIGHT arrows for focus
        left_color = BLUE if keys_pressed[pygame.K_LEFT] else GRAY
        right_color = BLUE if keys_pressed[pygame.K_RIGHT] else GRAY

        pygame.draw.polygon(screen, left_color, [(120, 200), (150, 180), (150, 220)])  # Left arrow
        pygame.draw.polygon(screen, right_color, [(430, 200), (400, 180), (400, 220)])  # Right arrow

        # Zoom motor status
        zoom_label = font_medium.render("ZOOM:", True, WHITE)
        screen.blit(zoom_label, (50, 270))

        zoom_color = GREEN if controller.zoom_state != "STOPPED" else GRAY
        zoom_status = font_medium.render(controller.zoom_state, True, zoom_color)
        screen.blit(zoom_status, (180, 270))

        # Draw UP/DOWN arrows for zoom
        up_color = BLUE if keys_pressed[pygame.K_UP] else GRAY
        down_color = BLUE if keys_pressed[pygame.K_DOWN] else GRAY

        pygame.draw.polygon(screen, up_color, [(275, 180), (255, 210), (295, 210)])  # Up arrow
        pygame.draw.polygon(screen, down_color, [(275, 340), (255, 310), (295, 310)])  # Down arrow

        # Instructions
        esc_text = font_small.render("Press ESC to exit", True, WHITE)
        screen.blit(esc_text, (screen_width//2 - esc_text.get_width()//2, 370))

        # Update display
        pygame.display.flip()
        clock.tick(60)  # 60 FPS

    # Cleanup
    controller.disconnect()
    pygame.quit()
    print("\nExited successfully")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        pygame.quit()
        sys.exit(0)

