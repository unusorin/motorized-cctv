#include <Arduino.h>

// Motor command struct for binary communication
// Positive values = forward, negative = backward, 0 = stop
// Magnitude represents speed (1-127 maps to 35%-100% duty cycle)
struct MotorCommand {
  int8_t focus;  // -127 to +127 (sign=direction, magnitude=speed)
  int8_t zoom;   // -127 to +127 (sign=direction, magnitude=speed)
} __attribute__((packed));

// Motor pin definitions - MX1508 dual H-bridge
// Focus motor (Motor A) - connected to second H-bridge
#define FOCUS_IN1 5
#define FOCUS_IN2 4

// Zoom motor (Motor B) - connected to first H-bridge
#define ZOOM_IN3 7
#define ZOOM_IN4 6

// Optical endstop pin definitions (interrupted by default, optical contact at limit)
#define FOCUS_ENDSTOP_MIN 2
#define FOCUS_ENDSTOP_MAX 3
#define ZOOM_ENDSTOP_MIN 1
#define ZOOM_ENDSTOP_MAX 0

// PWM configuration
#define PWM_FREQ 20000      // 20 kHz PWM frequency
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)
#define DEFAULT_SPEED 255   // Default full speed (can be adjusted 0-255)

// Motor state tracking
bool focusMotorActive = false;
bool zoomMotorActive = false;

// Motor direction tracking: 0=stopped, 1=forward, -1=backward
int focusMotorDirection = 0;
int zoomMotorDirection = 0;

// Motor speed tracking (0-255)
int focusMotorSpeed = DEFAULT_SPEED;
int zoomMotorSpeed = DEFAULT_SPEED;

// Endstop state tracking (to detect changes)
bool focusMinLastState = HIGH;
bool focusMaxLastState = HIGH;
bool zoomMinLastState = HIGH;
bool zoomMaxLastState = HIGH;

// Debounce timing
unsigned long focusMinLastTriggerTime = 0;
unsigned long focusMaxLastTriggerTime = 0;
unsigned long zoomMinLastTriggerTime = 0;
unsigned long zoomMaxLastTriggerTime = 0;
const unsigned long DEBOUNCE_DELAY = 200; // 200ms debounce time

// Command buffer
String commandBuffer = "";

// Function to stop focus motor
void focusStop() {
  // Detach PWM/LEDC channels to prevent buzzing
  ledcDetachPin(FOCUS_IN1);
  ledcDetachPin(FOCUS_IN2);
  // Set pins to OUTPUT mode and both HIGH for active brake (shorts motor windings)
  pinMode(FOCUS_IN1, OUTPUT);
  pinMode(FOCUS_IN2, OUTPUT);
  digitalWrite(FOCUS_IN1, HIGH);
  digitalWrite(FOCUS_IN2, HIGH);
  focusMotorActive = false;
  focusMotorDirection = 0;
  Serial.println("[DEBUG] Focus motor: STOPPED (brake mode)");
}

// Function to move focus motor forward
void focusForward() {
  // Check if MAX endstop is triggered
  if (digitalRead(FOCUS_ENDSTOP_MAX) == LOW) {
    Serial.println("[ERROR] Cannot move focus forward - MAX endstop triggered!");
    focusStop();
    return;
  }
  // Properly detach and clear IN2
  ledcDetachPin(FOCUS_IN2);
  pinMode(FOCUS_IN2, OUTPUT);
  digitalWrite(FOCUS_IN2, LOW);
  // Set IN1 with PWM
  analogWrite(FOCUS_IN1, focusMotorSpeed);
  focusMotorActive = true;
  focusMotorDirection = 1;
  Serial.print("[DEBUG] Focus motor: FORWARD (speed: ");
  Serial.print(focusMotorSpeed);
  Serial.println(")");
}

// Function to move focus motor backward
void focusBack() {
  // Check if MIN endstop is triggered
  if (digitalRead(FOCUS_ENDSTOP_MIN) == LOW) {
    Serial.println("[ERROR] Cannot move focus backward - MIN endstop triggered!");
    focusStop();
    return;
  }
  // Properly detach and clear IN1
  ledcDetachPin(FOCUS_IN1);
  pinMode(FOCUS_IN1, OUTPUT);
  digitalWrite(FOCUS_IN1, LOW);
  // Set IN2 with PWM
  analogWrite(FOCUS_IN2, focusMotorSpeed);
  focusMotorActive = true;
  focusMotorDirection = -1;
  Serial.print("[DEBUG] Focus motor: BACKWARD (speed: ");
  Serial.print(focusMotorSpeed);
  Serial.println(")");
}

// Function to stop zoom motor
void zoomStop() {
  // Detach PWM/LEDC channels to prevent buzzing
  ledcDetachPin(ZOOM_IN3);
  ledcDetachPin(ZOOM_IN4);
  // Set pins to OUTPUT mode and both HIGH for active brake (shorts motor windings)
  pinMode(ZOOM_IN3, OUTPUT);
  pinMode(ZOOM_IN4, OUTPUT);
  digitalWrite(ZOOM_IN3, HIGH);
  digitalWrite(ZOOM_IN4, HIGH);
  zoomMotorActive = false;
  zoomMotorDirection = 0;
  Serial.println("[DEBUG] Zoom motor: STOPPED (brake mode)");
}

// Function to move zoom motor forward
void zoomForward() {
  // Check if MAX endstop is triggered
  if (digitalRead(ZOOM_ENDSTOP_MAX) == LOW) {
    Serial.println("[ERROR] Cannot move zoom forward - MAX endstop triggered!");
    zoomStop();
    return;
  }
  // Properly detach and clear IN3
  ledcDetachPin(ZOOM_IN3);
  pinMode(ZOOM_IN3, OUTPUT);
  digitalWrite(ZOOM_IN3, LOW);
  // Set IN4 with PWM
  analogWrite(ZOOM_IN4, zoomMotorSpeed);
  zoomMotorActive = true;
  zoomMotorDirection = 1;
  Serial.print("[DEBUG] Zoom motor: FORWARD (speed: ");
  Serial.print(zoomMotorSpeed);
  Serial.println(")");
}

// Function to move zoom motor backward
void zoomBack() {
  // Check if MIN endstop is triggered
  if (digitalRead(ZOOM_ENDSTOP_MIN) == LOW) {
    Serial.println("[ERROR] Cannot move zoom backward - MIN endstop triggered!");
    zoomStop();
    return;
  }
  // Properly detach and clear IN4
  ledcDetachPin(ZOOM_IN4);
  pinMode(ZOOM_IN4, OUTPUT);
  digitalWrite(ZOOM_IN4, LOW);
  // Set IN3 with PWM
  analogWrite(ZOOM_IN3, zoomMotorSpeed);
  zoomMotorActive = true;
  zoomMotorDirection = -1;
  Serial.print("[DEBUG] Zoom motor: BACKWARD (speed: ");
  Serial.print(zoomMotorSpeed);
  Serial.println(")");
}

// Function to process binary motor command struct
void processMotorCommand(MotorCommand cmd) {
  Serial.println("[DEBUG] Received binary motor command");
  Serial.print("  Focus: ");
  Serial.print(cmd.focus);
  Serial.print(", Zoom: ");
  Serial.println(cmd.zoom);

  // Process focus motor command
  if (cmd.focus > 0) {
    // Forward: map 1-127 to speed 89-255 (35%-100% duty cycle)
    // This ensures minimum speed has enough torque to overcome static friction
    focusMotorSpeed = map(abs(cmd.focus), 1, 127, 89, 255);
    focusForward();
  } else if (cmd.focus < 0) {
    // Backward: map -1 to -127 to speed 89-255 (35%-100% duty cycle)
    focusMotorSpeed = map(abs(cmd.focus), 1, 127, 89, 255);
    focusBack();
  } else {
    // Stop
    focusStop();
  }

  // Process zoom motor command
  if (cmd.zoom > 0) {
    // Forward: map 1-127 to speed 89-255 (35%-100% duty cycle)
    zoomMotorSpeed = map(abs(cmd.zoom), 1, 127, 89, 255);
    zoomForward();
  } else if (cmd.zoom < 0) {
    // Backward: map -1 to -127 to speed 89-255 (35%-100% duty cycle)
    zoomMotorSpeed = map(abs(cmd.zoom), 1, 127, 89, 255);
    zoomBack();
  } else {
    // Stop
    zoomStop();
  }
}

// Function to process incoming text commands (for backward compatibility)
void processCommand(String command) {
  command.trim(); // Remove any whitespace
  command.toLowerCase(); // Convert to lowercase for easier comparison

  Serial.print("[DEBUG] Received text command: '");
  Serial.print(command);
  Serial.println("'");

  if (command == "focus_stop") {
    focusStop();
  } else if (command == "focus_forward") {
    focusForward();
  } else if (command == "focus_back") {
    focusBack();
  } else if (command == "zoom_stop") {
    zoomStop();
  } else if (command == "zoom_forward") {
    zoomForward();
  } else if (command == "zoom_back") {
    zoomBack();
  } else if (command.startsWith("focus_speed ")) {
    // Parse speed value (0-255)
    int speed = command.substring(12).toInt();
    if (speed >= 0 && speed <= 255) {
      focusMotorSpeed = speed;
      Serial.print("[DEBUG] Focus motor speed set to: ");
      Serial.println(focusMotorSpeed);
    } else {
      Serial.println("[ERROR] Invalid speed value. Must be 0-255.");
    }
  } else if (command.startsWith("zoom_speed ")) {
    // Parse speed value (0-255)
    int speed = command.substring(11).toInt();
    if (speed >= 0 && speed <= 255) {
      zoomMotorSpeed = speed;
      Serial.print("[DEBUG] Zoom motor speed set to: ");
      Serial.println(zoomMotorSpeed);
    } else {
      Serial.println("[ERROR] Invalid speed value. Must be 0-255.");
    }
  } else {
    Serial.print("[ERROR] Unknown command: '");
    Serial.print(command);
    Serial.println("'");
    Serial.println("[INFO] Available commands:");
    Serial.println("  - Text: focus_stop, focus_forward, focus_back");
    Serial.println("  - Text: zoom_stop, zoom_forward, zoom_back");
    Serial.println("  - Text: focus_speed <0-255>, zoom_speed <0-255>");
    Serial.println("  - Binary: Send MotorCommand struct (4 bytes)");
  }
}

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);

  // Wait for serial port to connect (ESP32-C3 needs more time for USB CDC)
  delay(2000);

  // Force a flush to ensure serial is ready
  while (!Serial && millis() < 5000) {
    delay(100);
  }

  // Configure motor pins as outputs
  pinMode(FOCUS_IN1, OUTPUT);
  pinMode(FOCUS_IN2, OUTPUT);
  pinMode(ZOOM_IN3, OUTPUT);
  pinMode(ZOOM_IN4, OUTPUT);

  // Configure optical endstop pins as inputs with pull-ups
  pinMode(FOCUS_ENDSTOP_MIN, INPUT_PULLUP);
  pinMode(FOCUS_ENDSTOP_MAX, INPUT_PULLUP);
  pinMode(ZOOM_ENDSTOP_MIN, INPUT_PULLUP);
  pinMode(ZOOM_ENDSTOP_MAX, INPUT_PULLUP);

  // Initialize all motors to stopped state
  focusStop();
  zoomStop();

  Serial.println("========================================");
  Serial.println("Motorized CCTV Controller - MX1508 H-Bridge");
  Serial.println("========================================");
  Serial.println("[INFO] System initialized successfully!");
  Serial.println("[INFO] Focus motor pins: IN1=GPIO5, IN2=GPIO4");
  Serial.println("[INFO] Zoom motor pins: IN3=GPIO7, IN4=GPIO6");
  Serial.println("[INFO] Focus endstops: MIN=GPIO2, MAX=GPIO3");
  Serial.println("[INFO] Zoom endstops: MIN=GPIO1, MAX=GPIO0");
  Serial.println("[INFO] PWM Configuration:");
  Serial.print("  - Frequency: ");
  Serial.print(PWM_FREQ);
  Serial.println(" Hz");
  Serial.print("  - Resolution: ");
  Serial.print(PWM_RESOLUTION);
  Serial.println(" bits (0-255)");
  Serial.print("  - Default speed: ");
  Serial.println(DEFAULT_SPEED);

  // Read and display initial endstop states
  Serial.println("[INFO] Initial endstop states:");
  Serial.print("  Focus MIN (GPIO2): ");
  Serial.println(digitalRead(FOCUS_ENDSTOP_MIN) == HIGH ? "HIGH (not triggered)" : "LOW (triggered)");
  Serial.print("  Focus MAX (GPIO3): ");
  Serial.println(digitalRead(FOCUS_ENDSTOP_MAX) == HIGH ? "HIGH (not triggered)" : "LOW (triggered)");
  Serial.print("  Zoom MIN (GPIO1): ");
  Serial.println(digitalRead(ZOOM_ENDSTOP_MIN) == HIGH ? "HIGH (not triggered)" : "LOW (triggered)");
  Serial.print("  Zoom MAX (GPIO0): ");
  Serial.println(digitalRead(ZOOM_ENDSTOP_MAX) == HIGH ? "HIGH (not triggered)" : "LOW (triggered)");

  Serial.println("[INFO] Communication protocols:");
  Serial.println("  1. Text commands (backward compatible):");
  Serial.println("     - focus_stop, focus_forward, focus_back");
  Serial.println("     - zoom_stop, zoom_forward, zoom_back");
  Serial.println("     - focus_speed <0-255>, zoom_speed <0-255>");
  Serial.println("  2. Binary struct (2 bytes):");
  Serial.println("     - Byte 0: focus (-127 to +127)");
  Serial.println("     - Byte 1: zoom (-127 to +127)");
  Serial.println("     - Positive=forward, Negative=backward, 0=stop");
  Serial.println("     - Magnitude=speed (1-127 maps to 35%-100% duty cycle)");
  Serial.println("[INFO] Ready to receive commands...");
  Serial.println("========================================");

  // Initialize last states with current readings
  focusMinLastState = digitalRead(FOCUS_ENDSTOP_MIN);
  focusMaxLastState = digitalRead(FOCUS_ENDSTOP_MAX);
  zoomMinLastState = digitalRead(ZOOM_ENDSTOP_MIN);
  zoomMaxLastState = digitalRead(ZOOM_ENDSTOP_MAX);
}

void loop() {
  // Check if data is available on serial port
  if (Serial.available() >= sizeof(MotorCommand)) {
    // Peek at first byte to determine if it's binary or text
    uint8_t firstByte = Serial.peek();

    // Binary commands will have values in range [-127, 127]
    // Text commands will be ASCII characters (typically >= 0x20 or printable)
    if (firstByte <= 127 || firstByte >= 129) {
      // Could be binary - check if looks like valid motor command
      uint8_t buffer[2];
      Serial.readBytes(buffer, 2);

      // Reinterpret as signed values
      int8_t focus = (int8_t)buffer[0];
      int8_t zoom = (int8_t)buffer[1];

      // If both values are in valid range or look like text, process accordingly
      if ((buffer[0] >= 0x20 && buffer[0] <= 0x7E) && (buffer[1] >= 0x20 && buffer[1] <= 0x7E)) {
        // Looks like ASCII text, add to command buffer
        commandBuffer += (char)buffer[0];
        if (buffer[1] == '\n' || buffer[1] == '\r') {
          if (commandBuffer.length() > 0) {
            processCommand(commandBuffer);
            commandBuffer = "";
          }
        } else {
          commandBuffer += (char)buffer[1];
        }
      } else {
        // Treat as binary command
        MotorCommand cmd;
        cmd.focus = focus;
        cmd.zoom = zoom;
        processMotorCommand(cmd);
      }
    } else {
      // Definitely text - read character by character
      char inChar = (char)Serial.read();
      if (inChar == '\n' || inChar == '\r') {
        if (commandBuffer.length() > 0) {
          processCommand(commandBuffer);
          commandBuffer = "";
        }
      } else {
        commandBuffer += inChar;
      }
    }
  } else if (Serial.available() > 0) {
    // Less than struct size available, process as text
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      commandBuffer += inChar;
    }
  }

  // Check optical endstops (LOW = optical contact/limit reached)
  // ...existing code...
  unsigned long currentTime = millis();

  // Focus motor endstops
  bool focusMinState = digitalRead(FOCUS_ENDSTOP_MIN);
  if (focusMinState != focusMinLastState && (currentTime - focusMinLastTriggerTime) > DEBOUNCE_DELAY) {
    if (focusMinState == LOW) {
      // MIN endstop triggered - stop if moving backward
      if (focusMotorActive && focusMotorDirection == -1) {
        focusStop();
      }
      Serial.println("[ENDSTOP] Focus MIN endstop (GPIO2) triggered!");
    } else {
      Serial.println("[ENDSTOP] Focus MIN endstop (GPIO2) released");
    }
    focusMinLastState = focusMinState;
    focusMinLastTriggerTime = currentTime;
  }

  bool focusMaxState = digitalRead(FOCUS_ENDSTOP_MAX);
  if (focusMaxState != focusMaxLastState && (currentTime - focusMaxLastTriggerTime) > DEBOUNCE_DELAY) {
    if (focusMaxState == LOW) {
      // MAX endstop triggered - stop if moving forward
      if (focusMotorActive && focusMotorDirection == 1) {
        focusStop();
      }
      Serial.println("[ENDSTOP] Focus MAX endstop (GPIO3) triggered!");
    } else {
      Serial.println("[ENDSTOP] Focus MAX endstop (GPIO3) released");
    }
    focusMaxLastState = focusMaxState;
    focusMaxLastTriggerTime = currentTime;
  }

  // Zoom motor endstops
  bool zoomMinState = digitalRead(ZOOM_ENDSTOP_MIN);
  if (zoomMinState != zoomMinLastState && (currentTime - zoomMinLastTriggerTime) > DEBOUNCE_DELAY) {
    if (zoomMinState == LOW) {
      // MIN endstop triggered - stop if moving backward
      if (zoomMotorActive && zoomMotorDirection == -1) {
        zoomStop();
      }
      Serial.println("[ENDSTOP] Zoom MIN endstop (GPIO1) triggered!");
    } else {
      Serial.println("[ENDSTOP] Zoom MIN endstop (GPIO1) released");
    }
    zoomMinLastState = zoomMinState;
    zoomMinLastTriggerTime = currentTime;
  }

  bool zoomMaxState = digitalRead(ZOOM_ENDSTOP_MAX);
  if (zoomMaxState != zoomMaxLastState && (currentTime - zoomMaxLastTriggerTime) > DEBOUNCE_DELAY) {
    if (zoomMaxState == LOW) {
      // MAX endstop triggered - stop if moving forward
      if (zoomMotorActive && zoomMotorDirection == 1) {
        zoomStop();
      }
      Serial.println("[ENDSTOP] Zoom MAX endstop (GPIO0) triggered!");
    } else {
      Serial.println("[ENDSTOP] Zoom MAX endstop (GPIO0) released");
    }
    zoomMaxLastState = zoomMaxState;
    zoomMaxLastTriggerTime = currentTime;
  }

  // Small delay to prevent CPU overload
  delay(10);
}

