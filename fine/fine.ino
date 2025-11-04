/*
 * Joystick Input Controller with Dual UART Communication and Haptic Feedback
 * 
 * Features:
 * - Joystick inputs (up/down/left/right/select) using SparkFun joystick
 * - Dual UART communication over hardware Serial pins 0/1 and SoftwareSerial pins 4/5
 * - Haptic feedback control on pin 13
 * - Responds to y/n/t characters for haptic control
 * 
 * UART Commands:
 * - 'y': Yes pattern (..)
 * - 'n': No pattern (---)
 * - 't': Test/signal pattern (-.-)
 * 
 * Pin Configuration:
 * - Joystick Horizontal (X): Pin A0
 * - Joystick Vertical (Y): Pin A1
 * - Joystick Select Button: Pin 3 (INPUT_PULLUP)
 * - Haptic Motor: Pin 13
 * - Hardware UART: Pins 0/1 (RX/TX)
 * - Software UART: Pins 4/5 (TX- Yellow/RX - Green)
 * 
 * Joystick Thresholds:
 * - Left: A0 < 400
 * - Right: A0 > 600
 * - Up: A1 < 400
 * - Down: A1 > 600
 * - Center: 400 <= A0 <= 600 AND 400 <= A1 <= 600
 */

#include <SoftwareSerial.h>

// Pin definitions
#define JOYSTICK_X_PIN A0
#define JOYSTICK_Y_PIN A1
#define JOYSTICK_SELECT_PIN 3
#define HAPTIC_PIN 13
#define SOFT_UART_TX_PIN 7
#define SOFT_UART_RX_PIN 6

// Joystick threshold settings
#define JOYSTICK_THRESHOLD_LOW 400
#define JOYSTICK_THRESHOLD_HIGH 600
#define JOYSTICK_CENTER_MIN 400
#define JOYSTICK_CENTER_MAX 600

// Debounce settings
#define DEBOUNCE_DELAY 50
#define JOYSTICK_DEBOUNCE_DELAY 100  // Longer debounce for joystick movement

// Create SoftwareSerial object for additional UART communication
SoftwareSerial softSerial(SOFT_UART_RX_PIN, SOFT_UART_TX_PIN);

// Joystick state tracking
struct JoystickState {
  int xValue;
  int yValue;
  bool selectPressed;
  int lastSelectState;
  unsigned long lastSelectDebounceTime;
  bool selectPressedFlag;
  
  // Direction states
  bool leftPressed;
  bool rightPressed;
  bool upPressed;
  bool downPressed;
  
  // Last direction states for edge detection
  bool lastLeftPressed;
  bool lastRightPressed;
  bool lastUpPressed;
  bool lastDownPressed;
  
  // Direction command sent flags
  bool leftCommandSent;
  bool rightCommandSent;
  bool upCommandSent;
  bool downCommandSent;
  
  // Debounce timers for directions
  unsigned long lastLeftDebounceTime;
  unsigned long lastRightDebounceTime;
  unsigned long lastUpDebounceTime;
  unsigned long lastDownDebounceTime;
};

// Initialize joystick state
JoystickState joystick = {
  512, 512, false, HIGH, 0, false,
  false, false, false, false,
  false, false, false, false,
  false, false, false, false,
  0, 0, 0, 0
};

// Haptic feedback settings
unsigned long hapticStartTime = 0;
unsigned long hapticDuration = 100; // milliseconds

// Haptic pattern settings
#define SHORT_PULSE_DURATION 100   // milliseconds for short pulse (.)
#define LONG_PULSE_DURATION 300    // milliseconds for long pulse (-)
#define PAUSE_BETWEEN_PULSES 150   // milliseconds between pulses in pattern

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  softSerial.begin(9600);
  
  // Configure joystick select pin as input with pull-up resistor
  pinMode(JOYSTICK_SELECT_PIN, INPUT_PULLUP);
  
  // Configure haptic pin as output
  pinMode(HAPTIC_PIN, OUTPUT);
  digitalWrite(HAPTIC_PIN, LOW);
  
  // Initialize joystick states
  joystick.lastSelectState = digitalRead(JOYSTICK_SELECT_PIN);
  joystick.selectPressed = (joystick.lastSelectState == LOW);
  
  // Read initial joystick position
  joystick.xValue = analogRead(JOYSTICK_X_PIN);
  joystick.yValue = analogRead(JOYSTICK_Y_PIN);
  
  // Initialize direction states based on initial position
  updateJoystickDirections();
  joystick.lastLeftPressed = joystick.leftPressed;
  joystick.lastRightPressed = joystick.rightPressed;
  joystick.lastUpPressed = joystick.upPressed;
  joystick.lastDownPressed = joystick.downPressed;
}

void loop() {
  // Check for incoming UART commands
  checkUARTCommands();
  
  // Process joystick inputs
  processJoystick();
  
  // Handle haptic feedback timing
  handleHapticFeedback();
  
  // Small delay for stability
  delay(10);
}

/**
 * Check for incoming UART commands and process them
 */
void checkUARTCommands() {
  // Check hardware Serial (pins 0/1)
  if (Serial.available()) {
    char command = Serial.read();
    processUARTCommand(command);
  }
  
  // Check SoftwareSerial (pins 4/5)
  if (softSerial.available()) {
    char command = softSerial.read();
    processUARTCommand(command);
  }
}

/**
 * Process UART commands for haptic control
 */
void processUARTCommand(char command) {
  // Handle haptic control commands
  if (command == 'y' || command == 'Y') {
    // Play yes pattern: two short pulses
    playHapticPattern("..");
  } else if (command == 'n' || command == 'N') {
    // Play no pattern: three long pulses
    playHapticPattern("---");
  } else if (command == 't' || command == 'T') {
    // Play test/signal pattern: long-short-long pulses
    playHapticPattern("-.-");
  } else if (command == 'w' || command == 'W') {
    // Play win pattern: four dots, two dashes (....--)
    playHapticPattern("....--");
  } else if (command == 'l' || command == 'L') {
    // Play lose pattern: four dashes, two dots (----..)
    playHapticPattern("----..");
  }
}

/**
 * Process joystick inputs with debouncing
 */
void processJoystick() {
  // Read current joystick values
  joystick.xValue = analogRead(JOYSTICK_X_PIN);
  joystick.yValue = analogRead(JOYSTICK_Y_PIN);
  
  // Process select button
  processSelectButton();
  
  // Process directional inputs
  processDirectionalInputs();
}

/**
 * Process select button with debouncing
 */
void processSelectButton() {
  // Read current select button state
  int reading = digitalRead(JOYSTICK_SELECT_PIN);
  
  // Check if button state changed (for debouncing)
  if (reading != joystick.lastSelectState) {
    joystick.lastSelectDebounceTime = millis();
    joystick.lastSelectState = reading;  // Update immediately to prevent timer reset
  }
  
  // Apply debounce logic
  if ((millis() - joystick.lastSelectDebounceTime) > DEBOUNCE_DELAY) {
    // Button state has been stable for debounce period
    bool currentPressed = (reading == LOW);
    if (currentPressed != joystick.selectPressed) {
      joystick.selectPressed = currentPressed;
      
      // Button pressed (LOW because of pull-up resistor)
      if (joystick.selectPressed && !joystick.selectPressedFlag) {
        joystick.selectPressedFlag = true;
        
        // Send select command via UART
        sendJoystickCommand('s');
      } else if (!joystick.selectPressed) {
        joystick.selectPressedFlag = false;
      }
    }
  }
}

/**
 * Process directional inputs with debouncing
 */
void processDirectionalInputs() {
  // Update current direction states
  updateJoystickDirections();
  
  // Check for left direction
  if (joystick.leftPressed != joystick.lastLeftPressed) {
    joystick.lastLeftDebounceTime = millis();
    joystick.lastLeftPressed = joystick.leftPressed;
    joystick.leftCommandSent = false;  // Reset command sent flag on state change
  }
  if ((millis() - joystick.lastLeftDebounceTime) > JOYSTICK_DEBOUNCE_DELAY) {
    if (joystick.leftPressed && !joystick.leftCommandSent) {
      // Right direction activated and debounced
      sendJoystickCommand('r');
      joystick.leftCommandSent = true;
    }
  }
  
  // Check for right direction
  if (joystick.rightPressed != joystick.lastRightPressed) {
    joystick.lastRightDebounceTime = millis();
    joystick.lastRightPressed = joystick.rightPressed;
    joystick.rightCommandSent = false;  // Reset command sent flag on state change
  }
  if ((millis() - joystick.lastRightDebounceTime) > JOYSTICK_DEBOUNCE_DELAY) {
    if (joystick.rightPressed && !joystick.rightCommandSent) {
      // Left direction activated and debounced
      sendJoystickCommand('l');
      joystick.rightCommandSent = true;
    }
  }
  
  // Check for up direction
  if (joystick.upPressed != joystick.lastUpPressed) {
    joystick.lastUpDebounceTime = millis();
    joystick.lastUpPressed = joystick.upPressed;
    joystick.upCommandSent = false;  // Reset command sent flag on state change
  }
  if ((millis() - joystick.lastUpDebounceTime) > JOYSTICK_DEBOUNCE_DELAY) {
    if (joystick.upPressed && !joystick.upCommandSent) {
      // Down direction activated and debounced
      sendJoystickCommand('d');
      joystick.upCommandSent = true;
    }
  }
  
  // Check for down direction
  if (joystick.downPressed != joystick.lastDownPressed) {
    joystick.lastDownDebounceTime = millis();
    joystick.lastDownPressed = joystick.downPressed;
    joystick.downCommandSent = false;  // Reset command sent flag on state change
  }
  if ((millis() - joystick.lastDownDebounceTime) > JOYSTICK_DEBOUNCE_DELAY) {
    if (joystick.downPressed && !joystick.downCommandSent) {
      // Up direction activated and debounced
      sendJoystickCommand('u');
      joystick.downCommandSent = true;
    }
  }
}

/**
 * Update joystick direction states based on current analog values
 */
void updateJoystickDirections() {
  // Update direction states based on thresholds
  joystick.leftPressed = (joystick.xValue < JOYSTICK_THRESHOLD_LOW);
  joystick.rightPressed = (joystick.xValue > JOYSTICK_THRESHOLD_HIGH);
  joystick.upPressed = (joystick.yValue < JOYSTICK_THRESHOLD_LOW);
  joystick.downPressed = (joystick.yValue > JOYSTICK_THRESHOLD_HIGH);
}

/**
 * Send joystick command via both UART interfaces
 */
void sendJoystickCommand(char command) {
  Serial.write(command);      // Send via hardware Serial (pins 0/1)
  softSerial.write(command);  // Send via SoftwareSerial (pins 4/5)
}

/**
 * Trigger haptic feedback
 */
void triggerHaptic() {
  digitalWrite(HAPTIC_PIN, HIGH);
  hapticStartTime = millis();
}

/**
 * Handle haptic feedback timing
 */
void handleHapticFeedback() {
  if (digitalRead(HAPTIC_PIN) == HIGH) {
    if (millis() - hapticStartTime >= hapticDuration) {
      digitalWrite(HAPTIC_PIN, LOW);
    }
  }
}

/**
 * Play haptic pattern
 */
void playHapticPattern(String pattern) {
  for (int i = 0; i < pattern.length(); i++) {
    char pulse = pattern.charAt(i);
    
    // Activate haptic motor
    digitalWrite(HAPTIC_PIN, HIGH);
    
    // Determine pulse duration
    if (pulse == '.') {
      delay(SHORT_PULSE_DURATION);
    } else if (pulse == '-') {
      delay(LONG_PULSE_DURATION);
    }
    
    // Turn off haptic motor
    digitalWrite(HAPTIC_PIN, LOW);
    
    // Add pause between pulses (except for last pulse)
    if (i < pattern.length() - 1) {
      delay(PAUSE_BETWEEN_PULSES);
    }
  }
}
