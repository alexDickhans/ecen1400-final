/*
 * Button Input Controller with Dual UART Communication and Haptic Feedback
 * 
 * Features:
 * - 5 button inputs (up/down/left/right/select)
 * - Dual UART communication over hardware Serial pins 0/1 and SoftwareSerial pins 4/5
 * - Haptic feedback control on pin 13
 * - Responds to y/n/t characters for haptic control
 * 
 * UART Commands:
 * - 'y': Enable haptics + yes pattern (..)
 * - 'n': Disable haptics + no pattern (---)
 * - 't': Test/signal pattern (-.-)
 * 
 * Pin Configuration:
 * - Button Up: Pin 8
 * - Button Down: Pin 9  
 * - Button Left: Pin 10
 * - Button Right: Pin 11
 * - Button Select: Pin 12
 * - Haptic Motor: Pin 13
 * - Hardware UART: Pins 0/1 (RX/TX)
 * - Software UART: Pins 4/5 (TX- Yellow/RX - Green)
 */

#include <SoftwareSerial.h>

// Pin definitions
#define BUTTON_UP_PIN 8
#define BUTTON_DOWN_PIN 9
#define BUTTON_LEFT_PIN 10
#define BUTTON_RIGHT_PIN 11
#define BUTTON_SELECT_PIN 12
#define HAPTIC_PIN 13
#define SOFT_UART_TX_PIN 4
#define SOFT_UART_RX_PIN 5

// Button debounce settings
#define DEBOUNCE_DELAY 50
#define BUTTON_COUNT 5

// Create SoftwareSerial object for additional UART communication
SoftwareSerial softSerial(SOFT_UART_RX_PIN, SOFT_UART_TX_PIN);

// Button structure to track state and timing
struct Button {
  int pin;
  char command;
  int lastState;
  int currentState;
  unsigned long lastDebounceTime;
  bool pressed;
};

// Initialize button array
Button buttons[BUTTON_COUNT] = {
  {BUTTON_UP_PIN, 'u', HIGH, HIGH, 0, false},
  {BUTTON_DOWN_PIN, 'd', HIGH, HIGH, 0, false},
  {BUTTON_LEFT_PIN, 'l', HIGH, HIGH, 0, false},
  {BUTTON_RIGHT_PIN, 'r', HIGH, HIGH, 0, false},
  {BUTTON_SELECT_PIN, 's', HIGH, HIGH, 0, false}
};

// Haptic feedback settings
bool hapticEnabled = true;
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
  
  // Configure button pins as inputs with pull-up resistors
  for (int i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttons[i].pin, INPUT_PULLUP);
  }
  
  // Configure haptic pin as output
  pinMode(HAPTIC_PIN, OUTPUT);
  digitalWrite(HAPTIC_PIN, LOW);
  
  // Initialize button states
  for (int i = 0; i < BUTTON_COUNT; i++) {
    buttons[i].lastState = digitalRead(buttons[i].pin);
    buttons[i].currentState = buttons[i].lastState;
  }
}

void loop() {
  // Check for incoming UART commands
  checkUARTCommands();
  
  // Process button inputs
  processButtons();
  
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
    // hapticEnabled = true;
    // Play yes pattern: two short pulses
    playHapticPattern("..");
  } else if (command == 'n' || command == 'N') {
    // hapticEnabled = false;
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
 * Process all button inputs with debouncing
 */
void processButtons() {
  for (int i = 0; i < BUTTON_COUNT; i++) {
    // Read current button state
    int reading = digitalRead(buttons[i].pin);
    
    // Check if button state changed (for debouncing)
    if (reading != buttons[i].lastState) {
      buttons[i].lastDebounceTime = millis();
      buttons[i].lastState = reading;  // Update immediately to prevent timer reset
    }
    
    // Apply debounce logic
    if ((millis() - buttons[i].lastDebounceTime) > DEBOUNCE_DELAY) {
      // Button state has been stable for debounce period
      if (reading != buttons[i].currentState) {
        buttons[i].currentState = reading;
        
        // Button pressed (LOW because of pull-up resistor)
        if (buttons[i].currentState == LOW && !buttons[i].pressed) {
          buttons[i].pressed = true;
          
          // Send button command via UART
          sendButtonCommand(buttons[i].command);
          
          // Trigger haptic feedback if enabled
          if (hapticEnabled) {
            triggerHaptic();
          }
        } else if (buttons[i].currentState == HIGH) {
          buttons[i].pressed = false;
        }
      }
    }
  }
}

/**
 * Send button command via both UART interfaces
 */
void sendButtonCommand(char command) {
  Serial.write(command);      // Send via hardware Serial (pins 0/1)
  softSerial.write(command);  // Send via SoftwareSerial (pins 4/5)
}

/**
 * Trigger haptic feedback
 */
void triggerHaptic() {
  if (hapticEnabled) {
    digitalWrite(HAPTIC_PIN, HIGH);
    hapticStartTime = millis();
  }
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
  if (!hapticEnabled) return;
  
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
