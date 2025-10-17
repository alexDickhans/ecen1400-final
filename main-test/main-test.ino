/*
 * Main Board Controller Test Utility
 * 
 * This utility allows testing communication with both coarse controllers.
 * 
 * Usage:
 * - Send commands: "1 t" or "2 w" (controller number, then command)
 * - Controller 1: Connected to pins 4/5 (RX/TX)
 * - Controller 2: Connected to pins 6/7 (RX/TX)
 * 
 * Available commands:
 * - 'u', 'd', 'l', 'r': Movement (up, down, left, right)
 * - 's': Select/start
 * - 't': Turn notification
 * - 'y': Yes/valid move
 * - 'n': No/invalid move
 * - 'w': Win pattern
 * - 'l': Lose pattern
 */

#include <SoftwareSerial.h>

// SoftwareSerial for communication with coarse controllers
SoftwareSerial coarseSerial1(3, 4); // RX, TX for first controller
SoftwareSerial coarseSerial2(5, 6); // RX, TX for second controller

void setup() {
  Serial.begin(115200);
  Serial.println("Main Board Controller Test Utility");
  Serial.println("==================================");
  
  // Initialize SoftwareSerial for coarse controllers
  coarseSerial1.begin(9600);
  coarseSerial2.begin(9600);
  
  Serial.println("Controllers initialized:");
  Serial.println("- Controller 1: pins 4/5");
  Serial.println("- Controller 2: pins 6/7");
  Serial.println();
  
  printHelp();
}

void loop() {
  // Check for input from Serial (computer)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processSerialCommand(command);
  }
  
  // Check for input from Controller 1
  if (coarseSerial1.available()) {
    char received = coarseSerial1.read();
    Serial.print("Controller 1 -> Main: '");
    Serial.print(received);
    Serial.println("'");
  }
  
  // Check for input from Controller 2
  if (coarseSerial2.available()) {
    char received = coarseSerial2.read();
    Serial.print("Controller 2 -> Main: '");
    Serial.print(received);
    Serial.println("'");
  }
  
  delay(10); // Small delay to prevent overwhelming the serial buffer
}

void processSerialCommand(String command) {
  // Handle special commands
  if (command == "help" || command == "h") {
    printHelp();
    return;
  }
  
  if (command == "clear" || command == "c") {
    Serial.println("\n--- Clearing screen ---\n");
    return;
  }
  
  // Parse controller command format: "1 t" or "2 w"
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println("Error: Invalid format. Use '1 t' or '2 w'");
    return;
  }
  
  String controllerStr = command.substring(0, spaceIndex);
  String charStr = command.substring(spaceIndex + 1);
  
  // Validate controller number
  int controller = controllerStr.toInt();
  if (controller != 1 && controller != 2) {
    Serial.println("Error: Controller must be 1 or 2");
    return;
  }
  
  // Validate character
  if (charStr.length() != 1) {
    Serial.println("Error: Command must be a single character");
    return;
  }
  
  char cmd = charStr.charAt(0);
  
  // Send command to specified controller
  Serial.print("Main -> Controller ");
  Serial.print(controller);
  Serial.print(": '");
  Serial.print(cmd);
  Serial.println("'");
  
  if (controller == 1) {
    coarseSerial1.write(cmd);
  } else {
    coarseSerial2.write(cmd);
  }
}

void printHelp() {
  Serial.println("Command Format: [controller] [command]");
  Serial.println("Examples:");
  Serial.println("  1 t    - Send 't' (turn notification) to controller 1");
  Serial.println("  2 w    - Send 'w' (win pattern) to controller 2");
  Serial.println("  1 s    - Send 's' (select) to controller 1");
  Serial.println();
  Serial.println("Available Commands:");
  Serial.println("  Movement: u (up), d (down), l (left), r (right)");
  Serial.println("  Game:     s (select/start)");
  Serial.println("  Feedback: t (turn), y (yes), n (no), w (win), l (lose)");
  Serial.println();
  Serial.println("Special Commands:");
  Serial.println("  help     - Show this help");
  Serial.println("  clear    - Clear screen");
  Serial.println();
  Serial.println("Monitor Mode: This utility will also display any");
  Serial.println("messages received from the controllers.");
  Serial.println("=====================================");
}
