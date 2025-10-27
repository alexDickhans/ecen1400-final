/*
 * 5x5 Servo Grid Controller using 2 PCA9685 Boards
 * 
 * Board Configuration:
 * - First PCA9685: A0 bridged (address 0x40), controls servos 0-14 (first 15 servos)
 * - Second PCA9685: All address pins grounded (address 0x41), controls servos 15-24 (last 10 servos)
 * 
 * Grid Layout (5x5):
 * Servo positions in grid:
 *  0  1  2  3  4
 *  5  6  7  8  9
 * 10 11 12 13 14
 * 15 16 17 18 19
 * 20 21 22 23 24
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

// Create PCA9685 objects
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First board (A0 bridged)
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Second board (all grounded)

// AltSoftSerial for communication with X player controller (more reliable)
AltSoftSerial playerXSerial; // Uses pins 8/9 (RX/TX)
// SoftwareSerial for communication with O player controller
SoftwareSerial playerOSerial(5, 6); // RX, TX for O player controller

// Button pins
#define SELECT_BUTTON_PIN 7
#define RESET_START_BUTTON_PIN 8

// Servo configuration
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN 150 // Minimum pulse length count (out of 4096)
#define SERVO_MAX 600 // Maximum pulse length count (out of 4096)

// Grid dimensions
#define GRID_SIZE 5
#define TOTAL_SERVOS 25

// Simulation mode - set to 1 to enable simulation (no servo movement, prints board state)
#define SIMULATION_MODE 1

// Servo states (3 distinct positions for Gomoku)
enum ServoState {
  STATE_NONE = 0,   // No piece (low position)
  STATE_X = 1,      // X piece (middle position)  
  STATE_O = 2       // O piece (high position)
};

// Grid state array - tracks current state of each servo
ServoState gridState[TOTAL_SERVOS];

// Game state variables
int cursorRow = 0;
int cursorCol = 0;
ServoState currentPlayer = STATE_X; // X starts first
bool gameActive = false; // Start inactive
bool gameStarted = false;
unsigned long lastWiggleTime = 0;
bool isWiggling = false;
int wiggleStep = 0;

// Checkerboard animation variables
unsigned long lastPatternTime = 0;
bool checkerboardState = false; // false = X pattern, true = O pattern

// Win sequence variables
bool winSequenceActive = false;
unsigned long winSequenceStartTime = 0;
int winSequenceStep = 0;
int lastMoveRow = 0;
int lastMoveCol = 0;
ServoState winningPlayer = STATE_NONE;

// Button state variables
bool selectButtonPressed = false;
bool resetStartButtonPressed = false;
bool lastSelectButtonState = HIGH;
bool lastResetStartButtonState = HIGH;
unsigned long lastSelectDebounceTime = 0;
unsigned long lastResetStartDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce delay

// Servo position mapping for each state
const int servoPositions[3] = {
  SERVO_MIN,                    // STATE_NONE
  (SERVO_MIN + SERVO_MAX) / 2,  // STATE_X
  SERVO_MAX                     // STATE_O
};

void setup() {
  Serial.begin(115200);
  Serial.println("Gomoku Game Starting...");
  
  // Initialize AltSoftSerial for X player controller
  playerXSerial.begin(9600);
  // Initialize SoftwareSerial for O player controller
  playerOSerial.begin(9600);
  
  // Initialize button pins
  pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_START_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize PCA9685 boards (only if not in simulation mode)
  #if SIMULATION_MODE
    Serial.println("SIMULATION MODE ENABLED - Servos will not move, board state will be printed to serial");
  #else
    pwm1.begin();
    pwm1.setOscillatorFrequency(27000000);
    pwm1.setPWMFreq(SERVO_FREQ);
    
    pwm2.begin();
    pwm2.setOscillatorFrequency(27000000);
    pwm2.setPWMFreq(SERVO_FREQ);
  #endif
  
  // Initialize game board
  initializeGame();
  
  Serial.println("Gomoku game initialized!");
  #if SIMULATION_MODE
    Serial.println("SIMULATION MODE: Board state will be printed instead of moving servos");
  #else
    Serial.println("Press RESET/START button to begin the game");
  #endif
  Serial.println("Controls:");
  Serial.println("  - X player controller: pins 8/9 (AltSoftSerial) - u/d/l/r to move, s to select");
  Serial.println("  - O player controller: pins 5/6 (SoftwareSerial) - u/d/l/r to move, s to select");
  Serial.println("  - SELECT button (pin 7): Make move during game");
  Serial.println("  - RESET/START button (pin 8): Start game or reset during game");
  Serial.println("Commands: 'display' to show board");
}

void loop() {
  if (!gameActive) {
    // Show checkerboard pattern when game is inactive
    updateCheckerboardPattern();
    
    // Check for start button input
    checkControllerInput();
    
    // Check for button input
    checkButtonInput();
    
    // Check for serial commands (for debugging)
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      
      if (command == "start") {
        startGame();
      } else if (command == "display") {
        displayGameState();
      }
    }
    
    delay(50);
    return;
  }
  
  // Handle win sequence if active
  if (winSequenceActive) {
    updateWinSequence();
    delay(100);
    return;
  }
  
  // Handle cursor wiggle animation
  updateCursorWiggle();
  
  // Check for input from player controllers
  checkControllerInput();
  
  // Check for button input
  checkButtonInput();
  
  // Check for serial commands (for debugging)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "display") {
      displayGameState();
    } else if (command == "reset") {
      resetGame();
    }
  }
  
  delay(50);
}

/**
 * Initialize game board and servos
 */
void initializeGame() {
  // Initialize all servos to NONE position
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gridState[i] = STATE_NONE;
    setServoPosition(i, servoPositions[STATE_NONE]);
  }
  
  // Reset game state
  cursorRow = 0;
  cursorCol = 0;
  currentPlayer = STATE_X;
  gameActive = false; // Start inactive
  gameStarted = false;
  isWiggling = false;
  checkerboardState = false;
  lastPatternTime = millis();
  winSequenceActive = false;
  winningPlayer = STATE_NONE;
  
  delay(1000); // Allow servos to reach position
  
  // Start with checkerboard pattern
  updateCheckerboardPattern();
}

/**
 * Set a specific servo in the grid to a given state
 * @param row - Row index (0-4)
 * @param col - Column index (0-4)
 * @param state - Servo state (STATE_NONE, STATE_X, STATE_O)
 */
void setGridState(int row, int col, ServoState state) {
  if (row < 0 || row >= GRID_SIZE || col < 0 || col >= GRID_SIZE) {
    Serial.println("Error: Grid coordinates out of range");
    return;
  }
  
  int servoIndex = row * GRID_SIZE + col;
  gridState[servoIndex] = state;
  setServoPosition(servoIndex, servoPositions[state]);
}

/**
 * Set all servos to the same state
 * @param state - Servo state for all servos
 */
void setAllServos(ServoState state) {
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gridState[i] = state;
    setServoPosition(i, servoPositions[state]);
  }
}

/**
 * Set a specific servo position
 * @param servoIndex - Servo index (0-24)
 * @param position - Pulse length (150-600)
 */
void setServoPosition(int servoIndex, int position) {
  if (servoIndex < 0 || servoIndex >= TOTAL_SERVOS) {
    return;
  }
  
  #if SIMULATION_MODE
    // In simulation mode, just print the board state instead of moving servos
    printSimulationBoard();
    return;
  #else
    // Determine which PCA9685 board to use
    if (servoIndex < 15) {
      // First 15 servos on first PCA9685 (A0 bridged)
      pwm1.setPWM(servoIndex, 0, position);
    } else {
      // Last 10 servos on second PCA9685 (all address pins grounded)
      int channel = servoIndex - 15;
      pwm2.setPWM(channel, 0, position);
    }
  #endif
}

/**
 * Display current game state in serial monitor
 */
void displayGameState() {
  Serial.println("\nCurrent Game State:");
  Serial.println("  0 1 2 3 4");
  
  for (int row = 0; row < GRID_SIZE; row++) {
    Serial.print(row);
    Serial.print(" ");
    for (int col = 0; col < GRID_SIZE; col++) {
      int servoIndex = row * GRID_SIZE + col;
      char piece = ' ';
      if (gridState[servoIndex] == STATE_X) piece = 'X';
      else if (gridState[servoIndex] == STATE_O) piece = 'O';
      
      if (row == cursorRow && col == cursorCol) {
        Serial.print("[");
        Serial.print(piece);
        Serial.print("]");
      } else {
        Serial.print(" ");
        Serial.print(piece);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  Serial.print("Current Player: ");
  Serial.println(currentPlayer == STATE_X ? "X" : "O");
  Serial.print("Cursor: (");
  Serial.print(cursorRow);
  Serial.print(",");
  Serial.print(cursorCol);
  Serial.println(")\n");
}

/**
 * Print board state in simulation mode (- for empty, x for X, o for O)
 */
void printSimulationBoard() {
  Serial.println("\n=== SIMULATION BOARD ===");
  for (int row = 0; row < GRID_SIZE; row++) {
    for (int col = 0; col < GRID_SIZE; col++) {
      int servoIndex = row * GRID_SIZE + col;
      char piece = '-';
      if (gridState[servoIndex] == STATE_X) piece = 'x';
      else if (gridState[servoIndex] == STATE_O) piece = 'o';
      
      Serial.print(piece);
      if (col < GRID_SIZE - 1) Serial.print(" ");
    }
    Serial.println();
  }
  Serial.print("Current Player: ");
  Serial.println(currentPlayer == STATE_X ? "X" : "O");
  Serial.print("Cursor: (");
  Serial.print(cursorRow);
  Serial.print(",");
  Serial.print(cursorCol);
  Serial.println(")");
  Serial.println("=======================\n");
}

/**
 * Check for input from player controllers
 */
void checkControllerInput() {
  // Check X player controller (pins 8/9 - AltSoftSerial)
  if (playerXSerial.available()) {
    char command = playerXSerial.read();
    processGameInput(command, STATE_X);
  }
  
  // Check O player controller (pins 5/6 - SoftwareSerial)
  if (playerOSerial.available()) {
    char command = playerOSerial.read();
    processGameInput(command, STATE_O);
  }
}

/**
 * Check for button input with debouncing
 */
void checkButtonInput() {
  // Read current button states
  bool currentSelectButton = digitalRead(SELECT_BUTTON_PIN);
  bool currentResetStartButton = digitalRead(RESET_START_BUTTON_PIN);
  
  // Check for select button press (falling edge with debounce)
  if (currentSelectButton != lastSelectButtonState) {
    lastSelectDebounceTime = millis();
  }
  
  if ((millis() - lastSelectDebounceTime) > DEBOUNCE_DELAY) {
    if (currentSelectButton == LOW && !selectButtonPressed) {
      selectButtonPressed = true;
      handleSelectButton();
    } else if (currentSelectButton == HIGH) {
      selectButtonPressed = false;
    }
  }
  
  // Check for reset/start button press (falling edge with debounce)
  if (currentResetStartButton != lastResetStartButtonState) {
    lastResetStartDebounceTime = millis();
  }
  
  if ((millis() - lastResetStartDebounceTime) > DEBOUNCE_DELAY) {
    if (currentResetStartButton == LOW && !resetStartButtonPressed) {
      resetStartButtonPressed = true;
      handleResetStartButton();
    } else if (currentResetStartButton == HIGH) {
      resetStartButtonPressed = false;
    }
  }
  
  // Update last button states
  lastSelectButtonState = currentSelectButton;
  lastResetStartButtonState = currentResetStartButton;
}

/**
 * Handle select button press
 */
void handleSelectButton() {
  if (gameActive) {
    // During active game, select button makes a move
    makeMove();
  }
  // If game is not active, select button does nothing
}

/**
 * Handle reset/start button press
 */
void handleResetStartButton() {
  if (!gameActive) {
    // If game is not active, start the game
    startGame();
  } else {
    // If game is active, reset the game
    resetGame();
  }
}

/**
 * Process game input commands
 */
void processGameInput(char command, ServoState player) {
  if (!gameActive) {
    // Only handle start command when game is inactive
    if (command == 's') {
      startGame();
    }
    return;
  }
  
  // Only process input from the current player
  if (player != currentPlayer) {
    return;
  }
  
  switch (command) {
    case 'u': // Up
      moveCursor(-1, 0);
      break;
    case 'd': // Down
      moveCursor(1, 0);
      break;
    case 'l': // Left
      moveCursor(0, -1);
      break;
    case 'r': // Right
      moveCursor(0, 1);
      break;
    case 's': // Select
      makeMove();
      break;
  }
}

/**
 * Move cursor to new position
 */
void moveCursor(int deltaRow, int deltaCol) {
  int newRow = cursorRow + deltaRow;
  int newCol = cursorCol + deltaCol;
  
  // Check bounds
  if (newRow >= 0 && newRow < GRID_SIZE && newCol >= 0 && newCol < GRID_SIZE) {
    cursorRow = newRow;
    cursorCol = newCol;
    startCursorWiggle();
  }
}

/**
 * Make a move at current cursor position
 */
void makeMove() {
  int servoIndex = cursorRow * GRID_SIZE + cursorCol;
  
  // Check if position is empty
  if (gridState[servoIndex] == STATE_NONE) {
    // Valid move
    setGridState(cursorRow, cursorCol, currentPlayer);
    sendFeedback('y'); // Yes - move is valid
    
    // Check for win
    if (checkWin(cursorRow, cursorCol, currentPlayer)) {
      startWinSequence(cursorRow, cursorCol, currentPlayer);
      return;
    }
    
    // Switch players
    currentPlayer = (currentPlayer == STATE_X) ? STATE_O : STATE_X;
    
    // Notify the new player that it's their turn
    sendTurnNotification();
    
    // Move cursor to next available position
    moveToNextEmpty();
    
  } else {
    // Invalid move - position already taken
    sendFeedback('n'); // No - move is invalid
  }
}

/**
 * Send feedback to controllers
 */
void sendFeedback(char response) {
  playerXSerial.write(response);
  playerOSerial.write(response);
}

/**
 * Send turn notification to appropriate controller
 */
void sendTurnNotification() {
  if (currentPlayer == STATE_X) {
    // Player X's turn - notify X player controller
    playerXSerial.write('t');
  } else {
    // Player O's turn - notify O player controller
    playerOSerial.write('t');
  }
}

/**
 * Send haptic feedback for game end (win/lose)
 */
void sendGameEndHaptics(ServoState winner) {
  if (winner == STATE_X) {
    // Player X wins - send 'w' to X controller, 'l' to O controller
    playerXSerial.write('w');  // Win pattern (....--)
    playerOSerial.write('l');  // Lose pattern (----..)
  } else {
    // Player O wins - send 'l' to X controller, 'w' to O controller
    playerXSerial.write('l');  // Lose pattern (----..)
    playerOSerial.write('w');  // Win pattern (....--)
  }
}

/**
 * Start cursor wiggle animation
 */
void startCursorWiggle() {
  isWiggling = true;
  wiggleStep = 0;
  lastWiggleTime = millis();
}

/**
 * Update cursor wiggle animation
 */
void updateCursorWiggle() {
  if (!isWiggling) return;
  
  unsigned long currentTime = millis();
  int servoIndex = cursorRow * GRID_SIZE + cursorCol;
  
  // Different wiggle patterns for occupied vs empty positions
  if (gridState[servoIndex] == STATE_NONE) {
    // Empty position - longer, centered on NONE state
    updateEmptyWiggle(servoIndex, currentTime);
  } else {
    // Occupied position - quicker, centered on existing piece
    updateOccupiedWiggle(servoIndex, currentTime);
  }
}

/**
 * Update wiggle for empty position
 */
void updateEmptyWiggle(int servoIndex, unsigned long currentTime) {
  const unsigned long WIGGLE_INTERVAL = 300; // Longer interval for empty
  const int WIGGLE_AMPLITUDE = 50; // Larger amplitude
  
  if (currentTime - lastWiggleTime >= WIGGLE_INTERVAL) {
    int basePosition = servoPositions[STATE_NONE];
    int offset = (wiggleStep % 4 < 2) ? WIGGLE_AMPLITUDE : -WIGGLE_AMPLITUDE;
    
    setServoPosition(servoIndex, basePosition + offset);
    
    wiggleStep++;
    lastWiggleTime = currentTime;
    
    // Stop after 8 steps (2 full cycles)
    if (wiggleStep >= 8) {
      isWiggling = false;
      setServoPosition(servoIndex, basePosition);
    }
  }
}

/**
 * Update wiggle for occupied position
 */
void updateOccupiedWiggle(int servoIndex, unsigned long currentTime) {
  const unsigned long WIGGLE_INTERVAL = 150; // Shorter interval for occupied
  const int WIGGLE_AMPLITUDE = 25; // Smaller amplitude
  
  if (currentTime - lastWiggleTime >= WIGGLE_INTERVAL) {
    int basePosition = servoPositions[gridState[servoIndex]];
    int offset = (wiggleStep % 4 < 2) ? WIGGLE_AMPLITUDE : -WIGGLE_AMPLITUDE;
    
    setServoPosition(servoIndex, basePosition + offset);
    
    wiggleStep++;
    lastWiggleTime = currentTime;
    
    // Stop after 6 steps (1.5 cycles)
    if (wiggleStep >= 6) {
      isWiggling = false;
      setServoPosition(servoIndex, basePosition);
    }
  }
}

/**
 * Move cursor to next empty position
 */
void moveToNextEmpty() {
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    if (gridState[i] == STATE_NONE) {
      cursorRow = i / GRID_SIZE;
      cursorCol = i % GRID_SIZE;
      startCursorWiggle();
      return;
    }
  }
  // Board is full - game is a draw
  gameActive = false;
  Serial.println("Game is a draw!");
}

/**
 * Check for win condition (5 in a row)
 */
bool checkWin(int row, int col, ServoState player) {
  // Check horizontal
  if (countInDirection(row, col, 0, 1, player) >= 5) return true;
  
  // Check vertical
  if (countInDirection(row, col, 1, 0, player) >= 5) return true;
  
  // Check diagonal \
  if (countInDirection(row, col, 1, 1, player) >= 5) return true;
  
  // Check diagonal /
  if (countInDirection(row, col, 1, -1, player) >= 5) return true;
  
  return false;
}

/**
 * Count consecutive pieces in a direction
 */
int countInDirection(int startRow, int startCol, int deltaRow, int deltaCol, ServoState player) {
  int count = 1; // Count the starting piece
  
  // Count in positive direction
  int row = startRow + deltaRow;
  int col = startCol + deltaCol;
  while (row >= 0 && row < GRID_SIZE && col >= 0 && col < GRID_SIZE && 
         gridState[row * GRID_SIZE + col] == player) {
    count++;
    row += deltaRow;
    col += deltaCol;
  }
  
  // Count in negative direction
  row = startRow - deltaRow;
  col = startCol - deltaCol;
  while (row >= 0 && row < GRID_SIZE && col >= 0 && col < GRID_SIZE && 
         gridState[row * GRID_SIZE + col] == player) {
    count++;
    row -= deltaRow;
    col -= deltaCol;
  }
  
  return count;
}

/**
 * Start the game
 */
void startGame() {
  if (gameStarted) return;
  
  Serial.println("Starting Gomoku game!");
  
  // Clear the board
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gridState[i] = STATE_NONE;
    setServoPosition(i, servoPositions[STATE_NONE]);
  }
  
  // Set game state
  gameActive = true;
  gameStarted = true;
  cursorRow = 0;
  cursorCol = 0;
  currentPlayer = STATE_X;
  
  delay(500); // Brief pause
  
  // Start cursor wiggle at top-left
  startCursorWiggle();
  
  // Notify first player (X) that it's their turn
  sendTurnNotification();
  
  Serial.println("Player X starts at position (0,0)");
}

/**
 * Update checkerboard pattern for inactive game
 */
void updateCheckerboardPattern() {
  unsigned long currentTime = millis();
  
  // Change pattern every 7 seconds (randomized between 5-10 seconds)
  if (currentTime - lastPatternTime >= 7000) {
    checkerboardState = !checkerboardState;
    lastPatternTime = currentTime;
    
    // Apply checkerboard pattern
    for (int row = 0; row < GRID_SIZE; row++) {
      for (int col = 0; col < GRID_SIZE; col++) {
        int servoIndex = row * GRID_SIZE + col;
        
        // Checkerboard pattern: alternate between X and O
        bool shouldBeX = ((row + col) % 2 == 0);
        
        if (checkerboardState) {
          // Invert the pattern
          shouldBeX = !shouldBeX;
        }
        
        ServoState state = shouldBeX ? STATE_X : STATE_O;
        gridState[servoIndex] = state;
        setServoPosition(servoIndex, servoPositions[state]);
      }
    }
    
    // In simulation mode, print the board after pattern change
    #if SIMULATION_MODE
      printSimulationBoard();
    #endif
  }
}

/**
 * Start win sequence animation
 */
void startWinSequence(int winRow, int winCol, ServoState winner) {
  Serial.print("Player ");
  Serial.print(winner == STATE_X ? "X" : "O");
  Serial.println(" wins! Starting win sequence...");
  
  winSequenceActive = true;
  winSequenceStartTime = millis();
  winSequenceStep = 0;
  lastMoveRow = winRow;
  lastMoveCol = winCol;
  winningPlayer = winner;
  
  // Stop cursor wiggle
  isWiggling = false;
  
  // Send haptic feedback for win/lose
  sendGameEndHaptics(winner);
}

/**
 * Update win sequence animation
 */
void updateWinSequence() {
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - winSequenceStartTime;
  
  // Animation timing: 200ms per step
  if (elapsed >= (winSequenceStep + 1) * 200) {
    winSequenceStep++;
    
    if (winSequenceStep <= 25) { // 25 steps to cover all positions
      animateWinStep();
    } else if (elapsed >= 5000) { // 5 seconds total
      // End win sequence and reset
      endWinSequence();
    }
  }
}

/**
 * Animate one step of the win sequence
 */
void animateWinStep() {
  // Calculate which positions to animate in this step
  // Start from the winning move and expand outward
  
  for (int row = 0; row < GRID_SIZE; row++) {
    for (int col = 0; col < GRID_SIZE; col++) {
      int servoIndex = row * GRID_SIZE + col;
      
      // Calculate distance from winning move
      int distance = max(abs(row - lastMoveRow), abs(col - lastMoveCol));
      
      // Animate positions at current step distance
      if (distance == winSequenceStep - 1) {
        // Move this position to winning player's state
        gridState[servoIndex] = winningPlayer;
        setServoPosition(servoIndex, servoPositions[winningPlayer]);
      }
    }
  }
  
  // In simulation mode, print the board after each animation step
  #if SIMULATION_MODE
    printSimulationBoard();
  #endif
}

/**
 * End win sequence and reset game
 */
void endWinSequence() {
  Serial.println("Win sequence complete. Resetting game...");
  
  // Reset win sequence state
  winSequenceActive = false;
  winningPlayer = STATE_NONE;
  
  // Reset game to initial state
  initializeGame();
}

/**
 * Reset game to initial state
 */
void resetGame() {
  Serial.println("Resetting game...");
  initializeGame();
}
