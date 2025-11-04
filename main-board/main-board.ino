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
 * 
 * Servo Connection Ordering:
 * - First PCA9685 (0x40, A0 bridged):
 *   Channels 0-14 connect to servo indices 0-14
 *   Grid mapping: Servo index = row * 5 + col
 *   Physical channels: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
 *   
 * - Second PCA9685 (0x41, all address pins grounded):
 *   Channels 0-9 connect to servo indices 15-24
 *   Physical channels: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 (map to grid indices 15-24)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

// Create PCA9685 objects
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First board (A0 bridged)
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x60); // Second board (all grounded)

// AltSoftSerial for communication with X player controller (more reliable)
AltSoftSerial playerXSerial; // Uses pins 8/9 (RX/TX)
// SoftwareSerial for communication with O player controller
SoftwareSerial playerOSerial(5, 6); // RX, TX for O player controller

// Button pins
#define SELECT_BUTTON_PIN 7
#define RESET_START_BUTTON_PIN 8

// Servo configuration
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVO_MIN 90 // Minimum pulse length count (out of 4096)
#define SERVO_MAX 525 // Maximum pulse length count (out of 4096)

// Grid dimensions
#define GRID_SIZE 5
#define TOTAL_SERVOS 25

// Win condition - number of pieces in a row needed to win
#define WIN_CONDITION 3

// Maximum number of servos that can move simultaneously (adjustable)
#define MAX_CONCURRENT_SERVO_MOVEMENTS 10

// Movement tracking duration (milliseconds a servo is considered "moving")
#define MOVEMENT_DURATION 300

// Simulation mode - set to 1 to enable simulation (no servo movement, prints board state)
#define SIMULATION_MODE 0

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

// Movement tracking structures (optimized for memory)
struct Movement {
  uint8_t servoIndex;      // 0-24, so uint8_t is sufficient
  uint16_t position;       // Pulse width (0-4095), uint16_t is sufficient
  unsigned long startTime;
  bool active;
};

Movement activeMovements[MAX_CONCURRENT_SERVO_MOVEMENTS];
struct PendingMovement {
  uint8_t servoIndex;
  uint16_t position;
  unsigned long scheduledStartTime;  // When this movement should actually start
};

// Reduced queue size to save memory (15 items = ~45 bytes instead of ~400 bytes)
#define MOVEMENT_QUEUE_SIZE 25
PendingMovement movementQueue[MOVEMENT_QUEUE_SIZE];
uint8_t queueHead = 0;
uint8_t queueTail = 0;
uint8_t queueSize = 0;
unsigned long lastMovementStartTime = 0;  // Track when last movement started for staggering

// Servo position mapping for each state
const int servoPositions[3] = {
  300,  // STATE_NONE
  470,                    // STATE_X
  100                    // STATE_O
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
  
  // Initialize movement tracking
  for (int i = 0; i < MAX_CONCURRENT_SERVO_MOVEMENTS; i++) {
    activeMovements[i].active = false;
  }
  queueHead = 0;
  queueTail = 0;
  queueSize = 0;
  
  // Seed random number generator for random queue processing
  randomSeed(analogRead(0));
  
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
  Serial.println("Commands: 'display' to show board, 'testwin' to test win detection");
}

void loop() {
  // Process movement queue to allow queued movements to start when slots become available
  processMovementQueue();
  
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
    processMovementQueue(); // Also process queue during win sequence
    delay(100);
    return;
  }
  
  // Check for wins at the start of each turn (in case a win was missed)
  checkForWins();
  
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
 * Count number of currently active movements
 */
int countActiveMovements() {
  int count = 0;
  unsigned long currentTime = millis();
  
  for (int i = 0; i < MAX_CONCURRENT_SERVO_MOVEMENTS; i++) {
    if (activeMovements[i].active) {
      // Check if movement has expired
      if (currentTime - activeMovements[i].startTime >= MOVEMENT_DURATION) {
        activeMovements[i].active = false;
      } else {
        count++;
      }
    }
  }
  
  return count;
}

/**
 * Find an available slot for a new movement
 */
int findAvailableMovementSlot() {
  unsigned long currentTime = millis();
  
  // First, clean up expired movements
  for (int i = 0; i < MAX_CONCURRENT_SERVO_MOVEMENTS; i++) {
    if (activeMovements[i].active) {
      if (currentTime - activeMovements[i].startTime >= MOVEMENT_DURATION) {
        activeMovements[i].active = false;
      }
    }
  }
  
  // Find an empty slot
  for (int i = 0; i < MAX_CONCURRENT_SERVO_MOVEMENTS; i++) {
    if (!activeMovements[i].active) {
      return i;
    }
  }
  
  return -1; // No available slot
}

/**
 * Add a movement to the active movements list
 */
void addActiveMovement(uint8_t servoIndex, uint16_t position) {
  int slot = findAvailableMovementSlot();
  if (slot >= 0) {
    activeMovements[slot].servoIndex = servoIndex;
    activeMovements[slot].position = position;
    activeMovements[slot].startTime = millis();
    activeMovements[slot].active = true;
  }
}

/**
 * Add a movement to the queue with staggered start time
 */
void enqueueMovement(uint8_t servoIndex, uint16_t position) {
  if (queueSize < MOVEMENT_QUEUE_SIZE) {
    movementQueue[queueTail].servoIndex = servoIndex;
    movementQueue[queueTail].position = position;
    
    // Schedule start time with random delay (10-100ms)
    // For movements queued at the same time, use the last scheduled time to accumulate delays
    unsigned long currentTime = millis();
    unsigned long staggerDelay = random(10, 101);  // 10-100ms random delay
    
    // If this is the first movement or we're starting fresh, use current time
    // Otherwise, accumulate delay from the last scheduled time
    if (queueSize == 0 && lastMovementStartTime == 0) {
      // First movement - schedule immediately with small delay
      movementQueue[queueTail].scheduledStartTime = currentTime + staggerDelay;
    } else {
      // Accumulate delay from the last scheduled time in the queue
      // Find the latest scheduled time in the queue
      unsigned long latestScheduledTime = currentTime;
      for (uint8_t i = 0; i < queueSize; i++) {
        uint8_t queuePos = (queueHead + i) % MOVEMENT_QUEUE_SIZE;
        if (movementQueue[queuePos].scheduledStartTime > latestScheduledTime) {
          latestScheduledTime = movementQueue[queuePos].scheduledStartTime;
        }
      }
      // Schedule after the latest scheduled time plus stagger delay
      movementQueue[queueTail].scheduledStartTime = latestScheduledTime + staggerDelay;
    }
    
    queueTail = (queueTail + 1) % MOVEMENT_QUEUE_SIZE;
    queueSize++;
  }
}

/**
 * Process queued movements when slots become available (in random order, with staggered timing)
 */
void processMovementQueue() {
  unsigned long currentTime = millis();
  int attempts = 0;  // Prevent infinite loop
  const int maxAttempts = queueSize;  // Try at most once per queued item
  
  while (queueSize > 0 && attempts < maxAttempts) {
    attempts++;
    
    int slot = findAvailableMovementSlot();
    if (slot < 0) {
      break; // No available slots
    }
    
    // Collect all ready movements first (scheduled time has passed)
    uint8_t readyIndices[MOVEMENT_QUEUE_SIZE];
    uint8_t readyCount = 0;
    
    for (uint8_t i = 0; i < queueSize; i++) {
      uint8_t queuePos = (queueHead + i) % MOVEMENT_QUEUE_SIZE;
      if (movementQueue[queuePos].scheduledStartTime <= currentTime) {
        readyIndices[readyCount] = i;
        readyCount++;
      }
    }
    
    // If no ready movement found, exit (will try again next loop iteration)
    if (readyCount == 0) {
      break;
    }
    
    // Randomly select one from all ready movements
    uint8_t selectedIndex = readyIndices[random(0, readyCount)];
    uint8_t selectedPos = (queueHead + selectedIndex) % MOVEMENT_QUEUE_SIZE;
    
    // Swap the randomly selected ready movement with the head item
    PendingMovement movement = movementQueue[selectedPos];
    movementQueue[selectedPos] = movementQueue[queueHead];
    movementQueue[queueHead] = movement;
    
    // Remove from queue
    queueHead = (queueHead + 1) % MOVEMENT_QUEUE_SIZE;
    queueSize--;
    
    // Update last movement start time for next stagger calculation
    lastMovementStartTime = currentTime;
    
    // Execute the movement
    if (movement.servoIndex < 15) {
      pwm1.setPWM(movement.servoIndex, 0, movement.position);
    } else {
      uint8_t channel = movement.servoIndex - 15;
      pwm2.setPWM(channel, 0, movement.position);
    }
    
    // Add to active movements
    addActiveMovement(movement.servoIndex, movement.position);
  }
}

/**
 * Set a specific servo position (with movement limiting)
 * @param servoIndex - Servo index (0-24)
 * @param position - Pulse length (150-600)
 */
void setServoPosition(int servoIndex, int position) {
  if (servoIndex < 0 || servoIndex >= TOTAL_SERVOS) {
    return;
  }
  
  #if SIMULATION_MODE
    // In simulation mode, just print the board state instead of moving servos
    printBoard();
    return;
  #else
    // Always queue movements - the queue processing will handle randomization and staggering
    // This ensures all movements go through the same randomized selection process
    enqueueMovement(servoIndex, position);
    
    // Process the queue to execute movements when slots are available
    processMovementQueue();
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
 * Print board state (- for empty, x for X, o for O)
 */
void printBoard() {
  Serial.println("\n=== BOARD STATE ===");
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
  Serial.println("==================\n");
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
    // Reset old cursor position to its base/idle position
    int oldServoIndex = cursorRow * GRID_SIZE + cursorCol;
    int oldBasePosition = servoPositions[gridState[oldServoIndex]];
    setServoPosition(oldServoIndex, oldBasePosition);
    
    // Update cursor position
    cursorRow = newRow;
    cursorCol = newCol;
    
    // Reset new cursor position to its base/idle position before starting wiggle
    int newServoIndex = cursorRow * GRID_SIZE + cursorCol;
    int newBasePosition = servoPositions[gridState[newServoIndex]];
    setServoPosition(newServoIndex, newBasePosition);
    
    // Start wiggle animation at new position
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
    // Valid move - capture current player before switching
    ServoState movePlayer = currentPlayer;
    setGridState(cursorRow, cursorCol, currentPlayer);
    sendFeedback('y', movePlayer); // Yes - move is valid, send to player who made the move
    
    // Print board state after move
    printBoard();
    
    // Check for win anywhere on the board
    if (checkBoardForWin(currentPlayer)) {
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
    sendFeedback('n', currentPlayer); // No - move is invalid, send to current player
  }
}

/**
 * Send feedback to the active controller only
 */
void sendFeedback(char response, ServoState player) {
  if (player == STATE_X) {
    // Send to X player controller only
    playerXSerial.write(response);
  } else {
    // Send to O player controller only
    playerOSerial.write(response);
  }
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
    delay(10);
    playerOSerial.write('l');  // Lose pattern (----..)
  } else {
    // Player O wins - send 'l' to X controller, 'w' to O controller
    playerXSerial.write('l');  // Lose pattern (----..)
    delay(10);
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
    
    // Animation continues forever - no stopping condition
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
    
    // Animation continues forever - no stopping condition
  }
}

/**
 * Move cursor to next empty position
 */
void moveToNextEmpty() {
  // Reset old cursor position to its base/idle position
  int oldServoIndex = cursorRow * GRID_SIZE + cursorCol;
  int oldBasePosition = servoPositions[gridState[oldServoIndex]];
  setServoPosition(oldServoIndex, oldBasePosition);
  
  // Always move cursor to position (0,0) regardless of availability
  cursorRow = 0;
  cursorCol = 0;
  
  // Reset new cursor position to its base/idle position before starting wiggle
  int newServoIndex = cursorRow * GRID_SIZE + cursorCol;
  int newBasePosition = servoPositions[gridState[newServoIndex]];
  setServoPosition(newServoIndex, newBasePosition);
  
  startCursorWiggle();
  
  // Check if board is full (all positions taken)
  bool boardFull = true;
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    if (gridState[i] == STATE_NONE) {
      boardFull = false;
      break;
    }
  }
  
  if (boardFull) {
    // Board is full - game is a draw
    gameActive = false;
    Serial.println("Game is a draw!");
  }
}

/**
 * Check for win condition (WIN_CONDITION in a row) from a specific position
 */
bool checkWin(int row, int col, ServoState player) {
  // Check horizontal
  if (countInDirection(row, col, 0, 1, player) >= WIN_CONDITION) return true;
  
  // Check vertical
  if (countInDirection(row, col, 1, 0, player) >= WIN_CONDITION) return true;
  
  // Check diagonal
  if (countInDirection(row, col, 1, 1, player) >= WIN_CONDITION) return true;
  
  // Check diagonal
  if (countInDirection(row, col, 1, -1, player) >= WIN_CONDITION) return true;
  
  return false;
}

/**
 * Check for win condition anywhere on the board
 */
bool checkBoardForWin(ServoState player) {
  // Check every position on the board for a win
  for (int row = 0; row < GRID_SIZE; row++) {
    for (int col = 0; col < GRID_SIZE; col++) {
      if (gridState[row * GRID_SIZE + col] == player) {
        if (checkWin(row, col, player)) {
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * Check for wins for both players and handle if found
 */
void checkForWins() {
  // Check for X wins
  if (checkBoardForWin(STATE_X)) {
    startWinSequence(cursorRow, cursorCol, STATE_X);
    return;
  }
  
  // Check for O wins
  if (checkBoardForWin(STATE_O)) {
    startWinSequence(cursorRow, cursorCol, STATE_O);
    return;
  }
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
  
  // Check for any existing wins before starting
  checkForWins();
  
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
    
    // Print the board after pattern change
    printBoard();
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
  
  // Print the board after each animation step
  printBoard();
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
