/*
 * 5x5 Servo Grid Controller using 2 PCA9685 Boards
 * 
 * Board Configuration:
 * - First PCA9685: Address 0x40, physical channels 6-15, controls grid servo indices 0-9 (first 10 squares)
 * - Second PCA9685: Address 0x60, physical channels 1-15, controls grid servo indices 10-24 (next 15 squares)
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
 * - First PCA9685 (0x40):
 *   Physical channels 6-15 connect to grid servo indices 0-9
 *   Grid mapping: Servo index = row * 5 + col
 *   Physical channel = servoIndex + 6 (e.g., servoIndex 0 → channel 6, servoIndex 9 → channel 15)
 *   
 * - Second PCA9685 (0x60):
 *   Physical channels 1-15 connect to grid servo indices 10-24
 *   Physical channel = servoIndex - 9 (e.g., servoIndex 10 → channel 1, servoIndex 24 → channel 15)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

// Create PCA9685 objects
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // First board (A0 bridged)
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x60); // Second board (all grounded)

// AltSoftSerial for communication with X player controller (more reliable)
AltSoftSerial playerXSerial; // Uses pins 8/9 (RX green/TX yellow)
// SoftwareSerial for communication with O player controller
SoftwareSerial playerOSerial(5, 6); // RX green, TX yellow for O player controller

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

// Win condition - number of pieces in a row needed to win (will be set based on gamemode)
uint8_t winCondition = 4;

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

// Game mode states
enum GameModeState {
  STATE_IDLE = 0,
  STATE_GAME_SELECT = 1,
  STATE_ACTIVE = 2
};

// Game mode types
enum GameType {
  GAME_GOMOKU = 0,
  GAME_CONNECT4 = 1
};

// Grid state array - tracks current state of each servo
ServoState gridState[TOTAL_SERVOS];

// Game state variables
GameModeState gameModeState = STATE_IDLE;
uint8_t currentGameMode = 0; // 0-2: Gomoku 3-5, 3-5: Connect4 3-5
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

// Gamemode display variables
uint8_t lastDisplayedGameMode = 255; // Track last displayed gamemode (255 = not set)

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

// Flag to use immediate movements (no delays) for gamemode display
bool useImmediateMovements = false;

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
  Serial.println(F("Gomoku Game Starting..."));
  
  // Initialize AltSoftSerial for X player controller
  playerXSerial.begin(9600);
  // Initialize SoftwareSerial for O player controller
  playerOSerial.begin(9600);
  
  // Initialize button pins
  pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_START_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize PCA9685 boards (only if not in simulation mode)
  #if SIMULATION_MODE
    Serial.println(F("SIMULATION MODE ENABLED - Servos will not move, board state will be printed to serial"));
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
  
  Serial.println(F("Gomoku game initialized!"));
  #if SIMULATION_MODE
    Serial.println(F("SIMULATION MODE: Board state will be printed instead of moving servos"));
  #else
    Serial.println(F("Press SELECT to choose gamemode, then SELECT again to start"));
  #endif
  Serial.println(F("Controls:"));
  Serial.println(F("  - X/O controllers: l/r to change gamemode, s to enter/start"));
  Serial.println(F("  - SELECT button: Enter gamemode select or start game"));
}

void loop() {
  // Process movement queue to allow queued movements to start when slots become available
  processMovementQueue();
  
  if (gameModeState == STATE_IDLE) {
    // Show checkerboard pattern when idle
    updateCheckerboardPattern();
    
    // Check for controller and button input
    checkControllerInput();
    checkButtonInput();
    
    // Check for serial commands (for debugging)
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      
      if (command == "start") {
        enterGameSelect();
      } else if (command == "display") {
        displayGameState();
      }
    }
    
    delay(50);
    return;
  }
  
  if (gameModeState == STATE_GAME_SELECT) {
    // Show gamemode pattern on board (similar to checkerboard pattern)
    updateGamemodeDisplay();
    
    // Check for controller and button input
    checkControllerInput();
    checkButtonInput();
    
    delay(50);
    return;
  }
  
  // STATE_ACTIVE - game is running
  if (!gameActive) {
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
  gameModeState = STATE_IDLE;
  currentGameMode = 0;
  cursorRow = 0;
  cursorCol = 0;
  currentPlayer = STATE_X;
  gameActive = false; // Start inactive
  gameStarted = false;
  winSequenceActive = false;
  isWiggling = false;
  checkerboardState = false;
  lastPatternTime = millis();
  winningPlayer = STATE_NONE;
  winCondition = 4;
  
  delay(1000); // Allow servos to reach position
  
  // Start with checkerboard pattern
  updateCheckerboardPattern();
}

/**
 * Enter game select mode
 */
void enterGameSelect() {
  gameModeState = STATE_GAME_SELECT;
  currentGameMode = 0; // Start at first gamemode
  gameStarted = false; // Reset game started flag
  lastDisplayedGameMode = 255; // Force update on next loop
}

/**
 * Update gamemode display on board
 * Shows pattern of squares in a row based on gamemode
 * Similar approach to checkerboard pattern - continuously applies the pattern
 */
void updateGamemodeDisplay() {
  // Only update if gamemode changed
  if (lastDisplayedGameMode == currentGameMode) {
    return; // Already displaying correct pattern
  }
  
  lastDisplayedGameMode = currentGameMode;
  
  uint8_t squaresInRow;
  bool isConnect4 = (currentGameMode >= 3);
  
  if (isConnect4) {
    // Connect 4 modes (3-5): map to 3-5 in a row
    squaresInRow = currentGameMode - 3 + 3; // 3, 4, or 5
  } else {
    // Gomoku modes (0-2): 3-5 in a row
    squaresInRow = currentGameMode + 3; // 3, 4, or 5
  }
  
  // Apply pattern to all servos (similar to checkerboard pattern)
  for (int row = 0; row < GRID_SIZE; row++) {
    for (int col = 0; col < GRID_SIZE; col++) {
      int servoIndex = row * GRID_SIZE + col;
      ServoState targetState = STATE_NONE;
      
      if (isConnect4) {
        // Connect 4: show pattern on bottom row
        uint8_t startCol = (GRID_SIZE - squaresInRow) / 2;
        if (row == GRID_SIZE - 1 && col >= startCol && col < startCol + squaresInRow) {
          targetState = STATE_X;
        }
      } else {
        // Gomoku: show pattern on diagonal
        uint8_t startRow = 0;
        uint8_t startCol = 0;
        if (squaresInRow == 3) {
          startRow = 1;
          startCol = 1;
        }
        
        if (row >= startRow && row < startRow + squaresInRow && 
            col >= startCol && col < startCol + squaresInRow && 
            row == col) {
          targetState = STATE_X;
        }
      }
      
      // Update if state changed
      if (gridState[servoIndex] != targetState) {
        gridState[servoIndex] = targetState;
        setServoPosition(servoIndex, servoPositions[targetState]);
      }
    }
  }
}

/**
 * Refresh gamemode display to maintain consistency
 * Only updates servos that are not in the correct position
 */
void refreshGamemodeDisplay() {
  uint8_t squaresInRow = currentGameMode + 3;
  bool isConnect4 = (currentGameMode >= 3);
  
  // Use immediate movements for refresh
  useImmediateMovements = true;
  
  if (isConnect4) {
    // Connect 4: show pattern on bottom row
    uint8_t startCol = (GRID_SIZE - squaresInRow) / 2;
    for (uint8_t i = 0; i < squaresInRow; i++) {
      int col = startCol + i;
      if (col < GRID_SIZE) {
        int servoIndex = (GRID_SIZE - 1) * GRID_SIZE + col;
        // Only update if state doesn't match
        if (gridState[servoIndex] != STATE_X) {
          gridState[servoIndex] = STATE_X;
          setServoPosition(servoIndex, servoPositions[STATE_X]);
        }
      }
    }
    // Clear other positions
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      int row = i / GRID_SIZE;
      int col = i % GRID_SIZE;
      bool shouldBeX = (row == GRID_SIZE - 1 && col >= (GRID_SIZE - squaresInRow) / 2 && 
                        col < (GRID_SIZE - squaresInRow) / 2 + squaresInRow);
      if (!shouldBeX && gridState[i] != STATE_NONE) {
        gridState[i] = STATE_NONE;
        setServoPosition(i, servoPositions[STATE_NONE]);
      }
    }
  } else {
    // Gomoku: show pattern on diagonal
    uint8_t startRow = 0;
    uint8_t startCol = 0;
    if (squaresInRow == 3) {
      startRow = 1;
      startCol = 1;
    }
    
    // Update pattern positions
    for (uint8_t i = 0; i < squaresInRow; i++) {
      int row = startRow + i;
      int col = startCol + i;
      if (row < GRID_SIZE && col < GRID_SIZE) {
        int servoIndex = row * GRID_SIZE + col;
        if (gridState[servoIndex] != STATE_X) {
          gridState[servoIndex] = STATE_X;
          setServoPosition(servoIndex, servoPositions[STATE_X]);
        }
      }
    }
    // Clear other positions
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      int row = i / GRID_SIZE;
      int col = i % GRID_SIZE;
      bool shouldBeX = false;
      if (squaresInRow == 3) {
        shouldBeX = (row >= 1 && row < 4 && col >= 1 && col < 4 && row == col);
      } else {
        shouldBeX = (row == col && row < squaresInRow);
      }
      if (!shouldBeX && gridState[i] != STATE_NONE) {
        gridState[i] = STATE_NONE;
        setServoPosition(i, servoPositions[STATE_NONE]);
      }
    }
  }
  
  useImmediateMovements = false;
  processMovementQueue();
}

/**
 * Set a specific servo in the grid to a given state
 * @param row - Row index (0-4)
 * @param col - Column index (0-4)
 * @param state - Servo state (STATE_NONE, STATE_X, STATE_O)
 */
void setGridState(int row, int col, ServoState state) {
  if (row < 0 || row >= GRID_SIZE || col < 0 || col >= GRID_SIZE) {
    Serial.println(F("Error: Grid coordinates out of range"));
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
    
    unsigned long currentTime = millis();
    unsigned long staggerDelay;
    
    if (useImmediateMovements) {
      // For gamemode display, use very short delays (0-5ms) for immediate feedback
      staggerDelay = random(0, 6);
    } else {
      // Normal game play: use random delay (10-100ms)
      staggerDelay = random(10, 101);
    }
    
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
 * Clear the movement queue completely
 */
void clearMovementQueue() {
  queueHead = 0;
  queueTail = 0;
  queueSize = 0;
  lastMovementStartTime = 0;
  
  // Clear all active movements
  for (int i = 0; i < MAX_CONCURRENT_SERVO_MOVEMENTS; i++) {
    activeMovements[i].active = false;
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
    // Convert logical grid index to physical servo index (vertically mirrored)
    uint8_t physicalIndex = getPhysicalServoIndex(movement.servoIndex);
    
    // First PCA9685 (0x40): handles physical servo indices 0-9 → physical channels 6-15
    // Second PCA9685 (0x60): handles physical servo indices 10-24 → physical channels 1-15
    if (physicalIndex >= 0 && physicalIndex <= 9) {
      // First board: physical indices 0-9 map to channels 6-15
      // Physical index 0 → channel 6, physical index 1 → channel 7, ..., physical index 9 → channel 15
      uint8_t channel = physicalIndex + 6;
      pwm1.setPWM(channel, 0, movement.position);
    } else if (physicalIndex >= 10 && physicalIndex <= 24) {
      // Second board: physical indices 10-24 map to channels 1-15
      // Physical index 10 → channel 1, physical index 11 → channel 2, ..., physical index 24 → channel 15
      uint8_t channel = physicalIndex - 9;
      pwm2.setPWM(channel, 0, movement.position);
    }
    
    // Add to active movements
    addActiveMovement(movement.servoIndex, movement.position);
  }
}

/**
 * Convert logical grid index to physical servo index (vertically mirrored)
 * Mirrors along horizontal axis: row 0 ↔ row 4, row 1 ↔ row 3, row 2 stays same
 * @param logicalIndex - Logical grid index (0-24)
 * @return Physical servo index (0-24)
 */
uint8_t getPhysicalServoIndex(uint8_t logicalIndex) {
  uint8_t row = logicalIndex / GRID_SIZE;
  uint8_t col = logicalIndex % GRID_SIZE;
  uint8_t mirroredRow = GRID_SIZE - 1 - row; // Mirror vertically
  return mirroredRow * GRID_SIZE + col;
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
  Serial.println(F("\nCurrent Game State:"));
  Serial.println(F("  0 1 2 3 4"));
  
  for (int row = 0; row < GRID_SIZE; row++) {
    Serial.print(row);
    Serial.print(' ');
    for (int col = 0; col < GRID_SIZE; col++) {
      int servoIndex = row * GRID_SIZE + col;
      char piece = ' ';
      if (gridState[servoIndex] == STATE_X) piece = 'X';
      else if (gridState[servoIndex] == STATE_O) piece = 'O';
      
      if (row == cursorRow && col == cursorCol) {
        Serial.print('[');
        Serial.print(piece);
        Serial.print(']');
      } else {
        Serial.print(' ');
        Serial.print(piece);
        Serial.print(' ');
      }
    }
    Serial.println();
  }
  Serial.print(F("Current Player: "));
  Serial.println(currentPlayer == STATE_X ? F("X") : F("O"));
  Serial.print(F("Cursor: ("));
  Serial.print(cursorRow);
  Serial.print(',');
  Serial.print(cursorCol);
  Serial.println(F(")\n"));
}

/**
 * Print board state (- for empty, x for X, o for O)
 */
void printBoard() {
  Serial.println(F("\n=== BOARD STATE ==="));
  for (int row = 0; row < GRID_SIZE; row++) {
    for (int col = 0; col < GRID_SIZE; col++) {
      int servoIndex = row * GRID_SIZE + col;
      char piece = '-';
      if (gridState[servoIndex] == STATE_X) piece = 'x';
      else if (gridState[servoIndex] == STATE_O) piece = 'o';
      
      Serial.print(piece);
      if (col < GRID_SIZE - 1) Serial.print(' ');
    }
    Serial.println();
  }
  Serial.print(F("Current Player: "));
  Serial.println(currentPlayer == STATE_X ? F("X") : F("O"));
  Serial.print(F("Cursor: ("));
  Serial.print(cursorRow);
  Serial.print(',');
  Serial.print(cursorCol);
  Serial.println(F(")"));
  Serial.println(F("==================\n"));
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
  if (gameModeState == STATE_IDLE) {
    // First press: enter game select mode
    enterGameSelect();
  } else if (gameModeState == STATE_GAME_SELECT) {
    // Second press: start the game
    startGame();
  } else if (gameModeState == STATE_ACTIVE && gameActive) {
    // During active game, select button makes a move
    makeMove();
  }
}

/**
 * Handle reset/start button press
 */
void handleResetStartButton() {
  // Reset button always goes back to idle
  resetGame();
}

/**
 * Process game input commands
 */
void processGameInput(char command, ServoState player) {
  if (gameModeState == STATE_IDLE) {
    // Handle select to enter game select mode
    if (command == 's') {
      enterGameSelect();
    }
    return;
  }
  
  if (gameModeState == STATE_GAME_SELECT) {
    // Handle gamemode selection
    if (command == 'l' || command == 'x' || command == 'o') {
      // Left/X/O buttons: decrease gamemode
      if (currentGameMode > 0) {
        currentGameMode--;
        lastDisplayedGameMode = 255; // Force update
      }
    } else if (command == 'r') {
      // Right button: increase gamemode
      if (currentGameMode < 5) {
        currentGameMode++;
        lastDisplayedGameMode = 255; // Force update
      }
    } else if (command == 's') {
      // Select: start game
      startGame();
    }
    return;
  }
  
  // STATE_ACTIVE - game is running
  if (!gameActive) {
    return;
  }
  
  // Only process input from the current player
  if (player != currentPlayer) {
    return;
  }
  
  // Check if Connect 4 mode
  bool isConnect4 = (currentGameMode >= 3);
  
  switch (command) {
    case 'u': // Up (only for Gomoku)
      if (!isConnect4) {
        moveCursor(-1, 0);
      }
      break;
    case 'd': // Down (only for Gomoku)
      if (!isConnect4) {
        moveCursor(1, 0);
      }
      break;
    case 'l': // Left
      if (isConnect4) {
        // Connect 4: move cursor left in top row
        if (cursorCol > 0) {
          moveCursor(0, -1);
        }
      } else {
        moveCursor(0, -1);
      }
      break;
    case 'r': // Right
      if (isConnect4) {
        // Connect 4: move cursor right in top row
        if (cursorCol < GRID_SIZE - 1) {
          moveCursor(0, 1);
        }
      } else {
        moveCursor(0, 1);
      }
      break;
    case 's': // Select
      makeMove();
      break;
  }
}

/**
 * Get effective board size based on gamemode
 */
uint8_t getBoardSize() {
  if (currentGameMode == 0 && winCondition == 3) {
    // 3 in a row gomoku: limit to 3x3
    return 3;
  }
  return GRID_SIZE;
}

/**
 * Check if a position is within valid board bounds
 */
bool isValidPosition(int row, int col) {
  uint8_t boardSize = getBoardSize();
  if (currentGameMode == 0 && winCondition == 3) {
    // 3x3 board: row/col 1-3 (indices 1-3)
    return row >= 1 && row < boardSize + 1 && col >= 1 && col < boardSize + 1;
  }
  return row >= 0 && row < boardSize && col >= 0 && col < boardSize;
}

/**
 * Move cursor to new position
 */
void moveCursor(int deltaRow, int deltaCol) {
  bool isConnect4 = (currentGameMode >= 3);
  int newRow = cursorRow + deltaRow;
  int newCol = cursorCol + deltaCol;
  
  if (isConnect4) {
    // Connect 4: cursor stays in top row (row 0)
    newRow = 0;
    // Check column bounds
    if (newCol >= 0 && newCol < GRID_SIZE) {
      int oldServoIndex = cursorRow * GRID_SIZE + cursorCol;
      int oldBasePosition = servoPositions[gridState[oldServoIndex]];
      setServoPosition(oldServoIndex, oldBasePosition);
      
      cursorRow = newRow;
      cursorCol = newCol;
      
      int newServoIndex = cursorRow * GRID_SIZE + cursorCol;
      int newBasePosition = servoPositions[gridState[newServoIndex]];
      setServoPosition(newServoIndex, newBasePosition);
      
      startCursorWiggle();
    }
  } else {
    // Gomoku: check bounds with board size limits
    if (isValidPosition(newRow, newCol)) {
      int oldServoIndex = cursorRow * GRID_SIZE + cursorCol;
      int oldBasePosition = servoPositions[gridState[oldServoIndex]];
      setServoPosition(oldServoIndex, oldBasePosition);
      
      cursorRow = newRow;
      cursorCol = newCol;
      
      int newServoIndex = cursorRow * GRID_SIZE + cursorCol;
      int newBasePosition = servoPositions[gridState[newServoIndex]];
      setServoPosition(newServoIndex, newBasePosition);
      
      startCursorWiggle();
    }
  }
}

/**
 * Make a move at current cursor position
 */
void makeMove() {
  bool isConnect4 = (currentGameMode >= 3);
  int targetRow = cursorRow;
  int targetCol = cursorCol;
  
  if (isConnect4) {
    // Connect 4: find lowest empty position in selected column
    targetRow = -1;
    for (int row = GRID_SIZE - 1; row >= 0; row--) {
      int idx = row * GRID_SIZE + cursorCol;
      if (gridState[idx] == STATE_NONE) {
        targetRow = row;
        break;
      }
    }
    
    // Check if column is full
    if (targetRow == -1) {
      sendFeedback('n', currentPlayer); // Column full
      return;
    }
  }
  
  int servoIndex = targetRow * GRID_SIZE + targetCol;
  
  // Check if position is empty
  if (gridState[servoIndex] == STATE_NONE) {
    // Valid move - capture current player before switching
    ServoState movePlayer = currentPlayer;
    
    // For Connect 4, animate the drop
    if (isConnect4) {
      // Enable immediate movements for animation to avoid timing issues
      useImmediateMovements = true;
      
      // Animate piece falling from top to target position
      for (int row = 0; row <= targetRow; row++) {
        int animIdx = row * GRID_SIZE + targetCol;
        if (row < targetRow) {
          // Show piece at this position
          gridState[animIdx] = currentPlayer;
          setServoPosition(animIdx, servoPositions[currentPlayer]);
          
          // Process queue multiple times to ensure movement starts
          for (int i = 0; i < 5; i++) {
            processMovementQueue();
            delay(10);
          }
          
          // Wait longer for piece to be visible (slower fall)
          delay(200);
          
          // Clear the position - ensure it's properly cleared
          gridState[animIdx] = STATE_NONE;
          setServoPosition(animIdx, servoPositions[STATE_NONE]);
          
          // Process queue multiple times to ensure clearing happens
          for (int i = 0; i < 5; i++) {
            processMovementQueue();
            delay(10);
          }
          
          // Wait for clearing to complete
          delay(100);
        } else {
          // Final position reached
          // First, ensure all intermediate positions are definitely cleared
          for (int clearRow = 0; clearRow < targetRow; clearRow++) {
            int clearIdx = clearRow * GRID_SIZE + targetCol;
            gridState[clearIdx] = STATE_NONE;
            setServoPosition(clearIdx, servoPositions[STATE_NONE]);
          }
          
          // Process queue multiple times to ensure all clears complete
          for (int i = 0; i < 10; i++) {
            processMovementQueue();
            delay(20);
          }
          
          // Now set final position immediately (no delay)
          setGridState(targetRow, targetCol, currentPlayer);
          
          // Process queue to ensure final position is set
          for (int i = 0; i < 5; i++) {
            processMovementQueue();
            delay(10);
          }
        }
      }
      
      // Disable immediate movements mode
      useImmediateMovements = false;
      
      // Final safety cleanup: double-check all intermediate positions are cleared
      for (int row = 0; row < targetRow; row++) {
        int idx = row * GRID_SIZE + targetCol;
        if (gridState[idx] != STATE_NONE) {
          gridState[idx] = STATE_NONE;
          setServoPosition(idx, servoPositions[STATE_NONE]);
        }
      }
      
      // Final queue processing to ensure cleanup completes
      for (int i = 0; i < 10; i++) {
        processMovementQueue();
        delay(20);
      }
      delay(100);
    } else {
      setGridState(targetRow, targetCol, currentPlayer);
    }
    
    sendFeedback('y', movePlayer); // Yes - move is valid
    
    // Print board state after move
    printBoard();
    
    // Check for win anywhere on the board
    if (checkBoardForWin(currentPlayer)) {
      startWinSequence(targetRow, targetCol, currentPlayer);
      return;
    }
    
    // Check if board is full with no winner (for all game modes)
    if (isBoardFull()) {
      Serial.println(F("Board is full with no winner. Cancelling game..."));
      resetGame();
      return;
    }
    
    // Switch players
    currentPlayer = (currentPlayer == STATE_X) ? STATE_O : STATE_X;
    
    // Reset cursor to correct zero position for this game type
    resetCursorToZeroPosition();
    
    // Notify the new player that it's their turn
    sendTurnNotification();
    
  } else {
    // Invalid move - position already taken
    sendFeedback('n', currentPlayer); // No - move is invalid
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
 * Move cursor to next empty position for Connect 4
 */
void moveToNextEmptyConnect4() {
  // Reset old cursor position
  int oldServoIndex = cursorRow * GRID_SIZE + cursorCol;
  int oldBasePosition = servoPositions[gridState[oldServoIndex]];
  setServoPosition(oldServoIndex, oldBasePosition);
  
  // Find next column with space
  for (int col = 0; col < GRID_SIZE; col++) {
    for (int row = GRID_SIZE - 1; row >= 0; row--) {
      int idx = row * GRID_SIZE + col;
      if (gridState[idx] == STATE_NONE) {
        cursorRow = 0;
        cursorCol = col;
        int newServoIndex = cursorRow * GRID_SIZE + cursorCol;
        int newBasePosition = servoPositions[gridState[newServoIndex]];
        setServoPosition(newServoIndex, newBasePosition);
        startCursorWiggle();
        return;
      }
    }
  }
  
  // Board is full - reset the game
  Serial.println(F("Board is full! Resetting game..."));
  resetGame();
}

/**
 * Move cursor to next empty position
 */
void moveToNextEmpty() {
  // Reset old cursor position
  int oldServoIndex = cursorRow * GRID_SIZE + cursorCol;
  int oldBasePosition = servoPositions[gridState[oldServoIndex]];
  setServoPosition(oldServoIndex, oldBasePosition);
  
  // Find first valid empty position
  uint8_t boardSize = getBoardSize();
  uint8_t startRow = (currentGameMode == 0 && winCondition == 3) ? 1 : 0;
  uint8_t startCol = (currentGameMode == 0 && winCondition == 3) ? 1 : 0;
  uint8_t endRow = (currentGameMode == 0 && winCondition == 3) ? boardSize + 1 : boardSize;
  uint8_t endCol = (currentGameMode == 0 && winCondition == 3) ? boardSize + 1 : boardSize;
  
  for (int row = startRow; row < endRow; row++) {
    for (int col = startCol; col < endCol; col++) {
      int idx = row * GRID_SIZE + col;
      if (gridState[idx] == STATE_NONE) {
        cursorRow = row;
        cursorCol = col;
        int newServoIndex = cursorRow * GRID_SIZE + cursorCol;
        int newBasePosition = servoPositions[gridState[newServoIndex]];
        setServoPosition(newServoIndex, newBasePosition);
        startCursorWiggle();
        return;
      }
    }
  }
  
  // Board is full - reset the game
  Serial.println(F("Board is full! Resetting game..."));
  resetGame();
}

/**
 * Check for win condition (winCondition in a row) from a specific position
 */
bool checkWin(int row, int col, ServoState player) {
  // Check horizontal
  if (countInDirection(row, col, 0, 1, player) >= winCondition) return true;
  
  // Check vertical
  if (countInDirection(row, col, 1, 0, player) >= winCondition) return true;
  
  // Check diagonal
  if (countInDirection(row, col, 1, 1, player) >= winCondition) return true;
  
  // Check anti-diagonal
  if (countInDirection(row, col, 1, -1, player) >= winCondition) return true;
  
  return false;
}

/**
 * Check for win condition anywhere on the board
 */
bool checkBoardForWin(ServoState player) {
  uint8_t boardSize = getBoardSize();
  uint8_t startRow = (currentGameMode == 0 && winCondition == 3) ? 1 : 0;
  uint8_t startCol = (currentGameMode == 0 && winCondition == 3) ? 1 : 0;
  uint8_t endRow = (currentGameMode == 0 && winCondition == 3) ? boardSize + 1 : boardSize;
  uint8_t endCol = (currentGameMode == 0 && winCondition == 3) ? boardSize + 1 : boardSize;
  
  // Check every position on the board for a win
  for (int row = startRow; row < endRow; row++) {
    for (int col = startCol; col < endCol; col++) {
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
 * Check if the board is full (all valid squares filled)
 * Works for all game modes (tic-tac-toe, gomoku, connect 4)
 */
bool isBoardFull() {
  uint8_t boardSize = getBoardSize();
  uint8_t startRow = (currentGameMode == 0 && winCondition == 3) ? 1 : 0;
  uint8_t startCol = (currentGameMode == 0 && winCondition == 3) ? 1 : 0;
  uint8_t endRow = (currentGameMode == 0 && winCondition == 3) ? boardSize + 1 : boardSize;
  uint8_t endCol = (currentGameMode == 0 && winCondition == 3) ? boardSize + 1 : boardSize;
  
  // Check all valid positions for the current game mode
  for (int row = startRow; row < endRow; row++) {
    for (int col = startCol; col < endCol; col++) {
      int servoIndex = row * GRID_SIZE + col;
      if (gridState[servoIndex] == STATE_NONE) {
        return false; // Found an empty square
      }
    }
  }
  
  return true; // All valid squares are filled
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
  while (isValidPosition(row, col) && 
         gridState[row * GRID_SIZE + col] == player) {
    count++;
    row += deltaRow;
    col += deltaCol;
  }
  
  // Count in negative direction
  row = startRow - deltaRow;
  col = startCol - deltaCol;
  while (isValidPosition(row, col) && 
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
  if (gameStarted && gameModeState == STATE_ACTIVE) return;
  
  // Set win condition based on gamemode
  // Gomoku modes (0-2): 3-5 in a row
  // Connect 4 modes (3-5): 3-5 in a row (map 3-5 to 0-2, then add 3)
  if (currentGameMode < 3) {
    winCondition = currentGameMode + 3; // Gomoku: 3-5
  } else {
    winCondition = currentGameMode - 3 + 3; // Connect 4: 3-5 (simplifies to currentGameMode - 0)
  }
  
  bool isConnect4 = (currentGameMode >= 3);
  
  Serial.print(F("Starting "));
  if (isConnect4) {
    Serial.print(F("Connect 4"));
  } else {
    Serial.print(F("Gomoku"));
  }
  Serial.print(F(" game ("));
  Serial.print(winCondition);
  Serial.println(F(" in a row)!"));
  
  // Clear the board completely first
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    gridState[i] = STATE_NONE;
    setServoPosition(i, servoPositions[STATE_NONE]);
  }
  delay(500); // Allow servos to reach cleared position before starting game
  
  // Set game state
  gameModeState = STATE_ACTIVE;
  gameActive = true;
  gameStarted = true;
  currentPlayer = STATE_X;
  
  // Set initial cursor position
  resetCursorToZeroPosition();
  
  delay(500); // Brief pause
  
  // Start cursor wiggle
  startCursorWiggle();
  
  // Notify first player (X) that it's their turn
  sendTurnNotification();
}

/**
 * Reset cursor to the correct zero position based on game type
 */
void resetCursorToZeroPosition() {
  // Reset old cursor position visually
  int oldServoIndex = cursorRow * GRID_SIZE + cursorCol;
  int oldBasePosition = servoPositions[gridState[oldServoIndex]];
  setServoPosition(oldServoIndex, oldBasePosition);
  
  // Determine game type
  bool isConnect4 = (currentGameMode >= 3);
  
  // Set cursor to correct zero position for this game type
  // For tic-tac-toe (3-in-a-row gomoku), valid board is at indices 1-3, so start at (1,1)
  // For Connect 4, always start at column 0 (row 0, col 0)
  // For all other games, start at (0,0)
  if (currentGameMode == 0 && winCondition == 3) {
    cursorRow = 1;
    cursorCol = 1;
  } else if (isConnect4) {
    // Connect 4: always start at column 0
    cursorRow = 0;
    cursorCol = 0;
  } else {
    cursorRow = 0;
    cursorCol = 0;
  }
  
  // Update new cursor position visually
  int newServoIndex = cursorRow * GRID_SIZE + cursorCol;
  int newBasePosition = servoPositions[gridState[newServoIndex]];
  setServoPosition(newServoIndex, newBasePosition);
  
  // Restart cursor wiggle
  startCursorWiggle();
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
  Serial.print(F("Player "));
  Serial.print(winner == STATE_X ? F("X") : F("O"));
  Serial.println(F(" wins! Starting win sequence..."));
  
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
  Serial.println(F("Win sequence complete. Resetting game..."));
  
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
  Serial.println(F("Resetting game..."));
  initializeGame();
}
