# ECEN 1400 Final Project: Interactive Multi-Game Board System

A physical implementation of multiple board games (Gomoku and Connect 4) using Arduino microcontrollers and servo motors, featuring dual input controllers with haptic feedback and game mode selection.

## 📄 [Project Final Report](/final-report.pdf)

## 🎮 Project Overview

This project creates an interactive 5x5 game board system that supports multiple game modes. Players use physical controllers to place X and O pieces represented by servo motor positions. The system includes haptic feedback, game mode selection, and supports two different input methods (button-based and joystick-based controllers).

## 🏗️ System Architecture

The system consists of three main components:

### 1. Main Board Controller (`main-board/`)
- **Hardware**: Arduino Uno with 2x PCA9685 PWM Servo Driver boards
- **Function**: Controls 25 servo motors arranged in a 5x5 grid
- **Features**: 
  - Multiple game modes (Gomoku 3-5 in a row, Connect 4 3-5 in a row)
  - Game mode selection with visual feedback
  - Game logic and win detection
  - Cursor animation and visual feedback
  - Dual controller communication (separate X and O player controllers)
  - Win sequence animations
  - On-board button controls (SELECT and RESET/START)
  - Movement queue system for optimized servo control

### 2. Coarse Input Controller (`coarse/`)
- **Hardware**: Arduino Uno with 5 digital buttons
- **Function**: Provides directional input via discrete buttons (typically used as one player controller)
- **Features**:
  - Up/Down/Left/Right/Select button inputs
  - Dual UART communication (Hardware Serial + SoftwareSerial)
  - Haptic feedback motor control with multiple patterns

### 3. Fine Input Controller (`fine/`)
- **Hardware**: Arduino Uno with analog joystick
- **Function**: Provides directional input via joystick (typically used as the other player controller)
- **Features**:
  - Analog joystick with threshold detection
  - Dual UART communication (Hardware Serial + SoftwareSerial)
  - Haptic feedback motor control with multiple patterns

## 🔌 Pin Configurations

### Main Board Controller
- **PCA9685 Board 1** (Address 0x40, A0 bridged): Controls servos 0-14
- **PCA9685 Board 2** (Address 0x60, all address pins grounded): Controls servos 15-24
- **X Player Controller**: AltSoftSerial on pins 8/9 (RX/TX)
- **O Player Controller**: SoftwareSerial on pins 5/6 (RX/TX)
- **SELECT Button**: Pin 7 (INPUT_PULLUP)
- **RESET/START Button**: Pin 8 (INPUT_PULLUP) - Note: conflicts with AltSoftSerial, consider different pin

### Coarse Controller
- **Buttons**: 
  - Up: Pin 8
  - Down: Pin 9
  - Left: Pin 10
  - Right: Pin 11
  - Select: Pin 12
- **Haptic Motor**: Pin 13
- **UART**: Hardware Serial (0/1) + SoftwareSerial (4/5 - RX/TX)

### Fine Controller
- **Joystick X**: Pin A0
- **Joystick Y**: Pin A1
- **Select Button**: Pin 3 (INPUT_PULLUP)
- **Haptic Motor**: Pin 13
- **UART**: Hardware Serial (0/1) + SoftwareSerial (6/7 - RX/TX)

## 🎯 Game Features

### Game Modes
The system supports 6 different game modes:

**Gomoku Modes (0-2):**
- Mode 0: 3 in a row (tic-tac-toe style, uses 3x3 sub-board)
- Mode 1: 4 in a row
- Mode 2: 5 in a row (classic Gomoku)

**Connect 4 Modes (3-5):**
- Mode 3: Connect 3
- Mode 4: Connect 4 (classic)
- Mode 5: Connect 5

### Servo States
- **STATE_NONE** (Position 300): Empty space
- **STATE_X** (Position 470): Player X piece
- **STATE_O** (Position 100): Player O piece

### Game Mechanics
- **5x5 Grid**: 25 positions for game pieces
- **Variable Win Condition**: 3-5 pieces in a row depending on selected mode
- **Turn-based**: Players alternate turns (X goes first)
- **Visual Cursor**: Animated cursor shows current selection
  - Gomoku: Full 2D movement (up/down/left/right)
  - Connect 4: Horizontal movement only (left/right) in top row, pieces drop to lowest available position
- **Haptic Feedback**: Controllers vibrate on valid/invalid moves and turn changes
- **Game Mode Selection**: Visual pattern on board shows selected mode before game starts

### Animations
- **Cursor Wiggle**: Visual indication of current position (different patterns for empty vs occupied positions)
- **Checkerboard Pattern**: Idle state animation (alternates every 7 seconds)
- **Game Mode Display**: Visual pattern showing selected game mode (diagonal for Gomoku, bottom row for Connect 4)
- **Connect 4 Drop Animation**: Pieces animate falling from top to target position
- **Win Sequence**: Celebration animation when game is won (expanding wave pattern)

## 📡 Communication Protocol

### Input Commands (Controllers → Main Board)
- `u` - Move cursor up (Gomoku only)
- `d` - Move cursor down (Gomoku only)
- `l` - Move cursor left (or decrease game mode in selection)
- `r` - Move cursor right (or increase game mode in selection)
- `s` - Select/place piece (or start game in selection)
- `x` or `o` - Decrease game mode (in game selection mode)

### Feedback Commands (Main Board → Controllers)
- `y` - Valid move (yes)
- `n` - Invalid move (no)
- `t` - Turn notification (your turn)
- `w` - Game won (win pattern)
- `l` - Game lost (lose pattern)

**Note**: Feedback is sent only to the active player's controller (X or O player receives separate communication)

### Haptic Patterns
- **Yes Pattern** (`y`): Three quick short pulses (...)
- **No Pattern** (`n`): Four long pulses (----)
- **Turn Pattern** (`t`): Long-short-long pulses (-.-)
- **Win Pattern** (`w`): Four dots, two dashes (....--)
- **Lose Pattern** (`l`): Four dashes, two dots (----..)

**Timing**:
- Short pulse: 100ms
- Long pulse: 300ms
- Pause between pulses: 150ms

## 🛠️ Setup Instructions

### Hardware Setup
1. **Main Board**:
   - Connect 2x PCA9685 boards to Arduino Uno via I2C (SDA/SCL)
   - Connect 25 servo motors to PWM outputs
   - First PCA9685: Bridge A0 pin for address 0x40, controls servos 0-14
   - Second PCA9685: Ground all address pins for address 0x60, controls servos 15-24
   - Connect SELECT button to pin 7 (with pull-up)
   - Connect RESET/START button to pin 8 (with pull-up) - Note: may conflict with AltSoftSerial
   - Connect X player controller via AltSoftSerial (pins 8/9)
   - Connect O player controller via SoftwareSerial (pins 5/6)

2. **Controllers**:
   - Connect buttons/joystick as per pin configurations
   - Connect haptic motors to pin 13
   - Connect UART communication lines (both Hardware Serial and SoftwareSerial)

### Software Setup
1. Install required Arduino libraries:
   - `Adafruit PWM Servo Driver Library` (for PCA9685 control)
   - `SoftwareSerial` (built-in, for O player controller)
   - `AltSoftSerial` (for X player controller - more reliable than SoftwareSerial)

2. Upload firmware to each Arduino:
   - `main-board.ino` to main controller
   - `coarse.ino` to button controller (assign to one player)
   - `fine.ino` to joystick controller (assign to other player)

## 🎮 How to Play

1. **Enter Game Selection**: Press SELECT button on main board or 's' on either controller
2. **Select Game Mode**: 
   - Use left/right buttons or l/r commands to cycle through modes (0-5)
   - Board displays visual pattern showing selected mode
   - Gomoku modes (0-2): Diagonal pattern showing pieces in a row
   - Connect 4 modes (3-5): Bottom row pattern showing pieces in a row
3. **Start Game**: Press SELECT button again or 's' command to start
4. **Move Cursor**: 
   - Gomoku: Use up/down/left/right to navigate full board
   - Connect 4: Use left/right to select column (cursor stays in top row)
5. **Place Piece**: Press select to place your piece
   - Connect 4: Piece automatically drops to lowest available position in column
6. **Win**: Get required pieces in a row (3-5 depending on mode) to win
7. **Reset**: Press RESET/START button on main board to return to idle, or game automatically resets after win sequence

## 🔧 Technical Details

### Servo Control
- **Frequency**: 50 Hz PWM
- **Pulse Range**: 90-525 (out of 4096)
- **Actual Positions**: 
  - None: 300 (middle-low)
  - X: 470 (middle-high)
  - O: 100 (low)
- **Movement Queue**: Movements are queued and processed with randomization and staggering to prevent power spikes
- **Concurrent Movements**: Maximum 10 servos can move simultaneously
- **Movement Duration**: 300ms per movement

### Debouncing
- **Button Debounce**: 50ms (main board and controllers)
- **Joystick Debounce**: 100ms
- **Joystick Threshold Values**: 
  - Left: < 400
  - Right: > 600
  - Up: < 400
  - Down: > 600
  - Center: 400-600 (both axes)

### Timing
- **Haptic Pulse**: 100ms duration (short), 300ms (long)
- **Pattern Timing**: 100ms short, 300ms long pulses, 150ms pause between
- **Cursor Animation**: 
  - Empty position: 300ms intervals, ±50 amplitude
  - Occupied position: 150ms intervals, ±25 amplitude
- **Checkerboard Pattern**: Changes every 7 seconds
- **Win Sequence**: 200ms per step, 5 seconds total
- **Connect 4 Drop**: ~200-300ms per row during animation

## 🎯 Game Rules

### Gomoku Modes
- Players alternate turns (X goes first)
- Place pieces on empty positions only
- Win by getting the required pieces in a row (3, 4, or 5 depending on mode)
- Rows can be horizontal, vertical, or diagonal
- Mode 0 (3-in-a-row): Uses 3x3 sub-board (indices 1-3)
- Game ends on win or board full (draw)

### Connect 4 Modes
- Players alternate turns (X goes first)
- Pieces drop to the lowest available position in selected column
- Cannot place in full columns
- Win by getting the required pieces in a row (3, 4, or 5 depending on mode)
- Rows can be horizontal, vertical, or diagonal
- Game ends on win or board full (draw)

## 📁 Project Structure

```
ecen1400-final/
├── README.md              # This file
├── main-board/            # Main game controller
│   └── main-board.ino     # Servo control and game logic
├── coarse/                # Button input controller
│   └── coarse.ino         # Digital button handling
└── fine/                  # Joystick input controller
    └── fine.ino           # Analog joystick handling
```

## 🎓 Educational Value

This project demonstrates:
- **Microcontroller Programming**: Arduino C++ programming with state machines
- **Hardware Integration**: Servo control via I2C, multiple PWM driver boards
- **Game Development**: Multiple game modes, win detection algorithms, turn-based logic
- **User Interface Design**: Haptic feedback, visual feedback, game mode selection
- **Communication Protocols**: Dual UART serial communication (Hardware Serial + SoftwareSerial/AltSoftSerial)
- **System Integration**: Multi-device coordination with separate player controllers
- **Resource Management**: Movement queue system, concurrent movement limiting to prevent power issues
- **Animation Systems**: Cursor animations, win sequences, drop animations


## 🔧 Additional Features

### Simulation Mode
The main board code includes a `SIMULATION_MODE` flag (set to 0 by default) that allows testing game logic without servo hardware. When enabled, board state is printed to serial instead of moving servos.

### Debug Commands (Serial Monitor)
- `start` - Enter game selection mode
- `display` - Show current game state
- `reset` - Reset game to idle state

### State Management
- **IDLE**: Shows checkerboard pattern, waiting for game start
- **GAME_SELECT**: Shows selected game mode pattern, allows mode selection
- **ACTIVE**: Game is running, players can make moves

---

**Course**: ECEN 1400 - Introduction to Digital And Analog Electronics  
**Project**: Interactive Multi-Game Board System  
**Technologies**: Arduino, C++, Servo Motors, Haptic Feedback, I2C, UART Communication
