# ECEN 1400 Final Project: Interactive Gomoku Game

A physical implementation of the Gomoku (Five-in-a-Row) board game using Arduino microcontrollers and servo motors, featuring dual input controllers with haptic feedback.

## 🎮 Project Overview

This project creates an interactive 5x5 Gomoku game board where players use physical controllers to place X and O pieces represented by servo motor positions. The system includes haptic feedback for enhanced user experience and supports two different input methods.

## 🏗️ System Architecture

The system consists of three main components:

### 1. Main Board Controller (`main-board/`)
- **Hardware**: Arduino Uno with 2x PCA9685 PWM Servo Driver boards
- **Function**: Controls 25 servo motors arranged in a 5x5 grid
- **Features**: 
  - Game logic and win detection
  - Cursor animation and visual feedback
  - Communication with input controllers
  - Win sequence animations

### 2. Coarse Input Controller (`coarse/`)
- **Hardware**: Arduino Uno with 5 digital buttons
- **Function**: Provides directional input via discrete buttons
- **Features**:
  - Up/Down/Left/Right/Select button inputs
  - Dual UART communication
  - Haptic feedback motor control

### 3. Fine Input Controller (`fine/`)
- **Hardware**: Arduino Uno with analog joystick
- **Function**: Provides directional input via joystick
- **Features**:
  - Analog joystick with threshold detection
  - Dual UART communication
  - Haptic feedback motor control

## 🔌 Pin Configurations

### Main Board Controller
- **PCA9685 Board 1** (Address 0x40): Controls servos 0-14
- **PCA9685 Board 2** (Address 0x41): Controls servos 15-24
- **Communication**: SoftwareSerial on pins 4/5 and 6/7

### Coarse Controller
- **Buttons**: Pins 8-12 (Up/Down/Left/Right/Select)
- **Haptic Motor**: Pin 13
- **UART**: Hardware Serial (0/1) + SoftwareSerial (4/5)

### Fine Controller
- **Joystick X**: Pin A0
- **Joystick Y**: Pin A1
- **Select Button**: Pin 3
- **Haptic Motor**: Pin 13
- **UART**: Hardware Serial (0/1) + SoftwareSerial (4/5)

## 🎯 Game Features

### Servo States
- **STATE_NONE** (Low Position): Empty space
- **STATE_X** (Middle Position): Player X piece
- **STATE_O** (High Position): Player O piece

### Game Mechanics
- **5x5 Grid**: 25 positions for game pieces
- **Win Condition**: 5 pieces in a row (horizontal, vertical, or diagonal)
- **Turn-based**: Players alternate turns
- **Visual Cursor**: Animated cursor shows current selection
- **Haptic Feedback**: Controllers vibrate on valid/invalid moves

### Animations
- **Cursor Wiggle**: Visual indication of current position
- **Checkerboard Pattern**: Idle state animation
- **Win Sequence**: Celebration animation when game is won

## 📡 Communication Protocol

### Input Commands (Controllers → Main Board)
- `u` - Move cursor up
- `d` - Move cursor down
- `l` - Move cursor left
- `r` - Move cursor right
- `s` - Select/place piece

### Feedback Commands (Main Board → Controllers)
- `y` - Valid move (yes)
- `n` - Invalid move (no)
- `t` - Turn notification (test pattern)

### Haptic Patterns
- **Yes Pattern** (`y`): Two short pulses (..)
- **No Pattern** (`n`): Three long pulses (---)
- **Turn Pattern** (`t`): Long-short-long pulses (-.-)

## 🛠️ Setup Instructions

### Hardware Setup
1. **Main Board**:
   - Connect 2x PCA9685 boards to Arduino Uno via I2C
   - Connect 25 servo motors to PWM outputs
   - Bridge A0 on first PCA9685 for address 0x40
   - Ground all address pins on second PCA9685 for address 0x41

2. **Controllers**:
   - Connect buttons/joystick as per pin configurations
   - Connect haptic motors to pin 13
   - Connect UART communication lines

### Software Setup
1. Install required Arduino libraries:
   - `Adafruit PWM Servo Driver Library`
   - `SoftwareSerial` (built-in)

2. Upload firmware to each Arduino:
   - `main-board.ino` to main controller
   - `coarse.ino` to button controller
   - `fine.ino` to joystick controller

## 🎮 How to Play

1. **Start Game**: Press select button on either controller
2. **Move Cursor**: Use directional inputs to navigate the board
3. **Place Piece**: Press select to place your piece
4. **Win**: Get 5 pieces in a row to win
5. **Reset**: Game automatically resets after win sequence

## 🔧 Technical Details

### Servo Control
- **Frequency**: 50 Hz PWM
- **Pulse Range**: 150-600 (out of 4096)
- **Positions**: 
  - None: 150 (low)
  - X: 375 (middle)
  - O: 600 (high)

### Debouncing
- **Button Debounce**: 50ms
- **Joystick Debounce**: 100ms
- **Threshold Values**: 400-600 for joystick center detection

### Timing
- **Haptic Pulse**: 100ms duration
- **Pattern Timing**: 100ms short, 300ms long pulses
- **Cursor Animation**: 300ms intervals

## 🎯 Game Rules

- Players alternate turns (X goes first)
- Place pieces on empty positions only
- Win by getting 5 pieces in a row
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
- **Microcontroller Programming**: Arduino C++ programming
- **Hardware Integration**: Servo control, I2C communication
- **Game Development**: State machines, game logic
- **User Interface Design**: Haptic feedback, visual feedback
- **Communication Protocols**: UART serial communication
- **System Integration**: Multi-device coordination

## 🔮 Future Enhancements

- Sound effects for moves and wins
- LED indicators for player turns
- Score tracking and statistics
- Different game modes (3x3, 7x7)
- AI opponent integration
- Wireless controller support

---

**Course**: ECEN 1400 - Introduction to Digital Systems  
**Project**: Interactive Gomoku Game Implementation  
**Technologies**: Arduino, C++, Servo Motors, Haptic Feedback
