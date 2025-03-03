# Line Following Robot Documentation

## Table of Contents

- [Overview](#overview)
- [Classes](#classes)
  - [Button](#button)
  - [Motor](#motor)
  - [PIDController](#pidcontroller)
  - [LineSensors](#linesensors)
  - [ChoreographyParser](#choreographyparser)
  - [RobotChoreography](#robotchoreography)
  - [RobotDance](#robotdance)
- [Data Structures](#data-structures)
- [State Enums](#state-enums)
- [Constants](#constants)
- [Usage](#usage)
- [Choreography Format](#choreography-format)

## Overview

This program implements a line-following robot capable of executing choreographed paths on a grid. The robot follows lines between intersections and turns at intersections to navigate to specified positions. The choreography can be loaded from EEPROM memory or received through serial line.

## Classes

### Button

Class that handles button input with debounce delay.

```cpp
ButtonState get_change_state()
```

- Returns the button state change (Pressed/Released/Off)

### Motor

A class extending the Servo class to control motor speeds.

### PIDController

Implements a PID (Proportional-Integral-Derivative) controller for line following.

#### Methods

```cpp
void setup(float p, float i, float d)
```

- Initializes PID constants
- Parameters:
  - p: Proportional constant
  - i: Integral constant
  - d: Derivative constant

```cpp
float compute(float error)
```

- Calculates PID output based on current error

```cpp
void reset()
```

- Resets PID controller state

### LineSensors

Manages the line sensors for detecting lines and intersections.

#### Methods

```cpp
bool detectIntersection()
```

- Detects if robot is at an intersection

```cpp
bool centerOnLine()
```

- Checks if center sensor is on the line (for detecting turn end)

```cpp
void readSensors()
```

- Updates all sensor values

```cpp
float calculateError()
```

- Calculates position error for PID line following

### ChoreographyParser

Parses choreography instructions from input strings.

#### Methods

```cpp
bool parseChar(char c)
```

- Parses a single character of choreography input
- Returns: true if instruction is complete

```cpp
void printError(char* message, char c)
```

- Prints parsing error messages and halts program execution with exit code 1

```cpp
bool lastInstructionCorrect()
```

- Verifies if last instruction was parsed correctly

```cpp
void parseFirstPos(char c)
```

- Parses first position character

```cpp
void parseSecondPos(char c)
```

- Parses second position character

```cpp
bool validCoordinate(uint8_t coordinate)
```

- Validates coordinate values (range 1->9)

```cpp
bool parseTime(char c, bool isSeparator)
```

- Parses time values after T identifier

```cpp
Instruction getInstruction()
```

- Returns current parsed instruction

```cpp
RobotHeading getStartingHeading()
```

- Returns parsed starting heading

```cpp
void reset()
```

- Resets parser state

### RobotChoreography

Manages choreography storage and execution.

#### Methods

```cpp
void setup()
```

- Initializes choreography from EEPROM or hardcoded values

```cpp
bool isChoreographyStored()
```

- Checks for valid choreography in EEPROM

```cpp
void saveChoreographyToEEPROM()
```

- Stores choreography to EEPROM

```cpp
void loadChoreographyFromEEPROM()
```

- Loads choreography from EEPROM

```cpp
void parseStringChoreography(const char* choreography, int length)
```

- Parses choreography from string

```cpp
void parseSerialInputChoreography()
```

- Parses choreography from serial input

```cpp
Path getNextPath(uint8_t col, uint8_t row, RobotHeading heading)
```

- Calculates path to next position

```cpp
Path getReturnPath(uint8_t col, uint8_t row, RobotHeading heading)
```

- Calculates path back to starting position

### RobotDance

Main robot control class implementing the state machine.

#### Methods

```cpp
void setup()
```

- Initializes all robot components

```cpp
void update()
```

- Main update loop handling state transitions

```cpp
void handleEnd(bool buttonPressed)
```

```cpp
void handleStopped(bool buttonPressed)
```

```cpp
void handleWaiting()
```

```cpp
void handleDancing()
```

```cpp
void handleGoing()
```

```cpp
void handleIntersection()
```

- Handle logic for each state

## Data Structures

### Instruction

```cpp
struct Instruction {
    uint8_t column;       // grid column (A-I)
    uint8_t row;          // grid row (1-9)
    bool columnFirst;     // whether to move to column or row first
    unsigned long time;   // time to wait at position
};
```

### Path

```cpp
struct Path {
    int8_t turns[2];
    uint8_t secondTurnPosition;  // position of second turn in path
    uint8_t pathLength;          // number of intersections in path
    unsigned long time;          // time to wait at destination
};
```

## State Enums

### RobotState

```cpp
enum RobotState {
    STOPPED,    // waiting for button press to start choreography
    DANCING,    // executing choreography
    WAITING,    // waiting at position based on instruction time
    END         // choreography complete
};
```

### RobotDancingState

```cpp
enum RobotDancingState {
    GOING,          // following line
    INTERSECTION,   // at intersection
    TURNING         // executing turn
};
```

### RobotHeading

```cpp
enum RobotHeading {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};
```

## Usage

- Power on robot
- Choose choreography source:
  - Use stored choreography from EEPROM or hardcoded
  - Send new choreography via serial line
- Press button to start the "dance"
- Press button during "dance" to return to starting position and orientation

## Choreography Format

Instructions should be in the form: `[Column][Row]T[Time]`

Example:

```
A1N E1 T8000 B2 T0
```

- Letters A-I represent columns
- Numbers 1-9 represent rows
- The order of column and row determines the path that is taken (if the column is first, the robot will change its colum position first)
- T followed by number represents instruction end time measured from the start of the "dance"
- First position must include orientation (N/E/S/W)
- Instructions have to be separated by at least one "separator char": space, tab, new line, comma or semicolon
