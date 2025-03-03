#include <Servo.h>
#include <EEPROM.h>

#define LEFT_M 12
#define RIGHT_M 13

constexpr int sensorPins[] = {A0, A1, A2, A3, A4};
constexpr int sensorLength = sizeof(sensorPins)/sizeof(sensorPins[0]);

constexpr int btn_pin = 2;


// Constants for EEPROM
const int EEPROM_MAGIC_ADDR = 0; // Address to store magic number
const int EEPROM_INSTRUCTION_COUNT_ADDR = 2; // Address to store number of instructions
const int EEPROM_INSTRUCTIONS_START_ADDR = 4; // Starting address for instructions
const unsigned int EEPROM_MAGIC_NUMBER = 0xABCD; // Marker for valid stored choreography


enum ButtonState { Pressed,
                     Released,
                     Off };

// class to detect button press with debounce
class Button {
  private:
    int pin;
    unsigned long last_t_state_changed;
    unsigned long debounce_duration;
    enum b_state { off,
                  on };
    bool last_state;
  public:
  
    Button(int pinNumber = 0) {
      set_pin(pinNumber);
    }

    void set_pin(int pinNumber) {
      pin = pinNumber;
    }

    void setup(int pin) {
      set_pin(pin);
      pinMode(pin, INPUT_PULLUP);
      last_state = current_state();
      debounce_duration = 40;  // millis
    }

    bool current_state() {
      if (!(digitalRead(pin))) {
        return on;
      } else {
        return off;
      }
    }

    ButtonState get_change_state() {
      if (changed()) {
        return last_state == on ? Pressed : Released;
      }
      return Off;
    }

    bool changed() {
      if (millis() - last_t_state_changed >= debounce_duration) {
        bool state = current_state();
        if (last_state != state) {
          last_t_state_changed = millis();
          last_state = state;
          return true;
        };
      };
      return false;
    }
};

// class for controlling motor speeds
class Motor : public Servo {
public:
  Motor(void) {
    _dir = 1;
  }
  void go(int percentage) {
    writeMicroseconds(1500 + _dir * percentage);
  }
  void setDirection(bool there) {
    if (there) {
      _dir = 1;
    } else {
      _dir = -1;
    }
  }
private:
  int _dir;
};

// PID Controller Class for line following
class PIDController {
private:
  float kp, ki, kd;
  float lastError;
  float integralError;
  unsigned long lastTime;

public:
  void setup(float p, float i, float d){
    kp = p;
    ki = i;
    kd = d;
    reset();
  }

  float compute(float error) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    
    // Proportional term
    float P = kp * error;
    
    // Integral term
    integralError += error * deltaTime;
    float I = ki * integralError;
    
    // Derivative term
    float derivative = (error - lastError) / deltaTime;
    float D = kd * derivative;
    
    // Total PID output
    float output = P + I + D;
    
    // Update for next iteration
    lastError = error;
    lastTime = currentTime;
    
    return output;
  }

  void reset() {
    lastError = 0;
    integralError = 0;
    lastTime = 0;
  }
};

// Class for sensors for line following (calculating error) and for other detection
class LineSensors {
private:
  int sensorValues[sensorLength];
  const int WHITE_THRESHOLD = 650;
  const int BLACK_THRESHOLD = 350;
  const float expectedCenter = 370;
  float lastError = 0.0;

public:
  void setup(){
    for (int i = 0; i < sensorLength; i++){
      pinMode(sensorPins[i], INPUT);
    }
  }

  int getSensorValue(int i){
    return sensorValues[i];
  }

  bool detectIntersection() {
    readSensors();
    
    bool farLeftBlack = sensorValues[0] < BLACK_THRESHOLD;
    bool farRightBlack = sensorValues[4] < BLACK_THRESHOLD;

    if(farLeftBlack || farRightBlack){
      return true;
    } else {
      return false;
    }
  }

  bool centerOnLine(){
    readSensors();

    return (sensorValues[2] < 300);
  }

  void readSensors() {
    for (int i = 0; i < 5; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
    }
  }

  float calculateError() {
    readSensors();

    bool leftOnLine = (sensorValues[1] < 500);

    float actualCenter = sensorValues[2];

    float error = actualCenter - expectedCenter;
    error = constrain(error, -350, 350);

    return error;
  }
};

enum ParserState {
  FirstPos,
  SecondPos,
  T,
  Heading,
  Time
};

// for direction of robot
enum RobotHeading{
  NORTH,
  EAST,
  SOUTH,
  WEST
};

// Struct for storing parsed instruction
struct Instruction {
  uint8_t column;
  uint8_t row;
  bool columnFirst;
  unsigned long time;
};

// Class for parsing choreography from any character stream
class ChoreographyParser {
  private:
    ParserState state = FirstPos;
    int charCount = 0;
    Instruction currentInstruction;
    RobotHeading startingHeading;
    int timeCharCount = 0;
    bool startParsed = false;
    unsigned long currentTime = 0;
  public:
    void printError(char* message, char c){
      Serial.print("> Input Error: ");
      Serial.print(message);
      Serial.print("'");
      Serial.print(c);
      Serial.print("'. At position: ");
      Serial.println(charCount);
      delay(1000);
      exit(1);
    }

    bool lastInstructionCorrect(){
      return (state == Time);
    }

    // returns true if a new instruction was parsed
    bool parseChar(char c){
      charCount++;

      bool isSeparator = strchr(" \t\r\n\f\v,;", c) != nullptr;

      bool finishedInstruction = false;

      switch (state){
        case FirstPos:
          if(isSeparator){
            break;
          }
          parseFirstPos(c);
          state = SecondPos;
          break;
        case SecondPos:
          if(isSeparator){
            printError("saparator between first and second position argument. Character: ", c);
          }
          parseSecondPos(c);
          if(startParsed){
            state = T;
          } else {
            state = Heading;
          }
          break;
        case Heading:
          if(isSeparator){
            if(startParsed){
              state = FirstPos;
            } else {
              printError("separator between starting position and direction. Character: ", c);
            }
          } else {
            startParsed = true;
            currentInstruction.time = 0;
            switch(toupper(c)){
              case 'N':
                startingHeading = NORTH;
                break;
              case 'E':
                startingHeading = EAST;
                break;
              case 'S':
                startingHeading = SOUTH;
                break;
              case 'W':
                startingHeading = WEST;
                break;
              default:
                startParsed = false;
                printError("expected starting direction (N, E, S, W) but got: ", c);
                break;
            }
            finishedInstruction = true;
          }
          break;
        case T:
          if(isSeparator){
            break;
          }
          if(toupper(c) == 'T'){
            state = Time;
            break;
          } else {
            printError("expected time identifier (t/T) but got: ", c);
          }
          break;
        case Time:
          finishedInstruction = parseTime(c, isSeparator);
          break;
      }

      return finishedInstruction;
      
    }

    // function for parsing first character of position
    void parseFirstPos(char c){
      if(isalpha(c)){
        uint8_t col = (toupper(c) - 'A') + 1;
        if(validCoordinate(col)){
          currentInstruction.columnFirst = true;
          currentInstruction.column = col;
        } else {
          printError("column out of range (A -> I). Got: ", c);
        }
      } else if (isdigit(c)){
        uint8_t row = (toupper(c) - '0');
        if(validCoordinate(row)){
          currentInstruction.columnFirst = false;
          currentInstruction.row = row;
        } else {
          printError("row out of range (1 -> 9). Got: ", c);
        }
      } else {
        printError("expected column (A -> I) or row (1 -> 9) but got: ", c);
      }
    }

    // function for parsing second character of position
    void parseSecondPos(char c){
      if(isalpha(c)){
        if(currentInstruction.columnFirst){
          printError("expected row (1 -> 9) after column but got letter: ", c);
        } else {
          uint8_t col = (toupper(c) - 'A') + 1;
          if(validCoordinate(col)){
            currentInstruction.column = col;
          } else {
            printError("column out of range (A -> I). Got: ", c);
          }
        }
      } else if (isdigit(c)){
        if(currentInstruction.columnFirst){
          uint8_t row = (toupper(c) - '0');
          if(validCoordinate(row)){
            currentInstruction.row = row;
          } else {
            printError("row out of range (1 -> 9). Got: ", c);
          }
        } else {
          printError("expected column (A -> I) after row but got letter: ", c);
        }
      }
    }

    bool validCoordinate(uint8_t coordinate){
      return (coordinate >= 1) && (coordinate <= 9);
    }

    // function for parsing digits after the time identifier (t/T)
    bool parseTime(char c, bool isSeparator){
      if(isSeparator){
        if(timeCharCount == 0){
          printError("separator between time identifier (t/T) and time. Character: ", c);
        } else {
          currentInstruction.time = (currentTime * 100); // from tenths of a second to ms
          resetForNextInstruction();
          return true;
        }
      }

      if(isdigit(c)){
        currentTime = (currentTime * 10) + (c - '0');
        timeCharCount++;
        return false;
      } else {
        printError("expected digit in time argument but got: ", c);
      }
    }

    Instruction getInstruction(){
      return currentInstruction;
    }

    RobotHeading getStartingHeading(){
      return startingHeading;
    }

    void resetForNextInstruction(){
      currentTime = 0;
      state = FirstPos;
      timeCharCount = 0;
    }

    void reset(){
      resetForNextInstruction();
      startParsed = false;
      charCount = 0;
    }
};

// Struct to store path between current position and next instruction
struct Path {
  int8_t turns[2];
  uint8_t secondTurnPosition;
  uint8_t pathLength;
  unsigned long time;
};

// Class for getting choreography instructions from Serial line or hardcoded string and for storing to and retrieving from EEPROM
class RobotChoreography {
  private:
    ChoreographyParser parser;
    const static int8_t maxInstructionCount = 40;
    Instruction instructions[maxInstructionCount];
    uint8_t instructionsLength;
    RobotHeading startingHeading;
    uint8_t readInstructionCounter = 0;
    uint8_t getInstructionCounter = 1; // skip starting position instruction
  public:

    void setup(){
      // Check if a choreography is stored in EEPROM
      if(isChoreographyStored()){
        // Serial.println("Reading from EEPROM.");
        loadChoreographyFromEEPROM();
      } else {
        // Serial.println("Using hard-coded choreography.");
        // otherwise use hard-coded choreography
        char choreography[] = "A1N E1 T8000 B2 T0 3A T35000 4C T567 D2 T700";
        int choreographyLength = strlen(choreography);

        parseStringChoreography(choreography, choreographyLength);
        
      }
    }

    bool isChoreographyStored(){
      uint16_t storedMagic;
      EEPROM.get(EEPROM_MAGIC_ADDR, storedMagic);
      return storedMagic == EEPROM_MAGIC_NUMBER;
    }

    void saveChoreographyToEEPROM(){
      instructionsLength ++;

      // magic number to indicate valid choreography
      EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_NUMBER);

      // store number of instructions
      EEPROM.put(EEPROM_INSTRUCTION_COUNT_ADDR, instructionsLength);
      
      // Store each instruction
      for (int i = 0; i < instructionsLength; i++) {
        int eepromAddr = EEPROM_INSTRUCTIONS_START_ADDR + (i * sizeof(Instruction));
        EEPROM.put(eepromAddr, instructions[i]);
      }

      // Store starting heading
      EEPROM.put(EEPROM_INSTRUCTIONS_START_ADDR + (instructionsLength * sizeof(Instruction)), startingHeading);
    }

    void loadChoreographyFromEEPROM(){
      // get number of instructions
      EEPROM.get(EEPROM_INSTRUCTION_COUNT_ADDR, instructionsLength);
      
      // get instructions
      for (int i = 0; i < instructionsLength; i++) {
        int eepromAddr = EEPROM_INSTRUCTIONS_START_ADDR + (i * sizeof(Instruction));
        EEPROM.get(eepromAddr, instructions[i]);
      }

      // get starting heading
      EEPROM.get(EEPROM_INSTRUCTIONS_START_ADDR + (instructionsLength * sizeof(Instruction)), startingHeading);
      
      // for choreography end check
      instructionsLength --;

      // reset get instruction counter
      getInstructionCounter = 1;
    }

    void reset() {
      getInstructionCounter = 1;
    }

    void parseStringChoreography(const char* choreography, int length){
      // reset for new parsing
      readInstructionCounter = 0;
      for(int i = 0; i < length; i++){
        char newCharacter = choreography[i];
        Serial.print(newCharacter);
        if(parser.parseChar(newCharacter)){
          instructions[readInstructionCounter++] = parser.getInstruction();
        }
      }
      if(parser.lastInstructionCorrect()){
        instructions[readInstructionCounter] = parser.getInstruction();
      }
      instructionsLength = readInstructionCounter;
      // reset for other parsing in the future
      readInstructionCounter = 0;
      startingHeading = parser.getStartingHeading();
      parser.reset();
    }

    void parseSerialInputChoreography(){
      // reset for new parsing
      readInstructionCounter = 0;
      Serial.println("Reading input from Serial line: ");
      while(Serial.available() > 0){
        char newCharacter = Serial.read();
        Serial.print(newCharacter);
        
        // if a complete instruction has been read
        if(parser.parseChar(newCharacter)){
          instructions[readInstructionCounter++] = parser.getInstruction();
          // Serial.print("Instruction Col: ");
          // Serial.println(instructions[readInstructionCounter - 1].column);
          // Serial.print("Instruction Row: ");
          // Serial.println(instructions[readInstructionCounter - 1].row);
          // Serial.print("Instruction Time: ");
          // Serial.println(instructions[readInstructionCounter - 1].time);
        }
      }
      instructionsLength = readInstructionCounter - 1;
      // reset for other parsing in the future
      readInstructionCounter = 0;
      startingHeading = parser.getStartingHeading();
      // save choreography to EEPROM for persistent storage
      saveChoreographyToEEPROM();
      parser.reset();
    }

    // function for getting next path in the choreography
    Path getNextPath(uint8_t col, uint8_t row, RobotHeading heading){
      Instruction nextInstruction = getNextInstruction();
      return getPath(col, row, heading, nextInstruction);
    }

    // function for getting path to the starting position
    Path getReturnPath(uint8_t col, uint8_t row, RobotHeading heading){
      Instruction startingInstruction = getStartingInstruction();
      return getPath(col, row, heading, startingInstruction);
    }

    // function for getting path from col, row to the nextInstruction
    Path getPath(uint8_t col, uint8_t row, RobotHeading heading, Instruction nextInstruction){
      Path nextPath;
      nextPath.time = nextInstruction.time;
      int8_t colDiff = nextInstruction.column - col;
      int8_t rowDiff = nextInstruction.row - row;
      nextPath.pathLength = abs(colDiff) + abs(rowDiff);
      // Serial.print("instruction col: ");
      // Serial.println(nextInstruction.column);
      // Serial.print("instruction row: ");
      // Serial.println(nextInstruction.row);
      // Serial.print("col: ");
      // Serial.println(col);
      // Serial.print("row: ");
      // Serial.println(row);
      // Serial.print("ColDiff: ");
      // Serial.println(colDiff);
      // Serial.print("RowDiff: ");
      // Serial.println(rowDiff);

      // Calculating first turn and second turn position
      if(nextInstruction.columnFirst){
        if(colDiff < 0){
          nextPath.turns[0] = WEST - heading;
          heading = WEST;
        } else if(colDiff > 0){
          nextPath.turns[0] = EAST - heading;
          heading = EAST;
        } else {
          nextPath.turns[0] = 0;
        }
        nextPath.secondTurnPosition = abs(colDiff);
      } else {
        if(rowDiff < 0){
          nextPath.turns[0] = SOUTH - heading;
          heading = SOUTH;
        } else if(rowDiff > 0){
          nextPath.turns[0] = NORTH - heading;
          heading = NORTH;
        } else {
          nextPath.turns[0] = 0;
        }
        nextPath.secondTurnPosition = abs(rowDiff);
      }

      // Calculating second turn
      if(nextInstruction.columnFirst){
        if(rowDiff < 0){
          nextPath.turns[1] = SOUTH - heading;
        } else if(rowDiff > 0){
          nextPath.turns[1] = NORTH - heading;
        } else {
          nextPath.turns[1] = 0;
        }
      } else {
        if(colDiff < 0){
          nextPath.turns[1] = WEST - heading;
        } else if(colDiff > 0){
          nextPath.turns[1] = EAST - heading;
        } else {
          nextPath.turns[1] = 0;
        }
      }

      // Serial.print("Path Length: ");
      // Serial.println(nextPath.pathLength);
      // Serial.print("First Turn: ");
      // Serial.println(nextPath.turns[0]);
      // Serial.print("Second Turn: ");
      // Serial.println(nextPath.turns[1]);
      // Serial.print("Second Turn Position: ");
      // Serial.println(nextPath.secondTurnPosition);

      return nextPath;

    }

    // function for getting starting "position"
    Instruction getStartingInstruction(){
      return instructions[0];
    }

    RobotHeading getStartingHeading(){
      return startingHeading;
    }

    Instruction getNextInstruction() {
      return instructions[getInstructionCounter++];
    }

    // if the last instruction has already been retrieved
    bool isEnd() {
      return getInstructionCounter >= instructionsLength + 1;
    }
};


// Different states for robot state machine

enum RobotState {
  STOPPED,
  DANCING,
  WAITING,
  END
};

enum RobotDancingState {
  GOING,
  INTERSECTION,
  TURNING
};

enum TurningState {
  TURN,
  SEARCHING
};

class RobotDance {
  private:
    RobotChoreography choreography;
    Button button;
    LineSensors sensors;
    PIDController pidController;
    Motor leftMotor;
    Motor rightMotor;

    // states for state machine
    RobotState state;
    RobotDancingState dancingState;
    TurningState turningState;


    // variables for intersection
    unsigned long lastIntersection = 0;
    unsigned long intersectionDelay = 400;

    int baseSpeed = 70;

    // variables for turning
    unsigned long currentTurnTime;
    int8_t currentTurnDirection;
    unsigned long lastTurnStart = 0;
    unsigned long lastTurnSearchStart = 0;
    int turnSpeed = 50;
    int leftTurnSpeed;
    int rightTurnSpeed;
    unsigned long turnLength = 1000;
    unsigned long turnSearchLength = 700;

    Path currentPath;
    uint8_t pathCounter = 0;

    // current position and heading
    RobotHeading heading;
    uint8_t col;
    uint8_t row;

    // variables for returning
    bool returning;
    bool returnStarted;
    bool returned = false;
    RobotHeading startingHeading;

    unsigned long startingTime;

  public:
    void setup(){
      leftMotor.attach(LEFT_M, 500, 2500);
      leftMotor.setDirection(true);
      rightMotor.attach(RIGHT_M, 500, 2500);
      rightMotor.setDirection(false);

      sensors.setup();
      button.setup(btn_pin);
      pidController.setup(0.30, 0.0, 0.0);
      state = STOPPED;
      choreography.setup();
    }

    void update() {

      bool buttonPressed = button.get_change_state() == Pressed;

      switch(state){
        case STOPPED:
          handleStopped(buttonPressed);
          break;
        case DANCING:
          if(buttonPressed){
            returnStarted = true;
          }
          handleDancing();
          break;
        case WAITING:
          handleWaiting();
          break;
        case END:
          handleEnd(buttonPressed);
          break;
      }

    }

    void handleEnd(bool buttonPressed){
      setMotors(0, 0);
      if(buttonPressed){
        setGoingBack();
        int8_t turn = getNextPathTurn();
        if(turn == 0){
          dancingState = GOING;
        } else {
          pidController.reset();
          setTurn(turn);
        }
      }
    }

    void handleStopped(bool buttonPressed){
      setMotors(0, 0);
      if (Serial.available() > 0) {
        choreography.parseSerialInputChoreography();
      }
      if(buttonPressed){
        startingTime = millis();
        // Serial.println("STARTING CHOREOGRAPHY");
        resetReturnVariables();
        choreography.reset();
        heading = choreography.getStartingHeading();
        startingHeading = heading;
        Instruction startingPosition = choreography.getStartingInstruction();
        col = startingPosition.column;
        row = startingPosition.row;
        currentPath = choreography.getNextPath(col, row, heading);
        pathCounter = 0;
        int turn = getNextPathTurn();
        setTurn(turn);
        state = DANCING;
      }
    }

    void handleWaiting(){
      setMotors(0, 0);
      if((millis() - startingTime) >= currentPath.time ){
        // *100 because from ms to tenths of a second
        state = DANCING;
        updatePath(choreography.getNextPath(col, row, heading));
        pathCounter++;
        int8_t turn = currentPath.turns[0];
        if(turn == 0){
          dancingState = GOING;
        } else {
          pidController.reset();
          setTurn(turn);
        }
      }
    }

    void setGoingBack(){
      returnStarted = false;
      updatePath(choreography.getReturnPath(col, row, heading));
      returning = true;
      dancingState = TURNING;
      state = DANCING;
    }

    void handleDancing(){
      switch(dancingState){
        case TURNING:
          handleTurning();
          break;
        case GOING:
          handleGoing();
          break;
        case INTERSECTION:
          handleIntersection();
          break;
      }
    }

    // normal line following
    void handleGoing(){
      // if intersection is detected, handle it
      if(sensors.detectIntersection()){
        dancingState = INTERSECTION;
        lastIntersection = millis();
        return;
      }

      float error = sensors.calculateError();
      float correction = pidController.compute(error);
      
      correctMotors(correction);
    }

    void handleIntersection(){
      if(millis() - lastIntersection < intersectionDelay){
        setMotors(baseSpeed, baseSpeed);
      } else {
        // update current position based on heading
        switch(heading){
          case NORTH:
            row++;
            break;
          case EAST:
            col++;
            break;
          case SOUTH:
            row--;
            break;
          case WEST:
            col--;
            break;
        }

        if(returnStarted){
          setGoingBack();
        }

        // set next intersection turn in current path
        int8_t turn = getNextPathTurn();
        if(turn == 0){
          dancingState = GOING;
        } else {
          pidController.reset();
          setTurn(turn);
        }
      }
    }

    // function to get next path turn in intersection based on path and current position in path
    int8_t getNextPathTurn(){
      if(pathCounter == 0){
        pathCounter++;
        return currentPath.turns[0];
      } else if (pathCounter >= currentPath.pathLength){
        if(!choreography.isEnd() && !returning){
          if((millis() - startingTime) < currentPath.time ){
            state = WAITING;
            return 0;
          }
          updatePath(choreography.getNextPath(col, row, heading));
          pathCounter++;
          return currentPath.turns[0];
        } else if (returning) {
          pathCounter = 0;
          returned = true;
          return (startingHeading - heading);
        } else {
          pathCounter = 0;
          state = END;
        }
        return 0;
      } else if (pathCounter == currentPath.secondTurnPosition){
        pathCounter++;
        return currentPath.turns[1];
      } else {
        pathCounter ++;
        return 0;
      }
    }
 
    void changeHeading(RobotHeading newHeading){
      setTurn(newHeading - heading);
      heading = newHeading;
    }

    void setTurn(int8_t turn){
      // Serial.print("HEADING before turn: ");
      // Serial.print(heading);
      turn = turn % 4;
      // Serial.print("Turn: ");
      // Serial.println(turn);

      // no turn
      if(turn == 0){
        dancingState = GOING;
        return;
      }

      if(turn == -2){
        turn = 2;
      }
      if(turn == 3){
        turn = -1;
      }
      if(turn == -3){
        turn = 1;
      }

      lastTurnStart = millis();
      turningState = TURN;
      currentTurnTime = abs(turn) * turnLength;
      currentTurnDirection = turn/abs(turn);
      leftTurnSpeed = currentTurnDirection * turnSpeed;
      rightTurnSpeed = - (currentTurnDirection * turnSpeed);
      heading = (heading + turn + 4) % 4;
      // Serial.print("HEADING after turn: ");
      // Serial.print(heading);

      dancingState = TURNING;
    }

    void handleTurning(){
      switch (turningState){
        case TURN:
          if((millis() - lastTurnStart) < (currentTurnTime - turnSearchLength)){
            setMotors(leftTurnSpeed, rightTurnSpeed);
          } else {
            turningState = SEARCHING;
          }
          break;
        case SEARCHING: // searching for line
          if(sensors.centerOnLine()){
            
            if(returned){
              turningState = TURN;
              dancingState = TURNING;
              state = STOPPED;
              break;
            }

            turningState = TURN;
            dancingState = GOING;
            
          } else {
            setMotors(leftTurnSpeed, rightTurnSpeed);
          }
          break;
      }
    }

    // to correct direction when line following
    void correctMotors(float correction){
      // Apply correction
      int leftSpeed = baseSpeed + correction;
      int rightSpeed = baseSpeed - correction;
      
      // Constrain speeds to prevent overflow
      leftSpeed = constrain(leftSpeed, 0, 100);
      rightSpeed = constrain(rightSpeed, 0, 100);
      
      setMotors(leftSpeed, rightSpeed);
    }

    void setMotors(int left, int right){
      leftMotor.go(left);
      rightMotor.go(right);
    }

    void updatePath(Path path){
      currentPath = path;
      pathCounter = 0;
    }

    void resetReturnVariables(){
      returning = false;
      returnStarted = false;
      returned = false;
    }
};


RobotDance robot;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  robot.setup();
}

void loop() {
  robot.update();
}
