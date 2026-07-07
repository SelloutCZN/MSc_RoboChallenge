/*====================================================================
================== DC MOTOR & MOTOR DRIVER CONTROL ===================
=====================================================================*/

// ---------------- MOTOR DRIVER (TB6612FNG) ----------------
// LEFT MOTOR = A side
#define AIN1 7
#define AIN2 6
#define PWMA 5  

// RIGHT MOTOR = B side
#define BIN1 8
#define BIN2 9
#define PWMB 10

//#define STBY   standby pin

// ---------------- ENCODER PINS ----------------
#define LEFT_ENC_A 20
#define LEFT_ENC_B 21
#define RIGHT_ENC_A 3
#define RIGHT_ENC_B 2

volatile long leftCount = 0;
volatile long rightCount = 0;

// ---------------- CALIBRATION ----------------
long COUNTS_PER_360 = 3000;  

// ---------------- ENCODER INTERRUPTS ----------------
void leftEncoderISR() {
  leftCount++;
}

void rightEncoderISR() {
  rightCount++;
}

// ---------------- MOTOR FUNCTIONS ----------------
/*void enableMotors() {
  digitalWrite(STBY, HIGH);
}*/

const int turnSpeedPWM = 90;
const int forwardSpeedPWM = 90;

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void driveForward(int spd) {
  //enableMotors();
  analogWrite(PWMA, spd);
  analogWrite(PWMB, spd);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);  
  digitalWrite(BIN2, LOW);   
}

void driveBackward(int spd) {
  //enableMotors();
  analogWrite(PWMA, spd);
  analogWrite(PWMB, spd);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void turnRight(int spd) {
  //enableMotors();
  analogWrite(PWMA, spd);
  analogWrite(PWMB, spd);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void turnLeft(int spd) {
  //enableMotors();
  analogWrite(PWMA, spd);
  analogWrite(PWMB, spd);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}


// ---------------- PIVOTING FUNCTION ----------------
// Pivot the robot about its own axis by a given angle (degrees).
// Positive degrees = left (CCW), negative = right (CW)
void pivotDegrees(int degrees) {
  // Reset encoder counts
  leftCount = 0;
  rightCount = 0;

  // Decide direction based on sign
  bool turnRightDirection = (degrees >= 0);

  // How many encoder counts do we need for this angle?
  long targetCounts = (COUNTS_PER_360 * (long)abs(degrees)) / 360;

  // Start turning in chosen direction
  if (turnRightDirection) {
    turnRight(turnSpeedPWM);
  } else {
    turnLeft(turnSpeedPWM);
  }

  // Spin until average encoder counts reach the target
  while ( ((abs(leftCount) + abs(rightCount)) / 2) < targetCounts ) {
    // keep turning
  }

  stopMotors();
}


/*====================================================================
===========================SONAR SENSOR ==============================
====================================================================*/

// ULTRASONIC PINS
#define FRONT_TRIG 26
#define FRONT_ECHO 28
#define RIGHT_TRIG 22
#define RIGHT_ECHO 24

// distance threshold to stop (cm)
const int stopDistance1 = 15; //wall for 45turn
const int stopDistance2 = 15; //before turning towards right tunnel
const int stopDistance3 = 15; //for whiteboard

// Right-wall minimum comfortable distance (cm)
const int rightWallMinDist = 4.5;   // tweak this value after testing

// ===== ULTRASONIC FUNCTION =====
  // Take one ultrasonic reading with basic sanity checks.
  // Returns distance in cm, or 999 if no valid reading.
long readUltrasonic(int trigPin, int echoPin) {
  // Ensure trigger is LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 10 µs HIGH pulse to trigger the sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse width (µs), timeout at 30 ms (~5 m)
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  // If no echo was received (timeout), treat as "very far"
  if (duration == 0) {
    return 999;  // sentinel for "no echo / very far"
  }

  // duration to distance conversion (float math, then back to long)
  float distance = duration * 0.0343f / 2.0f;

  // Basic sanity range for HC-SR04: ~2 cm to ~400 cm
  if (distance < 2.0f || distance > 400.0f) {
    return 999;  // out-of-range / likely noise
  }

  // Serial.println(distance);
  return (long)distance;
}

// ===== RIGHT WALL AVOIDANCE =====
void rightWallAvoid() {
  long dRight = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);

  // If we didn't get a valid reading (999 = "far/no echo"), just keep going
  if (dRight == 999 || dRight <= 0) {
    driveForward(forwardSpeedPWM);
    return;
  }

  // Debug print (optional)
  Serial.print("[Right wall] dRight = ");
  Serial.println(dRight);

  // Too close to the right wall -> make a small left correction
  if (dRight < rightWallMinDist) {
    // Small left "nudge": short left turn, then resume forward
    Serial.println("  -> Too close, nudging left");

    // brief left turn (blocking but short)
    pivotDegrees(10);
    delay(60);                  // tune this duration (ms) for your robot

  }
}


/*====================================================================
=========================== SERVO ==================================
====================================================================*/
#include <Servo.h>
Servo servo1;   // joint 1
Servo servo2;   // joint 2
Servo servo3;   // end-effector

// Angles for each position of the arm
const int S1_STATE1_ANGLE = 180;    // resting
const int S2_STATE1_ANGLE = 10;
const int S3_STATE1_ANGLE = 30;

const int S1_STATE2_ANGLE = 75;    // whiteboard
const int S2_STATE2_ANGLE = 135;
const int S3_STATE2_ANGLE = 30;

const int S1_STATE3_ANGLE = 20;     // drop pen
const int S2_STATE3_ANGLE = 180;
const int S3_STATE3_ANGLE = 75;

/*====================================================================
=========================== STATE MACHINE ============================
====================================================================*/
enum RobotState {
  STOP,
  STATE_360TURN,
  STATE_45TURN,
  RT,
  WHITEBOARD,
  EXIT,
  LT,
  DROP_PEN
};

RobotState currentState = STATE_360TURN;

// ---------------- SETUP ----------------
void setup() {

  Serial.begin(9600);
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  // pinMode(STBY, OUTPUT);

  // Encoder pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // Servo attaches
  servo1.attach(13);     // Servo 1 signal pin
  servo2.attach(12);    // Servo 2 signal pin
  servo3.attach(11);   // Servo 3 signal pin  
  
  // Initialize robot arm to resting position (if not already)
  servo1.write(S1_STATE1_ANGLE);
  servo2.write(S2_STATE1_ANGLE);
  Serial.println("Resting position");
  Serial.println("Opening claw");
  servo3.write(S3_STATE3_ANGLE);  // open claw
  delay(3000);
  Serial.println("Closing claw");
  servo3.write(S3_STATE1_ANGLE);   // close claw
  delay(3000);
  
  Serial.print("Init state = ");
  Serial.println((int)currentState); 
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // Simple state machine handling two states: 360TURN and 45TURN
  Serial.print("Loop state = ");
  Serial.println((int)currentState);
  switch (currentState) {
    // ===== STOP STATE ===== //
    case STOP:
      stopMotors();
      break;
    
    // ===== STATE 1 ===== //
    case STATE_360TURN:
      Serial.println("360 TURN");
      pivotDegrees(-360);
      delay(1000);
      currentState = STATE_45TURN; // !!! CHANGE FOR FINAL CODE
      break;

    // ===== STATE 3 ===== //
    case RT:
      Serial.println("RIGHT TUNNNEL");
      while (true) {
        long dFront = readUltrasonic(FRONT_TRIG, FRONT_ECHO);
        long dRight = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);
        
        if (dFront > 5 && dFront < 10) {    // break if close to whiteboard
          break;
        }
        
        if (dFront > 55) {
          if (dRight < 9) {      // right wall compensation BEFORE tunnel
            pivotDegrees(5);    
          } else {
            driveForward(forwardSpeedPWM);
          }
        } else if (dFront <= 55 && dFront > 45) {
          if (dRight < 4) {      // right wall compensation AT tunnel
            pivotDegrees(3);
          } else {
            driveForward(forwardSpeedPWM);
          }
        } else {
          if (dRight < 14) {      // right wall compensation AFTER tunnel
            pivotDegrees(3);
          } else {
            driveForward(forwardSpeedPWM);
          }
        }
      } // end while
      stopMotors();
      delay(3000);
      currentState = EXIT;
      break;

    // ===== STATE 5 ===== //    
    case LT:
      Serial.println("LEFT TUNNEL");
      delay(1000);
      while (true) {
        Serial.print("LT LOOPING @ ");
        long dFront = readUltrasonic(FRONT_TRIG, FRONT_ECHO);
        long dRight = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);
        Serial.println(dFront);
        // if (dFront > 5 && dFront < 30) {    // break if just outside yellow zone
        //   break;
        // }
        
        if (dFront > 55) {
          if (dRight < 11) {      // right wall compensation BEFORE tunnel
            pivotDegrees(4);    
          } else {
            driveForward(forwardSpeedPWM);
          }
        } else if (dFront <= 55 && dFront > 45) {
          if (dRight < 4) {      // right wall compensation AT tunnel
            pivotDegrees(3);
          } else {
            driveForward(forwardSpeedPWM);
          }
        } else {
          if (dRight < 14) {      // right wall compensation AFTER tunnel
            pivotDegrees(3);
          } else {
            driveForward(forwardSpeedPWM);
            if (dFront > 5 && dFront < 20) {    // break if just outside yellow zone
              break;
            }
          }
        } 
      } // end while
      stopMotors();
      delay(500);
      pivotDegrees(82);
      
      stopMotors();
      delay(2000);
      Serial.println("end of motion");
      currentState = DROP_PEN;
      break;

    // ===== STATE 7 ===== //
    case DROP_PEN:
      Serial.println("DROP_PEN");
      servo1.write(S1_STATE3_ANGLE);    // drop pen sequence
      servo2.write(S2_STATE3_ANGLE);
      delay(1000);
      servo3.write(S3_STATE3_ANGLE);    // open claw
      Serial.println("Dropping the pen...");

      delay(2500);
       
      servo1.write(S1_STATE1_ANGLE);  // reset to resting position
      servo2.write(S2_STATE1_ANGLE);
      servo3.write(S3_STATE1_ANGLE);
      currentState = STOP;
      break;

    // ===== STATE 2 ===== //
    case STATE_45TURN:
      Serial.println("45 TURN and DRIVE");
      pivotDegrees(-35);
      delay(300);

      //Drive forward until close to wall
      while (true) {
        long dRight = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);
        if (dRight > 3.5 && dRight < 13) {
          break;
        }
        driveForward(forwardSpeedPWM);
    }
      delay(500);

      // Left turn to restore straight trajectory
      pivotDegrees(50);
      delay(1000);
       
      //Drive straight until near wall
      while (true) {
        long dFront = readUltrasonic(FRONT_TRIG, FRONT_ECHO);
        long dRight = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);
        if (dFront > 3 && dFront < 14) {
          break;
        }
        if (dRight < 4.5) {    // right wall compensation
          pivotDegrees(4);
        } else {
          driveForward(forwardSpeedPWM);
        }
      }
      stopMotors();

      pivotDegrees(90);
      currentState = RT;
      break;
    
    
    // ===== STATE 4 ===== //
    case WHITEBOARD:
      Serial.println("WHITEBOARD");
      servo2.write(S2_STATE2_ANGLE);  // touch whiteboard
      
      delay(1000);
      servo1.write(S1_STATE2_ANGLE);
      servo3.write(S3_STATE2_ANGLE);
      Serial.println("Whiteboard position");

      delay(4000);

      servo1.write(S1_STATE1_ANGLE);  // return to resting position
      servo2.write(S2_STATE1_ANGLE);
      servo3.write(S3_STATE1_ANGLE);
      Serial.println("Resting position");

      delay(3000);
              
      currentState = STOP;
      break;
    
    // ===== STATE 5 ===== //
    case EXIT:
      // (last motion sequence)
      Serial.println("EXIT");
      // while (true) {    // backoff from whiteboard
      //   Serial.println("Backing off");
      //   long dFront = readUltrasonic(FRONT_TRIG, FRONT_ECHO);
      //   if (dFront > 18) {
      //     break;
      //   }
      //   driveBackward(90);
      // }
      Serial.println("pivoting 90");
      pivotDegrees(90);
      while (true) {  // drive to left wall
      Serial.println("Driving to left wall");
        long dFront = readUltrasonic(FRONT_TRIG, FRONT_ECHO);
        long dRight = readUltrasonic(RIGHT_TRIG, RIGHT_ECHO);
        if (dFront > 3 && dFront < 15) {
          break;
        }
        if (dRight < 4.5) {    // right wall compensation
          pivotDegrees(5);
        } else {
          driveForward(forwardSpeedPWM);
        }
      }
      Serial.println("Pivoting 90");
      pivotDegrees(90);
      currentState = LT;
      break;

  // ---- end of states -- //    
  // stopMotors();
  // delay(300);
  //     break;
  }
}
