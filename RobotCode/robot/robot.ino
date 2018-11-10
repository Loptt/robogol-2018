#include <SPI.h>  
#include <Pixy.h>
#include <Wire.h>

bool penaltyDefence = false; //True if the robot is defending a penalty
bool penaltyThrow = false; //True if the robot is throwing a penalty

/* ------------------
 * |   CONSTANTS    |
 * ------------------
 */

const float pi = 3.1415926535897932384626433832795028841971F;
const short ballRange = 10; //Sets the range where the robot is facing the front

/* --------------------------
 * |   MOTOR AND COMPASS    |
 * --------------------------
 */

/*
 * Contains all the functions that move a single wheel as desired
 */
class Motor {
  public:
    Motor(byte pinA, byte pinB);
    void turn(short velocity);
  
  private:
    byte pinA, pinB;
    void forwards(byte velocity);
    void backwards(byte velocity);
};

Motor::Motor(byte pinA, byte pinB) {
  this->pinA = pinA;
  this->pinB = pinB;
};

/*
 * forwards: Moves a motor forwards by a certain velocity
 * velocity: Motor strength <0 to 255>
 */
void Motor::forwards(byte velocity) {
  analogWrite(this->pinA, velocity);
  analogWrite(this->pinB, 0);
};

/*
 * backwards: Moves a motor backwards by a certain velocity
 * velocity: Motor strength <0 to 255>
 */
void Motor::backwards(byte velocity) {
  analogWrite(this->pinA, 0);
  analogWrite(this->pinB, velocity);
};

/*
 * turn: Turns a motor safely forwards or backwards
 * velocity: Motor strength <-255 to 255>
 */
void Motor::turn(short velocity) {
  if(velocity < 0) {
    this->backwards((byte)(-velocity));
  } else {
    this->forwards((byte)velocity);
  }
};

short compassDirection = 0;
short relativeDirection = 0;
short initialDirection = 0;

void ReadCompassSensor(){
  Wire.beginTransmission(0x01);
  Wire.write(0x44);
  Wire.endTransmission();
  Wire.requestFrom(0x01, 2); 
   while(Wire.available() < 2);
   byte lowbyte = Wire.read();  
   byte highbyte = Wire.read();
   compassDirection = word(highbyte, lowbyte); 
}

/* ------------------------
 * |   ROBOT FUNCTIONS    |
 * ------------------------
 */

Motor motorA(12, 11); //BACK LEFT
Motor motorB(10, 9); //FRONT LEFT
Motor motorC(8, 7); //BACK RIGHT
Motor motorD(6, 5); //FRONT RIGHT

/*
 * Contains all the functions that control the robot's motion
 */
class Robot {
  public:
    void moveTo(float x, float y, byte velocity);
    void rotate(short velocity);
    void arcLeft(short velocity, float curvature);
    void arcRight(short velocity, float curvature);
    void stopMotion();
    void align(byte velocity);
};

/*
 * moveTo: Moves the robot in a certain direction
 * x: X component of the movement vector <0 to 1>
 * y: Y component of the movement vector <0 to 1>
 * velocity: Motor strength <0 to 255>
 */
void Robot::moveTo(float x, float y, byte velocity) {
  float angle = atan2(y, x);
  motorA.turn(velocity * cos(angle + pi * 0.25F));
  motorB.turn(-velocity * sin(angle + pi * 0.25F));
  motorC.turn(velocity * sin(angle + pi * 0.25F));
  motorD.turn(-velocity * cos(angle + pi * 0.25F));
};

/*
 * rotate: Rotates the robot according to the right hand rule
 * velocity: Motor strength <-255 to 255>
 */
void Robot::rotate(short velocity) {
  motorA.turn(velocity);
  motorB.turn(velocity);
  motorC.turn(velocity);
  motorD.turn(velocity);
};

/*
 * arcLeft: Moves and rotates the robot leftwards if positive
 * velocity: Motor strength <-255 to 255>
 * curvature: How much the robot turns while moving <0 to 1>
 */
void Robot::arcLeft(short velocity, float curvature) {
  motorA.turn(-velocity * (1.0F - curvature));
  motorB.turn(-velocity * (1.0F - curvature));
  motorC.turn(velocity);
  motorD.turn(velocity);
}

/*
 * arcRight: Moves and rotates the robot rightward if positive
 * velocity: Motor strength <-255 to 255>
 * curvature: How much the robot turns while moving <0 to 1>
 */
void Robot::arcRight(short velocity, float curvature) {
  motorA.turn(-velocity);
  motorB.turn(-velocity);
  motorC.turn(velocity * (1.0F - curvature));
  motorD.turn(velocity * (1.0F - curvature));
}

/*
 * stopMotion: Stops the robot from moving and rotating
 */
void Robot::stopMotion() {
  motorA.turn(0);
  motorB.turn(0);
  motorC.turn(0);
  motorD.turn(0);
}

/*
 * align: Aligns the robot towards its relative north
 * velocity: Motor strength <0 to 255>
 */
void Robot::align(byte velocity) {
  while(relativeDirection > ballRange && relativeDirection < (360 - ballRange)) {
    ReadCompassSensor();
    relativeDirection = (((compassDirection - initialDirection) % 360) + 360) % 360;
    if(relativeDirection < 180) {
      this->rotate(-velocity);
    } else {
      this->rotate(velocity);
    }
  }
  this->stopMotion();
}

/* ------------------
 * |   VARIABLES    |
 * ------------------
 */

enum mode{START, ANALYSIS, DECISION, DEFENCE, PENALTY}; //Finite state machine
enum mode state;

Pixy pixy;
Robot robot;

uint16_t blocks;
int pixyIndex = -1;
int pixyArea = 0;
short ballX = 0; //X coordinate from -160 to 160
short ballY = 0; //Y coordinate from -100 to 100
short lastSeenTicks = 0;
short currentTime = 0;
short timeout = 20; //Number of ticks before the ball is considered lost
bool lastSeenRight = false; //Direction in which the robot will turn if the ball is lost
bool hasBall = false; //True if the robot is carrying the ball

void setup() {
  Serial.begin(9600);
  Serial.println("    ------------------------    ");
  Serial.println("<-~-| INITIALIZING ARDUINO |-~->");
  Serial.println("    ------------------------    ");
  Serial.println("");
  state = START;
  pixy.init();
  Serial.println("Pixy:\tstarted");
  Wire.begin();
  Wire.beginTransmission(0x01);
  Wire.write(0x00);
  Wire.endTransmission();
  while(Wire.available() > 0)
     Wire.read();
  Serial.println("Compass:\tstarted");
}

/*  ------------------
 * ~|   MAIN LOOP    |~
 *  ------------------
 * 
 * Valid commands:
 * robot.moveTo(x, y, velocity)
 * robot.rotate(velocity)
 * robot.arcLeft(velocity, curvature)
 * robot.arcRight(velocity, curvature)
 * robot.stopMotion()
 * robot.align(velocity)
 */

void loop() {
  blocks = pixy.getBlocks();
  ReadCompassSensor();
  relativeDirection = (((compassDirection - initialDirection) % 360) + 360) % 360;
  currentTime = millis();
  switch(state) {
    case START:
      robot.stopMotion();
      initialDirection = compassDirection;
      state = ANALYSIS;
    break;
    case ANALYSIS:
       pixyIndex = -1;
       pixyArea = 0;
       
       //Controls the ball timeout
       if(lastSeenTicks < timeout) {
        lastSeenTicks++;
       } else {
        hasBall = false;
       }
       
       //Locate the ball
       for(int i = 0; i < blocks; i++) {
        int area = pixy.blocks[i].width * pixy.blocks[i].height;
        if(area > pixyArea) {
          pixyArea = area;
          pixyIndex = i;
          ballX = 160 - (short)pixy.blocks[i].x;
          ballY = 100 - (short)pixy.blocks[i].y;
        }
       }

       //If the ball is found
       if(pixyIndex != -1) {
        lastSeenTicks = 0;
        if(ballX < 0) {
          lastSeenRight = false;
        } else {
          lastSeenRight = true;
        }
        
        //Check if the robot has the ball
        if(ballY < -80) {
          hasBall = true;
        } else {
          hasBall = false;
        }
       }

       if(penaltyDefence) {
        state = DEFENCE;
       } else if (penaltyThrow) {
        state = PENALTY;
       } else {
        state = DECISION;
       }       
    break;
    case DECISION:
       if(pixyIndex == -1) {
        //If the ball is lost
        if(lastSeenTicks >= timeout) {
          if(lastSeenRight) {
            robot.rotate(-130);
          } else {
            robot.rotate(130);
          }
        }
       } else {
         //If the ball is found
         if(ballX < 0) {
            robot.arcLeft(250, -ballX / 200.0F);
          } else {
            robot.arcRight(250, ballX / 200.0F);
          }
       }

       //if the ball has been acquired, move it to the goal
       if(hasBall) {
        if(relativeDirection > ballRange * 3 && relativeDirection < (360 - ballRange * 3)) {
          robot.align(255);
        }
        robot.moveTo(0.0F, 1.0F, 255);
       }
       
       state = ANALYSIS;
    break;
    case DEFENCE:
      if(currentTime % 1000 == 0) {
        robot.align(255);
      }
      if(pixyIndex == -1) {
        if(lastSeenTicks >= timeout) {
          if(lastSeenRight) {
            robot.moveTo(1.0F, 0.0F, 150);
          } else {
            robot.moveTo(-1.0F, 0.0F, 150);
          }
        }
      } else {
        robot.moveTo(ballX, 0.0F, 255);
      }
      state = ANALYSIS;
    break;
    case PENALTY:
       robot.align(150);
       robot.moveTo(0.0F, -1.0F, 80);
       delay(100);
       robot.arcLeft(-80, 0.6F);
       delay(800);
       robot.stopMotion();
       delay(1000);
       robot.moveTo(-1.0F, 0.0F, 120);
       delay(600);
       robot.stopMotion();
       delay(1000);
       robot.moveTo(0.0F, 1.0F, 255);
       delay(500);
       robot.stopMotion();
       delay(20000);
       penaltyThrow = false;
       state = ANALYSIS;
    break;
  }
}
