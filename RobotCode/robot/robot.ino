#include <SPI.h>  
#include <Pixy.h>
#include <Wire.h>

/* ------------------
 * |   CONSTANTS    |
 * ------------------
 */

const float pi = 3.1415926535897932384626433832795028841971F;

/* ------------------------------
 * |   CLASSES AND FUNCTIONS    |
 * ------------------------------
 */

class Motor {
  public:
    Motor(byte pinA, byte pinB);
    void turn(short value);
  
  private:
    byte pinA, pinB;
    void forwards(byte value);
    void backwards(byte value);
};

Motor::Motor(byte pinA, byte pinB) {
  this->pinA = pinA;
  this->pinB = pinB;
};

void Motor::forwards(byte value) {
  analogWrite(this->pinA, value);
  analogWrite(this->pinB, 0);
};

void Motor::backwards(byte value) {
  analogWrite(this->pinA, 0);
  analogWrite(this->pinB, value);
};

void Motor::turn(short value) {
  if(value < 0) {
    this->backwards((byte)(-value));
  } else {
    this->forwards((byte)value);
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

Motor motorA(12, 11); //BACK LEFT
Motor motorB(10, 9); //FRONT LEFT
Motor motorC(8, 7); //BACK RIGHT
Motor motorD(6, 5); //FRONT RIGHT

class Robot {
  public:
    void moveTo(float x, float y, byte value);
    void rotate(short value);
    void arcLeft(short value, float curvature);
    void arcRight(short value, float curvature);
    void stopMotion();
    void align(byte value);

   private:
};

/*
 * moveTo: Moves the robot in a certain direction
 * x: X component of the movement vector <0 to 1>
 * y: Y component of the movement vector <0 to 1>
 * value: Motor strength <0 to 255>
 */
void Robot::moveTo(float x, float y, byte value) {
  float angle = atan2(y, x);
  motorA.turn(value * cos(angle + pi * 0.25F));
  motorB.turn(-value * sin(angle + pi * 0.25F));
  motorC.turn(value * sin(angle + pi * 0.25F));
  motorD.turn(-value * cos(angle + pi * 0.25F));
};

/*
 * rotate: Rotates the robot according to the right hand rule
 * value: Motor strength <-255 to 255>
 */
void Robot::rotate(short value) {
  motorA.turn(value);
  motorB.turn(value);
  motorC.turn(value);
  motorD.turn(value);
};

/*
 * arcLeft: Moves and rotates the robot leftwards if positive
 * value: Motor strength <-255 to 255>
 * curvature: How much the robot turns while moving <0 to 1>
 */
void Robot::arcLeft(short value, float curvature) {
  motorA.turn(-value * (1.0F - curvature));
  motorB.turn(-value * (1.0F - curvature));
  motorC.turn(value);
  motorD.turn(value);
}

/*
 * arcRight: Moves and rotates the robot rightward if positive
 * value: Motor strength <-255 to 255>
 * curvature: How much the robot turns while moving <0 to 1>
 */
void Robot::arcRight(short value, float curvature) {
  motorA.turn(-value);
  motorB.turn(-value);
  motorC.turn(value * (1.0F - curvature));
  motorD.turn(value * (1.0F - curvature));
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
 * align: Aligns the robot towards it's relative north
 * value: Motor strength <0 to 255>
 */
void Robot::align(byte value) {
  while(relativeDirection > 10 && relativeDirection < 350) {
    ReadCompassSensor();
    relativeDirection = (((compassDirection - initialDirection) % 360) + 360) % 360;
    if(relativeDirection < 180) {
      this->rotate(-value);
    } else {
      this->rotate(value);
    }
  }
  this->stopMotion();
}

class Camera {
  public:
    Camera();
  
  private:
};

/* ------------------
 * |   VARIABLES    |
 * ------------------
 */

enum mode{START, ANALYSIS, DECISION, PENALTY, END};
enum mode state;

Pixy pixy;
Robot robot;

uint16_t blocks;
int pixyIndex = -1;
int pixyArea = 0;
short xPos = 0;
short yPos = 0;
short lastSeenTicks = 0;
short maxSeenTicks = 20;
short velocity = 150;
bool lastSeen = false;

void setup() {
  Serial.begin(9600);
  Serial.println("    ------------------------    ");
  Serial.println("<-~-| INITIALIZING ARDUINO |-~->");
  Serial.println("    ------------------------    ");
  Serial.println("");
  state = START;
  pixy.init();
  Serial.println("Pixy...\tstarted");
  Wire.begin();
  Wire.beginTransmission(0x01);
  Wire.write(0x00);
  Wire.endTransmission();
  while(Wire.available() > 0)
     Wire.read();
  Serial.println("Compass...\tstarted");
}

/* ------------------
 * |   MAIN LOOP    |
 * ------------------
 * 
 * Valid commands:
 * robot.moveTo(x, y, value)
 * robot.rotate(value)
 * robot.arcLeft(value, curvature)
 * robot.arcRight(value, curvature)
 * robot.stopMotion()
 * robot.align(value)
 */

void loop() {
  blocks = pixy.getBlocks();
  ReadCompassSensor();
  relativeDirection = (((compassDirection - initialDirection) % 360) + 360) % 360;
  switch(state) {
    case START:
      robot.stopMotion();
      initialDirection = compassDirection;
      state = ANALYSIS;
    break;
    case ANALYSIS:
       pixyIndex = -1;
       pixyArea = 0;
       if(lastSeenTicks <= maxSeenTicks) {
        lastSeenTicks++;
       }
       for(int i = 0; i < blocks; i++) {
        int area = pixy.blocks[i].width * pixy.blocks[i].height;
        if(area > pixyArea) {
          pixyArea = area;
          pixyIndex = i;
          xPos = (short)pixy.blocks[i].x - 160;
          yPos = 100 - (short)pixy.blocks[i].y;
        }
       }
       if(pixyIndex != -1) {
        lastSeenTicks = 0;
        if(xPos < 0) {
          lastSeen = false;
        } else {
          lastSeen = true;
        }
       }
       state = DECISION;
    break;
    case DECISION:
      /*
       * Tell the robot how to move
       */
       if(pixyIndex == -1) {
        if(lastSeenTicks >= maxSeenTicks) {
          if(lastSeen) {
            robot.rotate(-velocity);
          } else {
            robot.rotate(velocity);
          }
        }
       } else {
         if(xPos < 0) {
            robot.arcLeft(velocity, -xPos / 200.0F);
          } else {
            robot.arcRight(velocity, xPos / 200.0F);
          }
          //robot.moveTo(xPos, 160.0F, velocity);
       }
       state = ANALYSIS;
    break;
    case PENALTY:
      /*
       * Make a penalty
       */
    break;
    case END:
      robot.stopMotion();
    break;
  }
}
