#include <SPI.h>  
#include <Pixy.h>

/* ------------------
 * |   CONSTANTS    |
 * ------------------
 */

const float pi = 3.1415926535897932384626433832795028841971F;

/* ----------------
 * |   CLASSES    |
 * ----------------
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

void setup() {
  Serial.begin(9600);
  Serial.println("    -----------------------    ");
  Serial.println("<-~-| ARDUINO HAS STARTED |-~->");
  Serial.println("    -----------------------    ");
  Serial.println("  (•_•) ( •_•)>⌐■-■ (⌐■_■)  ");
  state = START;
  pixy.init();
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
 */

void loop() {
  blocks = pixy.getBlocks();
  switch(state) {
    case START:
      robot.stopMotion();
      /*
       * Obtain initial angle
       */
      state = ANALYSIS;
    break;
    case ANALYSIS:
      /*
       * Calculate the ball and take a decision
       * Move to the decision case
       * Or move to penalty case
       */
    break;
    case DECISION:
      /*
       * Tell the robot how to move
       */
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
