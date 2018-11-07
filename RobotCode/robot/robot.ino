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
    Motor(byte, byte);
    void translate(short);
  
  private:
    byte pinA, pinB;
    void forwards(byte);
    void backwards(byte);
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

void Motor::translate(short value) {
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
    void translate(float x, float y, byte value);
    void rotate(short value);
    void stopMotion();

   private:
};

void Robot::translate(float x, float y, byte value) {
  float angle = atan2(y, x);
  Serial.println(angle);
  motorA.translate(value * cos(angle + pi * 0.25F));
  motorB.translate(-value * sin(angle + pi * 0.25F));
  motorC.translate(value * sin(angle + pi * 0.25F));
  motorD.translate(-value * cos(angle + pi * 0.25F));
};

void Robot::rotate(short value) {
  motorA.translate(value);
  motorB.translate(value);
  motorC.translate(value);
  motorD.translate(value);
};

void Robot::stopMotion() {
  motorA.translate(0);
  motorB.translate(0);
  motorC.translate(0);
  motorD.translate(0);
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

Robot robot;

/* -----------------------
 * |   START AND LOOP    |
 * -----------------------
 */

void setup() {
  Serial.begin(9600);
  Serial.println("    -----------------------    ");
  Serial.println("<-~-| ARDUINO HAS STARTED |-~->");
  Serial.println("    -----------------------    ");
  Serial.println("  (•_•) ( •_•)>⌐■-■ (⌐■_■)  ");
  state = START;
}

void loop() {
  switch(state) {
    case START:
      //robot.translate(0.0F, 0.0F, 0);
      robot.rotate(-70);
      delay(4000);
      state = END;
    break;
    case ANALYSIS:
      
    break;
    case DECISION:
      
    break;
    case PENALTY:
      
    break;
    case END:
      robot.stopMotion();
    break;
  }
}
