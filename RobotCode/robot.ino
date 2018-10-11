
class Motor
{
public:
  Motor(byte, byte, byte byte);

private:
  byte pinA, pinB;
  byte encA, encB;
};

Motor::Motor(byte pinA, byte pinB, byte encA, byte encB)
{
  this->pinA = pinA;
  this->pinB = pinB;
  this->encA = encA;
  this->encB = encB;
}

class Camera
{
public:
  Camera();

private:
}