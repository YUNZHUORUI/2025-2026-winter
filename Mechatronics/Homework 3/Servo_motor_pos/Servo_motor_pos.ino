#include <Servo.h>
Servo servoMotor;
int potPos;

void setup() {
  servoMotor.attach(9);
}
void loop () {
  potPos = map(analogRead(A0), 0, 1023, 0, 180);
  servoMotor.write(potPos);
  delay(15);
}