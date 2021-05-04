#include <Servo.h>
Servo myServo;
int servoPin = 6;
int servoPos = 90;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myServo.attach(servoPin);
myServo.write(servoPos);
}

void loop() {
  // put your main code here, to run repeatedly:
if (Serial.available() >0){
  servoPos = Serial.read();
  myServo.write(servoPos);
  delay(20);
  }
}
