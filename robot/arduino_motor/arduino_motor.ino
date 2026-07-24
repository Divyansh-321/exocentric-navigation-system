#include <Motor_Shield.h>
#include <SoftwareSerial.h>

DCMotor Amotor(1);
DCMotor Bmotor(2);

SoftwareSerial espSerial(2, 3);

void setup() {
  espSerial.begin(9600);
  Stop();
}

void loop() {
  if (espSerial.available()) {
    char command = espSerial.read();

    if (command == 'w' || command == 'W') {
      Forward(140);
    }
    else if (command == 's' || command == 'S') {
      Backward(120);
    }
    else if (command == 'a' || command == 'A') {
      Left();
    }
    else if (command == 'd' || command == 'D') {
      Right();
    }
    else if (command == 'x' || command == 'X') {
      Stop();
    }

    while (espSerial.available() > 0) {
      espSerial.read(); 
    }
  } 
} 

void Forward(unsigned char Speed) {
  Amotor.run(FORWARD);
  Bmotor.run(FORWARD);
  Amotor.setSpeed(Speed);
  Bmotor.setSpeed(Speed);
  delay(200);
  Stop();
}

void Backward(unsigned char Speed) {
  Amotor.run(BACKWARD);
  Bmotor.run(BACKWARD);
  Amotor.setSpeed(Speed);
  Bmotor.setSpeed(Speed);
  delay(200);
  Stop();
}

void Right() {
  Amotor.run(FORWARD);
  Bmotor.run(BACKWARD);
  Amotor.setSpeed(132);
  Bmotor.setSpeed(132);
  delay(160);
  Stop();
}

void Left() {
  Amotor.run(BACKWARD);
  Bmotor.run(FORWARD);
  Amotor.setSpeed(132);
  Bmotor.setSpeed(132);
  delay(160);
  Stop();
}

void Stop() {
  Amotor.run(RELEASE);
  Bmotor.run(RELEASE);
  Amotor.setSpeed(0);
  Bmotor.setSpeed(0);
}
