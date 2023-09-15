#include <Arduino.h>
#include <ESP32Servo.h>

#define PIN_SERVO A5

Servo myservo;

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
}

void loop() {
  if (!myservo.attached()) {
		myservo.setPeriodHertz(50); // standard 50 hz servo
		myservo.attach(33, 900, 2100); // Attach the servo after it has been detatched
	}


  while(1) {
    myservo.write(30);
    delay(1000);
  }
}

