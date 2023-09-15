#include <Arduino.h>
#include <ESP32Servo.h>
#include <SharpIR.h>
#include "Adafruit_VL53L0X.h"

// Données de calibration du servo
#define PIN_SERVO 33
#define ANGLE_MIN -60
#define ANGLE_MAX 60
#define ANGLE_OFFSET 90

Servo myservo;

// Données de calibration du capteur 1
#define PIN_CAPTEUR_1 A5

SharpIR sensor( SharpIR::GP2Y0A21YK0F, PIN_CAPTEUR_1);

// Données de calibration du capteur 2
Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void moveServo(int angle) {
  angle = angle > ANGLE_MAX ? ANGLE_MAX : angle;
  angle = angle < ANGLE_MIN ? ANGLE_MIN : angle;
  myservo.write(angle + ANGLE_OFFSET);
}

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  pinMode(PIN_SERVO, OUTPUT);
  pinMode(PIN_CAPTEUR_1, INPUT);

  while (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }

  lox.startRangeContinuous();
}

void loop() {
  if (!myservo.attached()) {
		myservo.setPeriodHertz(50); // standard 50 hz servo
		myservo.attach(33, 900, 2100); // Attach the servo after it has been detatched
	}

  moveServo(0);
  delay(1000);
  
  while(1) {
    if (lox.isRangeComplete()) {
    Serial.print("Distance in mm: ");
    Serial.println(lox.readRange());
  } 
    delay(100);
  }
}

