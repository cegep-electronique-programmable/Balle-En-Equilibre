#include <Arduino.h>
#include <ESP32Servo.h>
#include <SharpIR.h>
#include "Adafruit_VL53L0X.h"

// Données du régulateur PID
#define KP 0.1
#define KI 0.01
#define KD 0.00
#define DT 0.01
#define POSITION_CENTRALE 280
unsigned long previousMillisControlLoop = 0;

// Données de calibration du servo
#define PIN_SERVO 33
#define ANGLE_MIN -50
#define ANGLE_MAX 50
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

  float position_balle = 0;
  float x[2] = {0, 0}; // raw position
  float y[2] = {0, 0}; // filtered position

  float consigne = POSITION_CENTRALE;
  float dt = DT;
  float erreur = 0;
  float erreur_somme = 0;
  float erreur_delta = 0;
  float erreur_precedente = 0;
  float position_moteur = 0;

  while(1) {
    

    /*
    if (nouvelle_position_balle < 1000) {
      position_balle = 0.25 * nouvelle_position_balle + 0.75 * position_balle;
    }
    */


    // Boucle de controle de la vitesse horizontale
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillisControlLoop >= dt*1000)
    {
      previousMillisControlLoop = currentMillis;

      // Lecture de la position de la balle
      x[1] = 1000;
      while (x[1] > 900) { // On recommence la lecture si la valeur est trop grande
        while (!lox.isRangeComplete());
        x[1] = lox.readRange();
      }
      
      Serial.print(x[1]);
      Serial.print(", ");

      // Filtre passe-bas
      y[1] = 0.228 * y[0] + 0.85 * x[1] + 0.385 * x[0]; 
      position_balle = x[1];
      Serial.print(position_balle);
      Serial.print(", ");

      x[0] = x[1];
      y[0] = y[1];

      // anti windup
      if (position_balle < 40 || position_balle > 500) {
       // erreur_somme = 0;
      }


      // Calcul de la consigne
      erreur = consigne - position_balle;
      Serial.print(erreur);
      Serial.print(", ");

      // anti windup
      if (position_moteur <= ANGLE_MAX && position_moteur >= ANGLE_MIN) {
        erreur_somme = erreur_somme + erreur*dt;
      }

      erreur_delta = (erreur - erreur_precedente) / dt;
      /*
      if (abs(erreur_delta) < 100) {
        erreur_delta = 0;
      }
      */

      float P = KP * erreur;
      float I = KI * erreur_somme;
      float D = KD * erreur_delta;
      
      
      Serial.print(P);
      Serial.print(", ");
      
      Serial.print(I);
      Serial.print(", ");
      
      Serial.print(D);
      Serial.print(", ");
      
      position_moteur = P + I + D;

      erreur_precedente = erreur;

      // Saturation
      position_moteur = position_moteur > ANGLE_MAX ? ANGLE_MAX : position_moteur;
      position_moteur = position_moteur < ANGLE_MIN ? ANGLE_MIN : position_moteur;
      Serial.println(position_moteur);

      moveServo(position_moteur);
    }
  } 
    
}

