#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include <Servo.h>

// Moteurs
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorLEFT = AFMS.getMotor(1);
Adafruit_DCMotor *myMotorRIGHT = AFMS.getMotor(2);

// Capteur ultrason
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 200
NewPing DistanceSensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Servo
Servo servo;
#define SERVO_PIN 4
int spoint = 90; // Position centrale du servo

// Détection d'obstacles
const int obstacleThreshold = 15; // Seuil de distance pour éviter l'obstacle
int distance = 0;                 // Distance mesurée
int L = 0, R = 0;                 // Distances mesurées à gauche et droite

// Vitesse de base
int baseSpeed = 50;

void setup() {
    Serial.begin(9600);

    // Initialisation du Motor Shield
    if (!AFMS.begin()) {
        Serial.println("Motor Shield non détecté !");
        while (1);
    }

    // Servo
    servo.attach(SERVO_PIN);
    servo.write(spoint); // Position centrale
    delay(500);

    Serial.println("Démarrage du robot...");
}

void loop() {
    // Mesurer la distance avec l'ultrason
    distance = DistanceSensor.ping_cm();

    if (distance > 0 && distance <= obstacleThreshold) {
        Serial.println("Obstacle détecté !");
        Stop();
        delay(1000); // Pause d'une seconde
        Obstacle();  // Éviter l'obstacle
    } else {
        // Avancer tout droit
        backward();
    }

    // Debugging
    Serial.print("Distance mesurée : ");
    Serial.println(distance);

    delay(10); // Délai pour lisibilité
}

void forward() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(FORWARD);
    myMotorRIGHT->run(FORWARD);
}

void backward() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(BACKWARD);
    myMotorRIGHT->run(BACKWARD);
}

void right() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(FORWARD);
    myMotorRIGHT->run(BACKWARD);
}

void left() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(BACKWARD);
    myMotorRIGHT->run(FORWARD);
}

void Stop() {
    myMotorLEFT->setSpeed(0);
    myMotorRIGHT->setSpeed(0);
    myMotorLEFT->run(RELEASE);
    myMotorRIGHT->run(RELEASE);
}

// Éviter l'obstacle en scannant à gauche et à droite
void Obstacle() {
    servo.write(45); // Scanner à gauche
    delay(800);
    L = DistanceSensor.ping_cm();

    servo.write(135); // Scanner à droite
    delay(800);
    R = DistanceSensor.ping_cm();

    servo.write(spoint); // Retour au centre
    delay(500);

    // Décision : tourner vers la direction avec le plus d'espace libre
    if (L > R) {
        Serial.println("Obstacle à droite, tourner à gauche.");
        left();
        delay((500));
    } else {
        Serial.println("Obstacle à gauche, tourner à droite.");
        right();
        delay(500);
    }

    // Reprendre la marche avant
    forward();
}
