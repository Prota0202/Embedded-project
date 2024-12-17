#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include <Servo.h>

// Moteurs
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorLEFT = AFMS.getMotor(3);
Adafruit_DCMotor *myMotorRIGHT = AFMS.getMotor(4);


// Variables globales pour la distance et les mesures de l'obstacle
int distance = 0; // Distance mesurée par le capteur ultrason
int L = 0;        // Distance mesurée à gauche lors du scan d'obstacle
int R = 0;        // Distance mesurée à droite lors du scan d'obstacle

// Capteurs IR
#define IR_LEFT A0
#define IR_RIGHT A1

// Capteur ultrason
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 200
NewPing DistanceSensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Servo
Servo servo;
#define SERVO_PIN 3
int spoint = 90; // Position du servo

// PID
float kp = 1.5, ki = 0.0, kd = 0.5;
float previousError = 0, integral = 0;
int baseSpeed = 60;

// Suivi de distance
volatile int ticksGauche = 0; // Ticks pour le moteur gauche
volatile int ticksDroite = 0; // Ticks pour le moteur droit
float distanceParcourue = 0;
float distanceTotale = 1200; // Distance totale avant arrêt (en mm)
float circonferenceRoue = 30 * PI; // Circonférence de la roue
int ticksParRotation = 20; // Ticks par rotation (à ajuster)

// Détection d'obstacles
const int obstacleThreshold = 15;

int buttonState = 0;

// Encodeurs
#define ENCODER_LEFT_A 2
#define ENCODER_RIGHT_A 3

void setup() {
    Serial.begin(9600);
    AFMS.begin();

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);

    // Attach interruptions pour compter les ticks des encodeurs
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), countTicksGauche, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), countTicksDroite, RISING);

    servo.attach(SERVO_PIN);
    servo.write(spoint); // Position initiale du servo
    delay(500);
}

void loop() {
    // Vérification des obstacles
    distance = DistanceSensor.ping_cm();
    if (distance > 0 && distance <= obstacleThreshold) {
        Obstacle();
        return;
    }

    // Mise à jour de la distance parcourue
    updateDistance();

    // Si la distance totale est atteinte, arrêter le robot
    if (distanceParcourue >= distanceTotale) {
        Stop();
        Serial.println("Arrêt final atteint : distance parcourue dépassée.");
        while (1); // Arrêt complet du programme
    }

    // Lecture des capteurs IR
    int valueLeft = analogRead(IR_LEFT);
    int valueRight = analogRead(IR_RIGHT);

    // Calcul de l'erreur pour le PID
    int error = valueRight - valueLeft;
    float correction = computePID(error);

    // Calcul des vitesses des moteurs en fonction de la correction PID
    int speedMotorLEFT = constrain(baseSpeed - correction, 0, 50);
    int speedMotorRIGHT = constrain(baseSpeed + correction, 0, 50);

    // Appliquer les vitesses calculées
    myMotorLEFT->setSpeed(speedMotorLEFT);
    myMotorLEFT->run(FORWARD);

    myMotorRIGHT->setSpeed(speedMotorRIGHT);
    myMotorRIGHT->run(FORWARD);

    // Debugging : afficher les données importantes
    Serial.print("Distance parcourue : ");
    Serial.println(distanceParcourue);
    Serial.print("Correction PID : ");
    Serial.println(correction);
    delay(10); // Pause pour lisibilité
}

float computePID(float error) {
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
}

void Obstacle() {
    Stop();
    backward();
    delay(500);
    Stop();

    servo.write(45); // Scanner vers la gauche
    delay(800);
    L = DistanceSensor.ping_cm();

    servo.write(135); // Scanner vers la droite
    delay(800);
    R = DistanceSensor.ping_cm();

    servo.write(spoint); // Retour au centre
    delay(500);

    if (L < R) {
        right();
        delay(500);
        Stop();
        delay(200);
    } else if (L > R) {
        left();
        delay(500);
        Stop();
        delay(200);
    } else {
        forward();
    }
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

void updateDistance() {
    noInterrupts(); // Empêche les interruptions durant la lecture
    int ticksGaucheCopy = ticksGauche;
    int ticksDroiteCopy = ticksDroite;
    interrupts(); // Réactive les interruptions

    // Moyenne des ticks pour estimer la distance parcourue
    float moyenneTicks = (ticksGaucheCopy + ticksDroiteCopy) / 2.0;
    distanceParcourue += moyenneTicks * (circonferenceRoue / ticksParRotation);

    // Réinitialiser les ticks pour la prochaine boucle
    ticksGauche = 0;
    ticksDroite = 0;
}

void countTicksGauche() {
    ticksGauche++;
}

void countTicksDroite() {
    ticksDroite++;
}