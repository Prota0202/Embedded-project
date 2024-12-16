#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include <Servo.h>

// Moteurs
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorLEFT = AFMS.getMotor(3);
Adafruit_DCMotor *myMotorRIGHT = AFMS.getMotor(4);

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
int spoint = 90; // Pos du servo

// PID
float kp = 1.5, ki = 0.0, kd = 0.5;
float previousError = 0, integral = 0;
int baseSpeed = 50;

// detect d'obstacles
const int obstacleThreshold = 15;

int L = 0, R = 0, distance = 0;

void setup() {
    Serial.begin(9600);
    AFMS.begin();

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    servo.attach(SERVO_PIN);
    servo.write(spoint); // Position initiale (centrale)
    delay(500);
}

void loop() {
    distance = DistanceSensor.ping_cm();
    if (distance > 0 && distance <= obstacleThreshold) {
        Obstacle();
        return;
    }

    int valueLeft = analogRead(IR_LEFT);
    int valueRight = analogRead(IR_RIGHT);

    int error = valueRight - valueLeft;
    float correction = computePID(error);

    int speedMotorLEFT = constrain(baseSpeed - correction, 0, 50);
    int speedMotorRIGHT = constrain(baseSpeed + correction, 0, 50);

    myMotorLEFT->setSpeed(speedMotorLEFT);
    myMotorLEFT->run(FORWARD);

    myMotorRIGHT->setSpeed(speedMotorRIGHT);
    myMotorRIGHT->run(FORWARD);

    Serial.print("Distance: "); Serial.print(distance);
    Serial.print(" Error: "); Serial.print(error);
    Serial.print(" Correction: "); Serial.println(correction);

    delay(10); 
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

    servo.write(45); // vers la gauche
    delay(800);
    L = DistanceSensor.ping_cm();

    servo.write(135); // vers la droite
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
    myMotorLEFT->run(BACKWARD);
    myMotorRIGHT->run(BACKWARD);
}

void backward() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(FORWARD);
    myMotorRIGHT->run(FORWARD);
}

void right() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(BACKWARD);
    myMotorRIGHT->run(FORWARD);
}

void left() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(FORWARD);
    myMotorRIGHT->run(BACKWARD);
}

void Stop() {
    myMotorLEFT->setSpeed(0);
    myMotorRIGHT->setSpeed(0);
    myMotorLEFT->run(RELEASE);
    myMotorRIGHT->run(RELEASE);
}