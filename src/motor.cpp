// motor driver
#include "rover.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS;
Adafruit_DCMotor *leftMotor;
Adafruit_DCMotor *rightMotor;

// set up the drive system
void setupDrive() {
    AFMS = Adafruit_MotorShield();
    leftMotor = AFMS.getMotor(3);
    rightMotor = AFMS.getMotor(4);
    AFMS.begin();
}

void setNewSpeed(uint8_t speed) {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    leftMotor->setSpeed(speed);
    rightMotor->setSpeed(speed);
}

// set both drive motors forward at the given speed (0-255)
void driveForward(uint8_t speed) {
    setNewSpeed(speed);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
}

// set both drive motors reverse at the given speed (0-255)
void driveReverse(uint8_t speed) {
    setNewSpeed(speed);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
}

// stop both drive motors
void driveStop() {
    setNewSpeed(0);
}

// turn in place by driving the right motor forward and the left
// motor reverse at the given speed (0-255)
void driveLeft(uint8_t speed) {
    setNewSpeed(speed);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
}

// turn in place by driving the left motor forward and the right
// motor reverse at the given speed (0-255)
void driveRight(uint8_t speed) {
    setNewSpeed(speed);
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
}
