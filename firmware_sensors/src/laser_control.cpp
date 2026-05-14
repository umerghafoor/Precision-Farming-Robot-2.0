#include "laser_control.h"
#include "constants.h"
#include <Arduino.h>

static bool laserState = false;

void initLaser() {
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    laserState = false;
}

void laserOn() {
    digitalWrite(LASER_PIN, HIGH);
    laserState = true;
}

void laserOff() {
    digitalWrite(LASER_PIN, LOW);
    laserState = false;
}

bool isLaserOn() {
    return laserState;
}
