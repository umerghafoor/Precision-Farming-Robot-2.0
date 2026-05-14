#include "laser_control.h"
#include "constants.h"
#include <Arduino.h>

static bool laserState = false;

#if LASER_ACTIVE_LOW
  #define LASER_ON_LEVEL  LOW
  #define LASER_OFF_LEVEL HIGH
#else
  #define LASER_ON_LEVEL  HIGH
  #define LASER_OFF_LEVEL LOW
#endif

void initLaser() {
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LASER_OFF_LEVEL);
    laserState = false;
}

void laserOn() {
    digitalWrite(LASER_PIN, LASER_ON_LEVEL);
    laserState = true;
}

void laserOff() {
    digitalWrite(LASER_PIN, LASER_OFF_LEVEL);
    laserState = false;
}

bool isLaserOn() {
    return laserState;
}
