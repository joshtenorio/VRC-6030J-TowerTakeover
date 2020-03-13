/**
 *  @file src/robot/Pid.cpp
 */
#include "main.h"

static float positionErrorIntegral[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float integralZone[10] = {0, 0, 20, 20, 0, 0, 0, 0, 0, 0};
float errorPrevious[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float PID(double current, double target, char subsystemIdentifier, float kP, float kI, float kD) {
    float errorCurrent = (float) (target - current);
    
    float derivative = errorCurrent - errorPrevious[subsystemIdentifier];
    errorPrevious[subsystemIdentifier] = errorCurrent;

    float pTerm = kP * (float) errorCurrent;
    float iTerm = 0.0;
    if(errorCurrent < integralZone[subsystemIdentifier]){
        positionErrorIntegral[subsystemIdentifier] += errorCurrent;
        iTerm = kI * positionErrorIntegral[subsystemIdentifier];
    }
    float dTerm = kD * derivative;
    int motorSpeed = (int) (pTerm + iTerm + dTerm);
    
    return (float) okapi::remapRange(motorSpeed, -127.0, 127.0, -1.0, 1.0);
}



