/**
 *  @file src/robot/Pid.cpp
 */
#include "main.h"

float PID(double current, double target, float kp){
	return (float) okapi::remapRange(kp * (float) (target - current), -127.0, 127.0, -1.0, 1.0);
}
