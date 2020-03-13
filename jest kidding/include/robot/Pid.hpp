/**
 *  @file include/robot/Pid.hpp
 */
#pragma once

float PID(double current, double target, char subsystemIdentifier, float kP, float kI, float kD);