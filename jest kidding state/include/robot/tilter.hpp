/**
 * @file include/robot/tilter.hpp
 */ 
#pragma once
#include "robot/copycat.hpp"
namespace Tilter{
    void initialize();
    int getLimitSwitch();
    void setVelocity(float speed);
    double getCurrentAngle();
    void setAngle(float angle, int timeLimit);
    void setPID(float target);
    std::vector<MotorData> copyData();
    void runCopyCat(int velocity);
    double getMotorTemperature();
}
