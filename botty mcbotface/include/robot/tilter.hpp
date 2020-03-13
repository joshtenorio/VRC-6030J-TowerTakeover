/**
 * @file include/robot/tilter.hpp
 */ 
#pragma once
#include "robot/copycat.hpp"
namespace Tilter{
    void initialize();
    void setVelocity(float speed);
    double getCurrentAngle();
    void setAngle(float angle);
    void autoGoToAngle(float angle);
    void driver(okapi::Controller master);
    std::vector<MotorData> copyData();
    void runCopyCat(int velocity);
}
