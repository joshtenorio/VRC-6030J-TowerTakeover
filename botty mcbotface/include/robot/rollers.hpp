/**
 * @file include/robot/rollers.hpp
 */ 
#pragma once
#include "robot/copycat.hpp"
namespace Rollers{
    void initialize();
    void setVelocity(float speed);
    void driver(okapi::Controller controller);
    std::vector<MotorData> copyData();
    void runCopyCat(int velocity);
}