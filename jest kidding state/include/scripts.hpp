/**
 * @file include/scripts.hpp
 */ 
#pragma once

namespace Driver{
    void velocityChassis(okapi::Controller master);
    void voltageChassis(okapi::Controller master);
    void rollers(okapi::Controller master);
    void holdCube(okapi::Controller master);
    void tilter(okapi::Controller master);
    void arm(okapi::Controller master);
    void setArmTarget(okapi::Controller master);
    void reset(okapi::Controller master);
    void deployStack(okapi::Controller master);
}

namespace Autonomous{
    void deploy();
    void oneCube();
    void blueRow();
    void blueTwoRow();
    void BlueLargeStack();
    void redRow();
    void redTwoRow();
    void redLargeStack();
}