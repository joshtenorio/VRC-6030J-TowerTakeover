/**
 * @file src/scripts/drivermacros.cpp
 */ 
#include "main.h"

void Driver::deploy(okapi::Controller master){
    Tilter::driver(master);
    Chassis::driver(master);
    if(master.getAnalog(okapi::ControllerAnalog::leftY) < 0.0 || master.getAnalog(okapi::ControllerAnalog::rightY) < 0.0){
        Rollers::setVelocity(-0.4);
    }
    else{
        Rollers::setVelocity(0.0);
    }
}

void Driver::towerDrop(okapi::Controller master){
    Chassis::driver(master);
    Lift::driver(master);
    Rollers::setVelocity(-0.65);
    
}