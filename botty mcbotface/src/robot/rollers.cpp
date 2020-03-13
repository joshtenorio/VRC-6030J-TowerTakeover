/**
 * @file src/robot/rollers.cpp
 */ 
#include "main.h"

okapi::MotorGroup rollerMotors({-ROLLER_RIGHT_MOTORPORT, ROLLER_LEFT_MOTORPORT});

//initial speed and direction
float rollerSpeed = 1.0;
float rollerDirection = 1.0;

void Rollers::initialize(){
    rollerMotors.tarePosition();
}
void Rollers::setVelocity(float speed){
    rollerMotors.controllerSet(speed);
}

void Rollers::driver(okapi::Controller master){

    if(master.getDigital(okapi::ControllerDigital::right)) rollerDirection = -1.0;
    else if(master.getDigital(okapi::ControllerDigital::Y)) rollerDirection = 1.0;

    if(master.getDigital(okapi::ControllerDigital::X)) rollerSpeed = 1.0;
    else if(master.getDigital(okapi::ControllerDigital::A)) rollerSpeed = 0.5;

    if(master.getDigital(okapi::ControllerDigital::L1)) Rollers::setVelocity(0.0);
    else if(master.getDigital(okapi::ControllerDigital::L2)) Rollers::setVelocity(rollerDirection * rollerSpeed);
}

std::vector<MotorData> Rollers::copyData(){
    std::vector<MotorData> dataOutput;
    dataOutput.push_back(MotorData{rollerMotors.getPosition(), rollerMotors.getTargetVelocity(), rollerMotors.getActualVelocity()});
    return dataOutput;
}

void Rollers::runCopyCat(int velocity){
    rollerMotors.moveVelocity(velocity);
}
