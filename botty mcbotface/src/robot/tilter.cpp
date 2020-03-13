/**
 * @file src/robot/tilter.cpp
 */
#include "main.h"

okapi::Motor tilter(TILTER_MOTORPORT, true, okapi::AbstractMotor::gearset::red);



void Tilter::initialize(){
    tilter.tarePosition();
}

void Tilter::setVelocity(float speed){
    tilter.controllerSet(speed);
}

double Tilter::getCurrentAngle(){
    return tilter.getPosition();
}

void Tilter::setAngle(float angle){
    /**
    while (Tilter::getCurrentAngle() < angle || Tilter::getCurrentAngle() > targetAngle){
        if (Tilter::getCurrentAngle() < angle){
            Tilter::setVelocity(50);
        }
        else{
            Tilter::setVelocity(-50);
        }
    }**/
    double threshold = 50.0;
    double diff = abs(angle) - abs(Tilter::getCurrentAngle());
    while(diff > threshold){
        Tilter::autoGoToAngle(angle);

        diff = abs(angle) - abs(Tilter::getCurrentAngle());
        pros::delay(20);
    }
    Tilter::setVelocity(0.0);
}

void Tilter::autoGoToAngle(float angle){
    Tilter::setVelocity(PID(Tilter::getCurrentAngle(), (double) angle, 0.7));
}

void Tilter::driver(okapi::Controller master){
    float speed;
    //pros::lcd::print(4, "tilter degrees %f", tilter.getPosition());
    if(master.getDigital(okapi::ControllerDigital::left)){
        speed = 1.0;
    }
    else if(master.getDigital(okapi::ControllerDigital::up)){
        if(tilter.getPosition() < -650.0){
            speed = -0.4;
        }
        else{
            speed = -1.0;
        }
    }
    else{
        speed = 0.0;
    }
    Tilter::setVelocity(speed);
}

std::vector<MotorData> Tilter::copyData(){
    std::vector<MotorData> dataOutput;
    dataOutput.push_back(MotorData{tilter.getPosition(), tilter.getTargetVelocity(), tilter.getActualVelocity()});
    return dataOutput;
}

void Tilter::runCopyCat(int velocity){
    tilter.moveVelocity(velocity);
}
