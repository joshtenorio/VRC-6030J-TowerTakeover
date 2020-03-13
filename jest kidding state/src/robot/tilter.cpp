/**
 * @file src/robot/tilter.cpp
 */
#include "main.h"

okapi::Motor tilter(TILTER_MOTORPORT);
pros::ADIDigitalIn tilterSwitch(TILTER_LIMIT_SENSORPORT);

void Tilter::initialize(){
    tilter.tarePosition();
}

int Tilter::getLimitSwitch(){
    return tilterSwitch.get_value();
}

void Tilter::setVelocity(float speed){
    tilter.controllerSet(speed);
}

double Tilter::getCurrentAngle(){
    if(Tilter::getLimitSwitch()) tilter.tarePosition();

    return tilter.getPosition();
}

void Tilter::setAngle(float angle, int timeLimit){

    int startTime = pros::millis();

    int timer = pros::millis() - startTime;
    double threshold = 50.0;
    double diff = abs(angle) - abs(Tilter::getCurrentAngle());

    while(diff > threshold){
        Tilter::setPID(angle);

        diff = abs(angle) - abs(Tilter::getCurrentAngle());
        pros::delay(20);

        timer = pros::millis() - startTime;
        if(timer > timeLimit) break;
    }
    Tilter::setVelocity(0.0);
}

void Tilter::setPID(float target){
    Tilter::setVelocity(PID(Tilter::getCurrentAngle(), (double) target, PID_TILTER));
}

std::vector<MotorData> Tilter::copyData(){
    std::vector<MotorData> dataOutput;
    dataOutput.push_back(MotorData{tilter.getPosition(), tilter.getTargetVelocity(), tilter.getActualVelocity()});
    return dataOutput;
}

void Tilter::runCopyCat(int velocity){
    tilter.moveVelocity(velocity);
}

double Tilter::getMotorTemperature(){
    return tilter.getTemperature();
}
