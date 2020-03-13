/**
 * @file src/robot/arm.cpp
 */
#include "main.h"

okapi::Motor arm(ARM_MOTORPORT, false, okapi::AbstractMotor::gearset::green);
pros::ADIDigitalIn armSwitch(ARM_LIMIT_SENSORPORT);


double liftTarget = 0.0;

void Arm::reset() {
	arm.tarePosition();
}

void Arm::setVelocity(float liftSpeed){
    arm.controllerSet(liftSpeed);
}

void Arm::setVoltage(float liftVolt){
    arm.moveVoltage( (int) (liftVolt * 12000));
}

int Arm::getLimitSwitch(){
    return armSwitch.get_value();
}

double Arm::getPosition(){
    if(Arm::getLimitSwitch()) arm.tarePosition();

    return arm.getPosition();
}

void Arm::setPID(float target){
    Arm::setVelocity(PID(Arm::getPosition(), target, PID_ARM_AUTO));
}

void Arm::setAngle(float angle){
    double threshold = 50.0;
    double diff = abs(angle) - abs(Arm::getPosition());

    while(diff > threshold){
        Arm::setPID(angle);

        diff = abs(angle) - abs(Arm::getPosition());
        pros::delay(20);
    }
    Arm::setVelocity(0.0);
}

double Arm::getMotorTemperature(){
    return arm.getTemperature();
}