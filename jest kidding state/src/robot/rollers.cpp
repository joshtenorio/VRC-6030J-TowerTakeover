/**
 * @file src/robot/rollers.cpp
 */ 
#include "main.h"

okapi::MotorGroup rollerMotors({-ROLLER_RIGHT_MOTORPORT, ROLLER_LEFT_MOTORPORT});
pros::ADILineSensor irSensor(ROLLER_IR_SENSORPORT);
int irZeroValue = 0;

void Rollers::initialize(){
    
    rollerMotors.tarePosition();

    int sumSensorValues = 0;
    for(int i = 0; i < 500; i++){
        sumSensorValues += irSensor.get_value();
        pros::delay(1);
    }
    irZeroValue = sumSensorValues/500;
    JesterOS::write(3, "calibrated rollers!");
}

void Rollers::setVelocity(float speed){
    rollerMotors.controllerSet(speed);
}

double Rollers::getPosition(){
    return rollerMotors.getPosition();
}

int Rollers::getIRCalibratedData(){
    return irSensor.get_value() - irZeroValue;//irSensor.get_value_calibrated();
}

int Rollers::getIRRawData(){
    return irSensor.get_value();
}

std::vector<MotorData> Rollers::copyData(){
    std::vector<MotorData> dataOutput;
    dataOutput.push_back(MotorData{rollerMotors.getPosition(), rollerMotors.getTargetVelocity(), rollerMotors.getActualVelocity()});
    return dataOutput;
}

void Rollers::runCopyCat(int velocity){
    rollerMotors.moveVelocity(velocity);
}

double Rollers::getMotorTemperature(){
    rollerMotors.getTemperature();
}
