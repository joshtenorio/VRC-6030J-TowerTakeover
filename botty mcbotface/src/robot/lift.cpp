/**
 * @file src/robot/lift.cpp
 */
#include "main.h"

//TODO: refactor to MotorGroup
//TODO: make initialize function that resets the lift encoder
//TODO: tune lift PID
okapi::Motor leftLift(LIFT_LEFT_MOTORPORT, true, okapi::AbstractMotor::gearset::red);
okapi::Motor rightLift(LIFT_RIGHT_MOTORPORT, false, okapi::AbstractMotor::gearset::red);
okapi::MotorGroup liftMotors({
	leftLift, rightLift
});

double liftTarget = 0.0;

void Lift::reset() {
	liftMotors.tarePosition();
}

void Lift::setVelocity(float liftSpeed){
    liftMotors.controllerSet(liftSpeed);
}

void::Lift::driver(okapi::Controller master){
    if(master.getDigital(okapi::ControllerDigital::R1)){
        Lift::setVelocity(.8);
        liftTarget = liftMotors.getPosition();
    }
    else if(master.getDigital(okapi::ControllerDigital::R2)){
        Lift::setVelocity(-.45);
        liftTarget = liftMotors.getPosition();
    }
    else{
        if (rightLift.getPosition() < 100) { //lift is low, don't need to hold position
		    Lift::setVelocity(0.0);
	   	  }
	      else {
		        Lift::setVelocity(PID(rightLift.getPosition(), liftTarget, 0.6) + 0.15);
	   		}
    }
}
