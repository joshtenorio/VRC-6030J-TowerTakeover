/**
 * @file src/scripts/driverscripts.cpp
 */ 
#include "main.h"

//globals
int rollerState = CUBE_NOT_FOUND;
double rollerCubePosition = 0.0;
bool rollerMacroActive;
bool holdingStack = false;
double rollerStackPosition = -70.0;
double armTarget = 0.0;
double armPosition = 0.0;
bool armMacroActive = false;
int tilterStatus = TILTER_MANUAL;
double lChassisStackPosition = -100.0;
double rChassisStackPosition = -100.0;

void Driver::velocityChassis(okapi::Controller master){
    //deadbanding joystick inputs to filter out unecessary chassis movements
    //need to push joystick more than 5% to get any sort of tilterOutput
    float leftInput = (float) okapi::deadband(master.getAnalog(okapi::ControllerAnalog::leftY), -0.05, 0.05);
    float rightInput = (float) okapi::deadband(master.getAnalog(okapi::ControllerAnalog::rightY), -0.05, 0.05);

    //cubing each joystick input so that we get more precision in chassis speed without losing top speeds
    //if we push the left joystick at ~80%, the left side will tilterOutput ~50% power
    float leftOutput = leftInput * leftInput * leftInput;
    float rightOutput = rightInput * rightInput *rightInput;

    Chassis::setVelocity(leftOutput, rightOutput);
}


void Driver::voltageChassis(okapi::Controller master){
    float leftInput = (float) okapi::deadband(master.getAnalog(okapi::ControllerAnalog::leftY), -0.05, 0.05);
    float rightInput = (float) okapi::deadband(master.getAnalog(okapi::ControllerAnalog::rightY), -0.05, 0.05);

    float leftCubed = leftInput * leftInput * leftInput;
    float rightCubed = rightInput * rightInput * rightInput;

    float leftOutput = leftCubed;
    float rightOutput = rightCubed;
    if(master.getDigital(okapi::ControllerDigital::B)){
        leftOutput = (float) constrain(leftCubed, -0.8, 0.8);
        rightOutput = (float) constrain(rightCubed, -0.8, 0.8);
    }
    Chassis::setVoltage(leftOutput, rightOutput);

    //updating roller stack position here because this function runs regardless of shift key
    //putting it here because if not then this line shows up three different times, so easier
    //to update value here if need be
    if(!holdingStack){
        rollerStackPosition = Rollers::getPosition() - 150.0;
        lChassisStackPosition = Chassis::getLeftPosition() - 100.0;
        rChassisStackPosition = Chassis::getRightPosition() - 100.0;
    }

}


void Driver::rollers(okapi::Controller master){
    float tilterOutput = 0.0;
    rollerMacroActive = false;
    if(master.getDigital(okapi::ControllerDigital::L2)){
        holdingStack = false;
        tilterOutput = 1.0;
        Arm::setVelocity(-0.1);  
    }
    else if(master.getDigital(okapi::ControllerDigital::L1)){
        tilterOutput = -1.0;
        holdingStack = false;
        }
    else{
        tilterOutput = 0.0;
        holdingStack = false;
    }

    Rollers::setVelocity(tilterOutput);
}


void Driver::holdCube(okapi::Controller master){
    float tilterOutput = 0.0;
    //intake cube to hold in roller
    if(master.getDigital(okapi::ControllerDigital::L2)){
        rollerMacroActive = true;
        rollerState = CUBE_NOT_FOUND;
        holdingStack = false;
    }

    //drop a cube from tray to hold in roller
    else if(master.getDigital(okapi::ControllerDigital::Y)){
        rollerMacroActive = true;
        rollerState = CUBE_DROP;
        holdingStack = false;
    }
    //else if L1, run rollers outward but slower - for putting cube in tower
    else if(master.getDigital(okapi::ControllerDigital::L1)){
        rollerMacroActive = false;
        holdingStack = false;
        tilterOutput = -0.5;
    }
    else if(!rollerMacroActive) tilterOutput = 0.0;

    //roller macro active
    if(rollerMacroActive){
        switch(rollerState){
            case CUBE_NOT_FOUND:
                if(Rollers::getIRCalibratedData() > -100) tilterOutput = 1.0;
                else{
                    rollerState = CUBE_DETECTED;
                    rollerCubePosition = Rollers::getPosition() + 200.0;
                }
                break;
            case CUBE_DETECTED:
                if(abs(rollerCubePosition - Rollers::getPosition()) > 50.0){
                    tilterOutput = PID(Rollers::getPosition(), rollerCubePosition, PID_ROLLERS);
                }
                else rollerState = CUBE_IN_ROLLERS;
                break;
            case CUBE_IN_ROLLERS:
                tilterOutput = 0.0;
                break;
            case CUBE_DROP:
                if(Rollers::getIRCalibratedData() > -100) tilterOutput = -1.0;
                else tilterOutput = 0.0;
                break;
        } //switch rollerState
    } //if rollerMacroActive

    Rollers::setVelocity(tilterOutput);
} //Driver::holdCube


void Driver::tilter(okapi::Controller master){
    float tilterOutput = 0.0;
    //withdraw tray
    if(master.getDigital(okapi::ControllerDigital::right)){
        /**if(Tilter::getLimitSwitch()) tilterOutput = 0.0;
        else tilterOutput = 1.0;*/
        tilterStatus = TILTER_WITHDRAW;
    }

    //deploy tray
    else if(master.getDigital(okapi::ControllerDigital::Y)){
        tilterStatus = TILTER_MANUAL;
        //slow zone - tray almost vertical
        if(Tilter::getCurrentAngle() < -1250.0) tilterOutput = -0.4;
        //normal zone - tray not almost vertical
        else tilterOutput = -1.0;

        //keeping arm down
        Arm::setVelocity(-0.1);
    }

    //inactive state
    else tilterOutput = 0.0;

    if(tilterStatus == TILTER_WITHDRAW && !Tilter::getLimitSwitch()) tilterOutput = 1.0;
    else if(tilterStatus == TILTER_WITHDRAW && Tilter::getLimitSwitch()){
        tilterOutput = 0.0;
        tilterStatus = TILTER_MANUAL;
    }
    Tilter::setVelocity(tilterOutput);
}


//TODO: optimise macro stuff ie updating current arm position
void Driver::arm(okapi::Controller master){

    printf("arm position %f\n", armPosition);
    armPosition = Arm::getPosition();

    if(Arm::getLimitSwitch()) armPosition = 0.0;
    
    //going up
    if(master.getDigital(okapi::ControllerDigital::R1)){
        Arm::setVelocity(1.0);
        armTarget = Arm::getPosition();
        //manually controlling arm so setting macro boolean to false
        armMacroActive = false;
    }

    //going down
    else if(master.getDigital(okapi::ControllerDigital::R2)){

        //move arm down unless limit switch is active
        if(Arm::getLimitSwitch()) Arm::setVelocity(0.0);
        else Arm::setVelocity(-0.8);

        armTarget = Arm::getPosition();
        //setting macro boolean to false because we are manually controlling arm
        armMacroActive = false;
    }

    //default state
    else{
        //if arm is low, don't hold position (don't need to)
        if(armPosition < 150.0) Arm::setVelocity(0.0);
        //else if arm is high, hold position
	    else Arm::setVelocity(PID(armPosition, armTarget, PID_ARM_HOLD));
    }
}

void Driver::setArmTarget(okapi::Controller master){
    armPosition = Arm::getPosition();

    if(Arm::getLimitSwitch()) armPosition = 0.0;

    //setting arm targets
    if(master.getDigital(okapi::ControllerDigital::R1)){ //mid tower
        armTarget = 475.0;
        armMacroActive = true;
    }
    //bring arm down unless limit switch is activated
    else if(master.getDigital(okapi::ControllerDigital::R2) && !Arm::getLimitSwitch()){
        Arm::setVelocity(-0.8);
    }
    else Arm::setVelocity(0.0);

    //sending arm targets to motor, or not
    if(armMacroActive){ //macro is active so set target to arm macro
        Arm::setVelocity(PID(Arm::getPosition(), armTarget, PID_ARM_AUTO));
    }
    else if(Arm::getPosition() < 100) Arm::setVelocity(0.0);
	else Arm::setVelocity(PID(Arm::getPosition(), armTarget, PID_ARM_HOLD));
	   	
}

void Driver::reset(okapi::Controller master){
    if(master.getDigital(okapi::ControllerDigital::A)){
        Arm::reset();
        Tilter::initialize();
    }
}

//this variable is only used for deployStack function
float tilterOutput = 0.0;
void Driver::deployStack(okapi::Controller master){

    //withdraw tray
    if(master.getDigital(okapi::ControllerDigital::right)) tilterStatus = TILTER_WITHDRAW;
    //deploy tray
    else if(master.getDigital(okapi::ControllerDigital::Y)){
        tilterStatus = TILTER_DEPLOY;
    }

    //activate roller PID for keeping stack in rollers
    if(master.getDigital(okapi::ControllerDigital::R2)) holdingStack = true;

    //outtake slowly
    if(master.getDigital(okapi::ControllerDigital::L1)){
        Rollers::setVelocity(-0.5);
    }
    //hold stack in rollers
    else if(holdingStack) Rollers::setVelocity(PID(Rollers::getPosition(), rollerStackPosition, PID_ROLLERS));
    else Rollers::setVelocity(0.0);

    switch(tilterStatus){
        case TILTER_WITHDRAW:
            if(!Tilter::getLimitSwitch()) tilterOutput = 1.0;
            else{
                tilterOutput = 0.0;
                tilterStatus = TILTER_MANUAL;
            }
            break;
        case TILTER_DEPLOY:
                tilterOutput = (float) constrain(PID(Tilter::getCurrentAngle(), -2080.0, 3, 0.35, 0.0, 0.9), -0.8, 0.0);
                Arm::setVelocity(-0.1);
                break;
        case TILTER_MANUAL:
                tilterOutput = 0.0;
                break;
    }

    Tilter::setVelocity(tilterOutput);
}
