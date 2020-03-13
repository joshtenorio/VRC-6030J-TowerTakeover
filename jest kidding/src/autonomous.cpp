#include "main.h"

/**
 * @file autonomous.cpp
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {

    //Chassis::turnTo(90.0, 1, 50000);

    Autonomous::deploy();
    //Autonomous::blueTwoRow();
    //Autonomous::redLargeStack();
    
    switch(autonIdentifier){
        case AUTON_BLUE_SQUARE_ONEROW:
            Autonomous::blueRow();
            break;
        case AUTON_BLUE_SQUARE_TWOROW:
            Autonomous::blueTwoRow();
            break;
        case AUTON_BLUE_LARGE_STACK:
            Autonomous::BlueLargeStack();
            break;
        case AUTON_RED_SQUARE_ONEROW:
            Autonomous::redRow();
            break;
        case AUTON_RED_SQUARE_TWOROW:
            Autonomous::redTwoRow();
            break;
        case AUTON_RED_LARGE_STACK:
            Autonomous::redLargeStack();
            break;
        case AUTON_ONE_CUBE:
            Autonomous::oneCube();
            break;
        case AUTON_NONE:
            break;
        default:
            break;
    }

    

    //Autonomous::blueRow();
    //Autonomous::blueTwoRow();
    //Autonomous::redRow();
    //Autonomous::oneCube();

    
}
