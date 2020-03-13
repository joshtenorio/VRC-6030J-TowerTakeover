#include "main.h"

/**
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
    
    Autonomous::blueRow();
    //Autonomous::redRow();
    //Autonomous::oneCube();

    /**
     * BLUE PROTEC
     */
    /**
    Chassis::driveTo(500.0, 500.0, 1.0);
    pros::lcd::print(7, "finish step 1");
    Chassis::driveTo(-500.0, -500.0, 0.5);
    Lift::setVelocity(-0.1);
    pros::lcd::print(7, "finish step 2");
    Rollers::setVelocity(1.0);
    Chassis::driveTo(1600.0, 1600.0, 0.3);
    pros::delay(500);
    Rollers::setVelocity(0.0);
    Chassis::driveTo(-950.0, -950.0, 1.0);

    Rollers::setVelocity(0.8);
    //Chassis::turnTo(110.0_deg, robot, 0.0, 0.0);
    Chassis::driveTo(-750.0, 750.0, 0.7);
    pros::lcd::print(7, "finish turn");

    Rollers::setVelocity(0.0);
    Lift::setVelocity(0.0);
    //Chassis::driveTo(400.0, 400.0, 0.6);

    Rollers::setVelocity(-0.5);
    pros::delay(500);
    Rollers::setVelocity(0.0);

    //Chassis::driveTo(300.0, 300.0, 1.0);
    //Tilter::setAngle(-1300.0);
    Rollers::setVelocity(-0.8);
    pros::delay(2000);
    //Chassis::setVelocity(0.8, 0.8);
    //pros::delay(800);
    //Chassis::setVelocity(0.0, 0.0);
    pros::delay(500);
    Rollers::setVelocity(0.0);
    Chassis::driveTo(-800.0, -800.0, 1.0);


    /**
     * RED PROTEC
     
    Chassis::driveTo(500.0, 500.0, 1.0);
    pros::lcd::print(7, "finish step 1");
    Chassis::driveTo(-500.0, -500.0, 0.5);
    Lift::setVelocity(-0.1);
    pros::lcd::print(7, "finish step 2");
    Rollers::setVelocity(1.0);
    Chassis::driveTo(1600.0, 1600.0, 0.3);
    pros::delay(500);
    Rollers::setVelocity(0.0);
    Chassis::driveTo(-950.0, -950.0, 1.0);

    Rollers::setVelocity(0.8);
    //Chassis::turnTo(110.0_deg, robot, 0.0, 0.0);
    Chassis::driveTo(750.0, -750.0, 0.7);
    pros::lcd::print(7, "finish turn");

    Rollers::setVelocity(0.0);
    Lift::setVelocity(0.0);
    //Chassis::driveTo(400.0, 400.0, 0.6);

    Rollers::setVelocity(-0.5);
    pros::delay(500);
    Rollers::setVelocity(0.0);

    //Chassis::driveTo(300.0, 300.0, 1.0);
    //Tilter::setAngle(-1300.0);
    Rollers::setVelocity(-0.8);
    pros::delay(2000);
    //Chassis::setVelocity(0.8, 0.8);
    //pros::delay(800);
    //Chassis::setVelocity(0.0, 0.0);
    pros::delay(500);
    Rollers::setVelocity(0.0);
    Chassis::driveTo(-800.0, -800.0, 1.0);
    **/
}
