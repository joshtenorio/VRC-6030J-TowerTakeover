/**
 * @file src/scripts/autored.cpp
 */
#include "main.h" 

PositionTracker robot(0.0_in, 0.0_in, 0.0_deg, 2.75_in, 2.75_in, 2.75_in, LEFT_ODOM_WHEEL_TO_CENTER, RIGHT_ODOM_WHEEL_TO_CENTER, MID_ODOM_WHEEL_TO_CENTER);
 
 
void Autonomous::redRow(){
    
    Chassis::driveTo(500.0, 500.0, 1.0);
    pros::lcd::print(7, "finish step 1");
    Chassis::driveTo(-500.0, -500.0, 0.5);
    Lift::setVelocity(-0.1);
    pros::lcd::print(7, "finish step 2");
    Rollers::setVelocity(1.0);
    Chassis::driveTo(1800.0, 1800.0, 0.3);
    pros::delay(500);
    Rollers::setVelocity(0.0);
    Chassis::driveTo(-1150.0, -1150.0, 1.0);

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

    Chassis::driveTo(300.0, 300.0, 1.0);
    Tilter::setAngle(-1150.0);
    
    Rollers::setVelocity(-0.5);
    Chassis::setVelocity(0.8, 0.8);
    pros::delay(800);
    Chassis::setVelocity(0.0, 0.0);
    pros::delay(500);
    //Rollers::setVelocity(-0.5);
    
    Chassis::driveTo(-800.0, -800.0, 1.0);
}
