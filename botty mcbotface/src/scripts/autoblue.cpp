/**
 * @file src/scripts/autoblue.cpp
 */ 
#include "main.h"

void Autonomous::oneCube(){
    Chassis::setVelocity(-1.0, -1.0);
    pros::delay(500);
    Chassis::setVelocity(0.0, 0.0);
    pros::delay(600);
    Chassis::setVelocity(1.0, 1.0);
    pros::delay(750);
    Chassis::setVelocity(0.0, 0.0);
}

PositionTracker tracker(0.0_in, 0.0_in, 0.0_deg, 2.75_in, 2.75_in, 2.75_in, LEFT_ODOM_WHEEL_TO_CENTER, RIGHT_ODOM_WHEEL_TO_CENTER, MID_ODOM_WHEEL_TO_CENTER);
    
void Autonomous::blueRow(){
    //ChassisConfig robot = ChassisConfig{2.75, 2.75, 2.75, LEFT_ODOM_WHEEL_TO_CENTER, RIGHT_ODOM_WHEEL_TO_CENTER, MID_ODOM_WHEEL_TO_CENTER};
    
    //PurePursuitController chassisController(Pose{0.0, 0.0, 0.0}, robot, 8.0, 2.0, 1.0/41.0, 0.0, 0.0);
    //chassisController.generatePath({Point{0.0, 0.0}, Point{60.0, 0.0}}, 4, 30.0, 1);
    //chassisController.runPath(1, 4.0, false);
    //Chassis::turnTo(90.0_deg, tracker, 0.0, 0.0);

    
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
    Chassis::turnTo(-110.0_deg, tracker, 0.0, 0.0);
    //Chassis::driveTo()
    pros::lcd::print(7, "finish turn");

    Rollers::setVelocity(0.0);
    Lift::setVelocity(0.0);
    //Chassis::driveTo(400.0, 400.0, 0.6);

    Rollers::setVelocity(-0.5);
    pros::delay(500);
    Rollers::setVelocity(0.0);

    Chassis::driveTo(300.0, 300.0, 1.0);
    Tilter::setAngle(-1300.0);
    Chassis::setVelocity(0.8, 0.8);
    pros::delay(800);
    Chassis::setVelocity(0.0, 0.0);
    pros::delay(500);
    Chassis::driveTo(-800.0, -800.0, 1.0);
}

