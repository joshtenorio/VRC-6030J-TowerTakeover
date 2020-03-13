/**
 * @file src/scripts/autored.cpp
 */
#include "main.h" 

 
void Autonomous::redRow(){


    //row
    Chassis::driveTo(2450.0, 2450.0, false, 0.35, 6000);

    //tower cube (6th)
    Chassis::driveTo(0, 400, false, 0.35, 4000);
    Chassis::driveTo(650, 650, false, 0.3, 4000);
    pros::delay(900);
    Chassis::driveTo(-650, -650, false, 0.3, 4000);
    Rollers::setVelocity(0.0);
    Chassis::driveTo(0, -400, false, 0.35, 4000);
    Rollers::setVelocity(1.0);
    //next 2 lines are for 7th cube
    //Chassis::driveTo(650, 650, false, 0.35, 4000);
    //pros::delay(400);

    Chassis::driveTo(-1100.0, -1100.0, false, 0.6, 6000);
    //Chassis::smartRobot(-1100, -1100, false, 0.5, 1300, 0, 0, 4500);
    Rollers::setVelocity(0.0);
    //turn
    Chassis::turnTo(165.0, 1.0, 1000);

    Rollers::setVelocity(-0.5);
    pros::delay(300);
    Rollers::setVelocity(0.0);

    Chassis::smartRobot(1600, 1600, false, 0.45, -1300, 0.0, 0.0, 2000);
    Tilter::setAngle(-2800, 1800);
    Chassis::smartRobot(-1000, -1000, false, 0.4, -2200, -0.7, 0.0, 2500);
    Tilter::setVelocity(0.0);
    Rollers::setVelocity(0.0);
}

void Autonomous::redTwoRow(){
    //Autonomous::deploy();
    //row1
    Rollers::setVelocity(1.0);
    Arm::setVelocity(-0.5);
    Chassis::driveTo(1000, 1000, false, 0.50, 5000);
    pros::delay(500);
  
    //Nturn
    Chassis::turnTo(-85.0, 1.0, 1250);
    Rollers::setVelocity(0.0);
    Chassis::smartRobot(-1900, -1900, false, 0.6, 0.0, 0.0, 0.0, 1700);
    //Chassis::driveTo(-2100, -2100, false, 0.6, 4000);
    //Chassis::driveTo(-150, -50, false, 0.4, 1000);
    Chassis::turnTo(80.0, 1.0, 1250);
 
    //row2 
    Chassis::smartRobot(2200, 2200, false, 0.5, 0.0, 1.0, 0.0, 7000);
    pros::delay(750);
 
    Chassis::driveTo(-700.0, -700.0, false, 0.4, 6000);
 
    //turnDeploy
    Chassis::turnTo(140, 1.0, 1600);

    Chassis::smartRobot(1850, 1850, false, 0.35, -900, -.05, 0.0, 2000);
    Chassis::smartRobot(2000, 2000, false, 0.35, -2200, 0.0, 0.0, 1600);
    //Tilter::setAngle(-2400, 1800);
    Chassis::smartRobot(-1000, -1000, false, 0.4, -1000, -0.7, 0.0, 2500);
    Rollers::setVelocity(0.0);
    Tilter::setVelocity(0.0);
}

void Autonomous::redLargeStack(){
    
    Arm::setVelocity(-0.1);
    Rollers::setVelocity(1.0);
    Chassis::driveTo(1200, 1200, false, 0.45, 2000);
    Chassis::turnTo(120, 1, 2000);
    Chassis::driveTo(1200, 1200, false, 0.3, 2000);
    pros::delay(900);
    Chassis::driveTo(-800, -800, false, 0.35, 1500);
    Chassis::turnTo(195, 1, 2000);
    Chassis::driveTo(1000, 1000, false, 0.3, 1500);
    Chassis::turnTo(-90, 1, 1500);
    Chassis::driveTo(450, 450, false, 0.3, 1200);
    Rollers::setVelocity(-0.2);
    pros::delay(700);
    Rollers::setVelocity(0.0);
    Tilter::setAngle(-2000, 2500);
    Tilter::setVelocity(0.0);
    Chassis::smartRobot(-1000, -1000, false, -0.4, -2000, -0.5, 0.0, 2000);
}