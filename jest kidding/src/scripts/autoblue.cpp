/**
 * @file src/scripts/autoblue.cpp
 */ 
#include "main.h"

void Autonomous::deploy(){
    
    Rollers::setVelocity(-1.0);
    pros::delay(900);
    Rollers::setVelocity(1.0);
    Arm::setVelocity(-1.0);
    pros::delay(800);
    Arm::setVelocity(-0.4);
    
    /**
    Arm::setVelocity(1.0);
    Rollers::setVelocity(-1.0);
    pros::delay(1000);
    Arm::setVelocity(-1.0);
    Rollers::setVelocity(1.0);
    pros::delay(700);
    Arm::setVelocity(-0.2);
    **/
}


void Autonomous::oneCube(){
    Chassis::setVelocity(0.5, 0.5);
    pros::delay(500);
    Chassis::setVelocity(0.0, 0.0);
    pros::delay(600);
    Chassis::setVelocity(-0.5, -0.5);
    pros::delay(750);
    Chassis::setVelocity(0.0, 0.0);
}

   
void Autonomous::blueRow(){
    

    //row
    Chassis::driveTo(2450.0, 2450.0, false, 0.35, 6000);

    //tower cube (6th)
    Chassis::driveTo(400, 0, false, 0.35, 4000);
    Chassis::driveTo(650, 650, false, 0.3, 4000);
    pros::delay(900);
    Chassis::driveTo(-650, -650, false, 0.3, 4000);
    Rollers::setVelocity(0.0);
    Chassis::driveTo(-400, 0, false, 0.35, 4000);
    Rollers::setVelocity(1.0);
    //next 2 lines are for 7th cube
    //Chassis::driveTo(650, 650, false, 0.35, 4000);
    //pros::delay(400);

    Chassis::driveTo(-1100.0, -1100.0, false, 0.6, 6000);
    //Chassis::smartRobot(-1100, -1100, false, 0.5, 1300, 0, 0, 4500);
    Rollers::setVelocity(0.0);
    //turn
    Chassis::turnTo(-165.0, 1.0, 1000);

    Rollers::setVelocity(-0.5);
    pros::delay(300);
    Rollers::setVelocity(0.0);

    Chassis::smartRobot(1550, 1550, false, 0.35, -1300, 0.0, 0.0, 2000);
    Tilter::setAngle(-2800, 1800);
    Chassis::smartRobot(-1000, -1000, false, 0.4, -2200, -0.7, 0.0, 2500);
    Rollers::setVelocity(0.0);
}

void Autonomous::blueTwoRow(){
    
    //Autonomous::deploy();
    //row1
    Rollers::setVelocity(1.0);
    Arm::setVelocity(-0.5);
    Chassis::driveTo(1000, 1000, false, 0.60, 5000);
    pros::delay(500);
  
    //Nturn
    Chassis::turnTo(75.0, 1.0, 1500);
    Rollers::setVelocity(0.0);
    Chassis::smartRobot(-2000, -2000, false, 0.6, 0.0, 0.0, 0.0, 1700);
    //Chassis::driveTo(-2100, -2100, false, 0.6, 4000);
    //Chassis::driveTo(-150, -50, false, 0.4, 1000);
    Chassis::turnTo(-80.0, 1.0, 1000);
 
    //row2 
    Chassis::smartRobot(2100, 2100, false, 0.35, 0.0, 1.0, 0.0, 7000);
    pros::delay(750);
 
    Chassis::driveTo(-700.0, -700.0, false, 0.4, 6000);
 
    //turnDeploy
    Chassis::turnTo(-135, 1.0, 1600);

    Chassis::smartRobot(1850, 1850, false, 0.40, -1000, 0.0, 0.0, 2000);
    Chassis::smartRobot(2000, 2000, false, 0.35, -2400, 0.0, 0.0, 1800);
    //Tilter::setAngle(-2400, 1800);
    Chassis::smartRobot(-1000, -1000, false, 0.4, -1000, -0.7, 0.0, 2500);
    Rollers::setVelocity(0.0);
    Tilter::setVelocity(0.0);
  
}


void Autonomous::BlueLargeStack(){
    

    Chassis::driveTo(1000, 1000, false, 0.45, 5000);
    Chassis::turnTo(150, 1, 2000);
    Chassis::driveTo(500, 500, false, 0.4, 3000);
    Rollers::setVelocity(-1.0);
    pros::delay(2000);
    Chassis::driveTo(-400, -400, false, 0.4, 1400);
    Rollers::setVelocity(0.0);
    Arm::setVelocity(0.0);
}

