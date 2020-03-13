#include "main.h"
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 *
 */


void opcontrol() {
	okapi::Controller master;
	PositionTracker tracker(0.0_in, 0.0_in, 0.0_deg, 2.75_in, 2.75_in, 2.75_in, LEFT_ODOM_WHEEL_TO_CENTER, RIGHT_ODOM_WHEEL_TO_CENTER, MID_ODOM_WHEEL_TO_CENTER);
    
    double maxVelocity = 0.0;
	while (true) {
	    tracker.updatePosition(leftEnc.get(), rightEnc.get(), midEnc.get());
	    //Chassis::updatePosition(tracker);

	    double thetaInDegrees = tracker.getTheta() * (180.0 / PI);
        pros::lcd::print(0, "x: %f", tracker.getX());
        pros::lcd::print(1, "y: %f", tracker.getY());
        pros::lcd::print(2, "theta: %f", thetaInDegrees);
        tracker.logPosition();
        double currVelocity = Chassis::getLeftVelocity();
        if(currVelocity > maxVelocity){
            maxVelocity = currVelocity;
        }
        pros::lcd::print(3, "max velocity %f in/s", maxVelocity);


        if(master.getDigital(okapi::ControllerDigital::B)){
            Driver::deploy(master);
            
        }
        else if(master.getDigital(okapi::ControllerDigital::down)){
            Driver::towerDrop(master);
            Tilter::initialize();
        }
        else{
            Chassis::driver(master);
            Rollers::driver(master);
            Tilter::driver(master);
            Lift::driver(master);
        }

		pros::delay(20);
	}
}
