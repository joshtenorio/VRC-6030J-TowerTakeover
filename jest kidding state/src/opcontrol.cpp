#include "main.h"
/**
 * @file opcontrol.cpp
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

//CopyCatController nathan;

void opcontrol() {
	okapi::Controller master;
	
	while (true) {

		if(master.getDigital(TOWER_MACRO_BUTTON)){
			Driver::voltageChassis(master);
			Driver::holdCube(master);
			Driver::setArmTarget(master);
			Chassis::resetGyro();
		}
		else if(master.getDigital(STACK_MACRO_BUTTON)){
			Driver::voltageChassis(master);
			Driver::deployStack(master);
		}
		else{
			//Driver::velocityChassis(master);
			Driver::voltageChassis(master);
			Driver::rollers(master);
			Driver::tilter(master);
			Driver::arm(master);
		}

		//Driver::reset(master);
		pros::delay(20);
		JesterOS::write(4, "gyro %f", Chassis::getRotation());
		JesterOS::write(1, "roller IR cali %d", Rollers::getIRCalibratedData());
		JesterOS::motorMonitorTemperature();
	}
}
