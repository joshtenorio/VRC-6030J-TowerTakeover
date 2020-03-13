/**
 * @file src/robot/jesterOS.cpp
 * @brief Implementation of JesterOS, 6030J's autonomi selector and debug terminal
 * 
 * NOTE: The write() function is adapted from 5090Z's 2018-2019 code. Their released code
 *      was last updated on 11/17/2019, retrieved on 1/5/2020, and can be found at
 *      https://github.com/timeconfusing/OkapiLVGL/blob/master/src/robot_gui.cpp
 * 
 *      
 *
 * @author Josh Tenorio, ACP Robotics
 */
#include "main.h"
#include <stdio.h>
#include <stdarg.h>

#define MAX_LINES 6

lv_obj_t * autoDesc;
static lv_obj_t *tabview;
static lv_obj_t *autonTab;
static lv_obj_t *debugTab;
static lv_obj_t *motorTab;
static lv_obj_t * mainLabel;
static lv_obj_t * btnmAuto;
static lv_obj_t * debugLabel;
static lv_obj_t * motorLabel;


int autonIdentifier;

void JesterOS::updateAuton() {
	switch (autonIdentifier) {
	case AUTON_RED_SQUARE_TWOROW:
		lv_label_set_text(autoDesc, "red small two row");
		break;
	case AUTON_RED_SQUARE_ONEROW:
		lv_label_set_text(autoDesc, "red small one row");
		break;
	case AUTON_RED_LARGE_STACK:
		lv_label_set_text(autoDesc, "red large goal");
		break;
	case AUTON_BLUE_SQUARE_TWOROW:
		lv_label_set_text(autoDesc, "blue small two row");
		break;
	case AUTON_BLUE_SQUARE_ONEROW:
		lv_label_set_text(autoDesc, "blue small one row");
		break;
	case AUTON_BLUE_LARGE_STACK:
		lv_label_set_text(autoDesc, "blue large goal");
		break;
	case AUTON_ONE_CUBE:
		lv_label_set_text(autoDesc, "one cube");
		break;
	case AUTON_NONE:
		lv_label_set_text(autoDesc, "no auto");
		break;
	}
}

static lv_res_t JesterOS::autonSelect(lv_obj_t * btnm, const char *txt) {
	if (txt == "RL-STACK") autonIdentifier = AUTON_RED_LARGE_STACK;
	else if (txt == "BL-STACK") autonIdentifier = AUTON_BLUE_LARGE_STACK;
	else if (txt == "RS-TWOROW") autonIdentifier = AUTON_RED_SQUARE_TWOROW;
	else if (txt == "BS-TWOROW") autonIdentifier = AUTON_BLUE_SQUARE_TWOROW;
	else if (txt == "RS-ONEROW") autonIdentifier = AUTON_RED_SQUARE_ONEROW;
	else if (txt == "BS-ONEROW") autonIdentifier = AUTON_BLUE_SQUARE_ONEROW;
	else if (txt == "ONECUBE") autonIdentifier = AUTON_ONE_CUBE;
	else autonIdentifier = AUTON_NONE;
	JesterOS::updateAuton();
	//printf("auton identifer %d\n", autonIdentifier);
	return LV_RES_OK;
}

void JesterOS::init() {

	tabview = lv_tabview_create(lv_scr_act(), NULL);
	lv_obj_align(tabview, NULL, LV_ALIGN_IN_TOP_MID, 0, 20);

	autonTab = lv_tabview_add_tab(tabview, "Auton Selector");
	debugTab = lv_tabview_add_tab(tabview, "Debug");
	motorTab = lv_tabview_add_tab(tabview, "Motor Monitor");


	mainLabel = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(mainLabel, "jesterOS v1.0.0");
	lv_obj_align(mainLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);


	autoDesc = lv_label_create(autonTab, NULL);
	lv_label_set_text(autoDesc, "no auto selected");

	lv_obj_align(autoDesc, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);

	static const char * auto_map[] = { "RL-STACK", "BL-STACK", "\n", "RS-TWOROW", "BS-TWOROW", "\n", "RS-ONEROW", "BS-ONEROW", "\n", "ONECUBE", "NOAUTO", "" };

	btnmAuto = lv_btnm_create(autonTab, NULL);
	lv_btnm_set_map(btnmAuto, auto_map);
	lv_btnm_set_action(btnmAuto, JesterOS::autonSelect);

	debugLabel = lv_label_create(debugTab, NULL);
	lv_label_set_text(debugLabel, "owo");
	lv_obj_align(debugLabel, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);

	motorLabel = lv_label_create(motorTab, NULL);
	lv_label_set_text(motorLabel, "owo");
	lv_obj_align(motorLabel, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
}

void JesterOS::write(int line, const char *format, ...) {
	// holds the strings of each line in a 2d array of chars
	//JesterOS uses one (1) label object for all lines, need to keep un-edited lines consistent between write() calls
	static char buffer[MAX_LINES][64] = { "" };

	// lv_label_set_text() takes a simple string. copy the 2d buffer in to this string
	//64 characters per line * number of lines = size of string to give lv_label_set_text()
	char output[MAX_LINES * 64]; 

	// makes sure line is in range
	if ((line >= 1) || (line <= MAX_LINES)){
		va_list args;
		va_start(args, format);

		//takes string to print and entries in argument list according to format specifiers in format
		//and stores it in buffer[line]
		vsnprintf(buffer[line], 63, format, args); // 63 b/c reserve a spot for newline char
		// perror(buffer[line]);
		va_end(args);

		strcpy(output, ""); // output must be empty before we write each buffer line to it
		for (int i = 1; i <= MAX_LINES; i++) {
			strcat(output, buffer[i]);
			strcat(output, "\n"); // makes sure each buffer is on it a new line
		}
		lv_label_set_text(debugLabel, output);
	}
}

void JesterOS::motorMonitor(int line, const char *format, ...){
	static char buffer[MAX_LINES][64] = { "" };

	char output[MAX_LINES * 64]; 

	if ((line >= 1) || (line <= MAX_LINES)){
		va_list args;
		va_start(args, format);

		vsnprintf(buffer[line], 63, format, args); // 63 b/c reserve a spot for newline char
		// perror(buffer[line]);
		va_end(args);

		strcpy(output, ""); // output must be empty before we write each buffer line to it
		for (int i = 1; i <= MAX_LINES; i++) {
			strcat(output, buffer[i]);
			strcat(output, "\n"); // makes sure each buffer is on it a new line
		}
		lv_label_set_text(motorLabel, output);
	}
}

void JesterOS::motorMonitorTemperature(){
	JesterOS::motorMonitor(1, "Motor temperatures");
	JesterOS::motorMonitor(2, "Arm: %f C, Tilter: %f C", Arm::getMotorTemperature(), Tilter::getMotorTemperature());
	JesterOS::motorMonitor(3, "Rollers: %f C", Rollers::getMotorTemperature());
	JesterOS::motorMonitor(4, "Chassis L: %f C, Chassis R: %f C", Chassis::getLeftMotorTemperature(), Chassis::getRightMotorTemperature());
}
