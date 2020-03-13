/**
 * @file include/settings.hpp
 */ 
#pragma once

/**
 * a Note on V5 Smart Ports:
 * 
 * Some of the ports are dead.
 * Known dead ports: 1, 9, 14, 18, 20, 11, 13
 * Inconsistent ports: 16, 17, 19, 12
 */


/**
 *      Motor Ports
 */

//Chassis ports
#define CHASSIS_LEFT_FRONT_MOTORPORT   7
#define CHASSIS_LEFT_BACK_MOTORPORT    6
#define CHASSIS_RIGHT_FRONT_MOTORPORT  2
#define CHASSIS_RIGHT_BACK_MOTORPORT   3

//Roller ports
#define ROLLER_RIGHT_MOTORPORT         15
#define ROLLER_LEFT_MOTORPORT          4

//Tilter ports
#define TILTER_MOTORPORT               5

//Arm ports
#define ARM_MOTORPORT                  8

/**
 *      Sensor Constants
 */

//Inertial Measurement Units
#define IMU_SENSORPORT                 10

//Limit Switches
#define ARM_LIMIT_SENSORPORT           'H'
#define TILTER_LIMIT_SENSORPORT        'B'

//Infrared Light Sensors (Line Followers)
#define ROLLER_IR_SENSORPORT           'G'

//Roller macro states
#define CUBE_NOT_FOUND                 0
#define CUBE_DETECTED                  1
#define CUBE_IN_ROLLERS                2
#define CUBE_DROP                      3

//Tilter states
#define TILTER_WITHDRAW                0
#define TILTER_DEPLOY                  1
#define TILTER_MANUAL                  2

/**
 *      PID constants
 */
 //Chassis
#define PID_LEFT_CHASSIS  0, 0.5, 0.0, 0.3
#define PID_RIGHT_CHASSIS 1, 0.5, 0.0, 0.3
#define PID_TURN_CHASSIS  2, 0.9, 0.1, 0.0

 //Tilter
 #define PID_TILTER       3, 0.7, 0.0, 0.0

 //TODO: if having trouble tuning pid for tilter try using piecewise kP kI kD

 //Arm
 #define PID_ARM_AUTO     4, 0.4, 0.0, 0.0
 #define PID_ARM_HOLD     4, 0.15, 0.0, 0.0

 //Rollers
 #define PID_ROLLERS      5, 0.9, 0.0, 0.0

/**
 *      Autonomous Selector Constants
 */ 

//variable used to select an autonomous from user input
extern int autonIdentifier;

 #define AUTON_BLUE_SQUARE_ONEROW 4
 #define AUTON_BLUE_SQUARE_TWOROW 1
 #define AUTON_BLUE_LARGE_STACK   2
 #define AUTON_RED_SQUARE_ONEROW  3
 #define AUTON_RED_SQUARE_TWOROW  0
 #define AUTON_RED_LARGE_STACK    5
 #define AUTON_ONE_CUBE           6
 #define AUTON_NONE               7

/**
 *      Macro shift buttons
 */

//the shift button for the tower macro
#define TOWER_MACRO_BUTTON        okapi::ControllerDigital::down

//the shift button for stacking cubes
#define STACK_MACRO_BUTTON        okapi::ControllerDigital::B
 
/**
 *      Robot Physical Constants
 */

//Drive wheels
#define DRIVE_WHEEL_DIAMETER        4.00_in

//Odometry wheels
#define LEFT_ODOM_WHEEL_TO_CENTER   3.95_in
#define RIGHT_ODOM_WHEEL_TO_CENTER  3.33_in
#define MID_ODOM_WHEEL_TO_CENTER    6.81_in


//Chassis Kinematics
#define ROBOT_CHASSIS_WIDTH         0.0_in
#define MAX_VELOCITY                8.0_inps
#define MAX_ACCELERATION            2.0_inps2

//Tilter Positions
#define TILTER_DEPLOY_ANGLE         -485.0
#define TILTER_VERTICAL_ZONE        -280.0
#define TILTER_RESTING_ANGLE        0.0

/**
 *      Field Coordinates
 */
//Towers
#define CENTER_TOWER_COORDS          Point{0.0_in, 0.0_in}
#define NORTH_TOWER_COORDS           Point{0.0_in, 36.0_in}
#define SOUTH_TOWER_COORDS           Point{0.0_in, -36.0_in}
#define EAST_TOWER_COORDS            Point{48.0_in, 0.0_in}
#define WEST_TOWER_COORDS            Point{-48.0_in, 0.0_in}
#define RED_TOWER_COORDS             Point{-36.0_in, -72.0_in}
#define BLUE_TOWER_COORDS            Point{36.0_in, -72.0_in}

//Goals
#define RED_PROTECTED_GOAL_COORDS    Point{-72.0_in, 72.0_in}
#define RED_SQUARE_GOAL_COORDS       Point{-72.0_in, -72.0_in}
#define BLUE_PROTECTED_GOAL_COORDS   Point{72.0_in, 72.0_in}
#define BLUE_SQUARE_GOAL_COORDS      Point{72.0_in, -72.0_in}
