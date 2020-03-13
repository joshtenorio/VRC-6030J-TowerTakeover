/**
 * @file include/settings.hpp
 */ 
#pragma once

/**
 *      Motor Ports
 */

//Chassis ports
#define CHASSIS_LEFT_MOTORPORT     5
#define CHASSIS_RIGHT_MOTORPORT    15
#define CHASSIS_MIDDLE_MOTORPORT   14

//Roller ports
#define ROLLER_RIGHT_MOTORPORT     11
#define ROLLER_LEFT_MOTORPORT      17

//Tilter ports
#define TILTER_MOTORPORT           3

//Lift ports
#define LIFT_RIGHT_MOTORPORT       2
#define LIFT_LEFT_MOTORPORT        8

/**
 *      Sensor Constants
 */

//Odometry Wheels
#define LEFT_ODOM_SENSOR_CONFIG    'A', 'B', true
#define MIDDLE_ODOM_SENSOR_CONFIG  'C', 'D', false
#define RIGHT_ODOM_SENSOR_CONFIG   'E', 'F', false

extern okapi::ADIEncoder rightEnc;
extern okapi::ADIEncoder leftEnc;
extern okapi::ADIEncoder midEnc;

/**
 *      PID constants
 */
 //Chassis

 //Tilter

 //Lift
/**
 *      Robot Physical Constants
 */

//Drive wheels
#define DRIVE_WHEEL_DIAMETER        3.25_in

//Odometry wheels
#define LEFT_ODOM_WHEEL_TO_CENTER   3.95_in
#define RIGHT_ODOM_WHEEL_TO_CENTER  3.33_in
#define MID_ODOM_WHEEL_TO_CENTER    6.81_in

#define LEFT_ODOM_WHEEL_DIAMETER    2.75_in
#define RIGHT_ODOM_WHEEL_DIAMETER   2.75_in
#define MID_ODOM_WHEEL_DIAMETER     2.75_in

//Pure pursuit controller constants
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


