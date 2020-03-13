/**
 *  @file include/robot/odometry.hpp
 */
#pragma once

/**
 *      Units
 *
 *      Note: The first unit listed in each section is the unit used internally
 */

//Length
constexpr long double operator"" _in(long double x) { return x; } //inches
constexpr long double operator"" _ft(long double x) {return x * 12.0; } //feet
constexpr long double operator"" _tile(long double x) { return x * 24.0; } //field tile (1 field tile = 2 feet)

//Angle
constexpr long double operator"" _rad(long double x) { return x; } //radians
constexpr long double operator"" _deg(long double x) { return (x * PI) / 180.0; } //degrees
constexpr long double operator"" _rot(long double x) { return x * 2.0 * PI; } //rotations

//Linear Velocity
constexpr long double operator"" _inps(long double x) { return x; } //inches per second
constexpr long double operator"" _ftps(long double x) {return x * 12; } //feet per second
constexpr long double operator"" _tps(long double x) {return x * 24; } //field tiles per second

//Angular Velocity
constexpr long double operator"" _radps(long double x) {return x; } //radians per second
constexpr long double operator"" _degps(long double x) {return (x * PI) / 180.0; } //degrees per second
constexpr long double operator"" _rpm(long double x) {return (x * 2.0 * PI) / 60.0; } //rotations per minute

//Linear Acceleration
constexpr long double operator"" _inps2(long double x) {return x; } //inches per second squared


/**
 *      Data structs
 * 
 *      These are different ways to interpret odometry data
 */

/**
 * @struct Position
 * 
 * @brief this struct is used internally by the PositionTracker class
 */ 
typedef struct {
    //the orientation of the robot, in radians
    double orientation;

    //the x coord of the robot, in inches
    double x;

    //the y coord of the robot, in inches
    double y;

    //previous left drive encoder value
    double leftEncPrev;

    //previous right drive encoder value
    double rightEncPrev;
} Position;

/**
 * @struct Pose
 * 
 * @brief Used to describe a robot's position on the field and its orientation
 */
typedef struct {
    //the x coord of the robot, in inches
    double x;

    //the y coord of the robot, in inches
    double y;

    //the orientation of the robot, in radians
    double orientation;
} Pose;

/**
 * @struct Point
 * 
 * @brief Used to describe a robot's position on the field
 */
typedef struct {
    //the x coord of the robot, in inches
    double x;

    //the y coord of the robot, in inches
    double y;
} Point;

/**
 * @struct State
 * 
 * @brief Used to describe a robot's current state as well as its velocities
 */
typedef struct {
    //robot's current pose
    Pose robotPose;

    //robot's current velocity in x direction
    double velocityX;

    //robot's current velocity in y direction
    double velocityY;

    //robot's current angular velocity
    double velocityAngular;

    //robot's current velocity as a summation of x and y velocity components
    double velocityCombined;
} State;

/**
 * @struct ChassisConfig
 * 
 * @brief Used to describe a robot's chassis configuration, compatible with three encoder odometry
 */
typedef struct {
    //the left encoder wheel diameter in inches
    double lWheelDiameter;

    //the right encoder wheel diameter in inches
    double rWheelDiameter;

    //the middle encoder wheel diameter in inches
    double mWheelDiameter;

    //the perpendicular distance of the left wheel to the tracking center
    double lDistToCenter;
    
    //the perpendicular distance of the right wheel to the tracking center
    double rDistToCenter;
    
    //the perpendicular distance of the middle wheel to the tracking center
    double mDistToCenter;
} ChassisConfig;

/**
 * @class PositionTracker
 * 
 * @brief Two-encoder odometry used to track the robot's position on the field
 */
class PositionTracker{
private:
    //diameters of the tracking wheels, in inches
    double leftWheelDiameter;
    double rightWheelDiameter;

    //perpendicular distances of tracking wheels to center in inches
    double leftDistToCenter;
    double rightDistToCenter;

    //Position struct used to track data
    Position globalPosition;


public:
    /**
     * @brief Constructor for the PositionTracker class
     * 
     * @param initial The initial pose of the robot
     * @param robot The chassis configuration of the robot 
     */
    PositionTracker(Pose initial, ChassisConfig robot);

    /**
     * @brief Updates the robot's pose
     * 
     * @param left The current value of the left encoder
     * @param right The current value of the right encoder
     */ 
    void updatePose(double left, double right);

    /**
     * @brief Updates the robot's velocity
     */
    void updateVelocity();

    /**
     * @brief Manually sets the robot's pose
     * 
     * @param xNew The new x coordinate of the robot
     * @param yNew The new y coordinate of the robot
     * @param orientationNew The new orientation of the robot
     */ 
    void setPose(double xNew, double yNew, double orientationNew);

    /**
     * @brief Gets the robot's current x coordinate
     * 
     * @return The robot's x coordinate
     */
    double getX();

    /**
     * @brief Gets the robot's current y coordinate
     * 
     * @return The robot's y coordinate
     */
    double getY();

    /**
     * @brief Gets the robot's current orientation
     * 
     * @return The robot's orientation, in radians
     */ 
    double getTheta();

    /**
     * @brief Gets the robot's current position as a point
     * 
     * @return The robot's position as a point
     */
    Point getPoint();

    /**
     * @brief Gets the robot's current pose
     * 
     * @return The robot's current pose
     */
    Pose getPose();

    /**
     * @brief Gets the robot's current state
     * 
     * @return The robot's current state
     */
    State getState();

    /**
     * @brief Gets the point due north of the robot
     * Useful for turning to a specific direction.
     * @return The point due north of the robot
     */
    Point getDueNorth();
    
    /**
     * @brief Gets the point due south of the robot
     * Useful for turning to a specific direction.
     * @return The point due south of the robot
     */
    Point getDueSouth();
    
    /**
     * @brief Gets the point due east of the robot
     * Useful for turning to a specific direction.
     * @return The point due east of the robot
     */
    Point getDueEast();
    
    /**
     * @brief Gets the point due west of the robot
     * Useful for turning to a specific direction.
     * @return The point due west of the robot
     */
    Point getDueWest();

    /**
     * @brief Logs the robot's position to the microSD card
     */
    void logPosition();

    /**
     * @brief Logs the robot's pose to the microSD card
     */
    void logPose();
};
