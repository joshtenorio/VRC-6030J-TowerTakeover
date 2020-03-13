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


typedef struct {
    double orientation;
    double x;
    double y;

    double leftEncPrev;
    double rightEncPrev;
    double middleEncPrev;

} Position;

typedef struct {
    double x;
    double y;
    double orientation;
} Pose;

typedef struct {
    double x;
    double y;
} Point;

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

typedef struct {
    double lWheelDiameter;
    double rWheelDiameter;
    double mWheelDiameter;

    double lDistToCenter;
    double rDistToCenter;
    double mDistToCenter;
} ChassisConfig;

class PositionTracker{
private:
    //distance of the tracking wheels, in inches
    double leftWheelDiameter;
    double rightWheelDiameter;
    double middleWheelDiameter;

    //distance of tracking wheels to center in inches
    double leftDistToCenter;
    double rightDistToCenter;
    double middleDistToCenter;

    Position globalPosition;


public:
    PositionTracker(double initX, double initY, double initOrientation, double leftDiameter, double rightDiameter,
                    double middleDiameter, double leftDist, double rightDist, double middleDist);

    PositionTracker(Pose initial, ChassisConfig robot);

    void updatePosition(double left, double right, double middle);
    void updateVelocity();

    double getX();
    double getY();
    double getTheta();
    Point getPoint();
    Pose getPose();
    State getState();

    Point getDueNorth();
    Point getDueSouth();
    Point getDueEast();
    Point getDueWest();

    void logPosition();
    void logPose();

};
