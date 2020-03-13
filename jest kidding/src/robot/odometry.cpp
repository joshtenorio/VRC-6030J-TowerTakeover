/**
 *  @file src/robot/odometry.cpp
 */
#include "main.h"

PositionTracker::PositionTracker(Pose initial, ChassisConfig robot){
    globalPosition.x = initial.x;
    globalPosition.y = initial.y;
    globalPosition.orientation = initial.orientation;

    
    leftWheelDiameter = robot.lWheelDiameter;
    rightWheelDiameter = robot.rWheelDiameter;


    leftDistToCenter = robot.lDistToCenter;
    rightDistToCenter = robot.rDistToCenter;

    globalPosition.leftEncPrev, globalPosition.rightEncPrev;

}

void PositionTracker::updatePose(double left, double right) {
    //calculating change in robot's position since last position update, in inches
    double deltaL = (left - globalPosition.leftEncPrev) * (leftWheelDiameter * PI / 360.0);
    double deltaR = (right - globalPosition.rightEncPrev) * (rightWheelDiameter * PI / 360.0);

    //calculating average distance travelled since last position update, in inches
    double avgDist = (deltaL + deltaR) / 2.0;

    //updating previous encoder values with the current values
    globalPosition.leftEncPrev = left;
    globalPosition.rightEncPrev = right;

    double deltaTheta = (deltaL - deltaR) / (leftDistToCenter + rightDistToCenter); //angle the robot turned, derived from arc length formula

    //updating global angle
    globalPosition.orientation += deltaTheta;

    //Updating x and y coords using right triangle trigonometry
    globalPosition.x += avgDist * cos(globalPosition.orientation);
    globalPosition.y += avgDist * sin(globalPosition.orientation);
}

void PositionTracker::updateVelocity(){
    /**
    int currTime = pros::millis();
    int deltaTime = currTime - globalState.lastUpdateTime;
    if(deltaTime >= 20){

    } **/
}

void PositionTracker::setPose(double xNew, double yNew, double orientationNew){
    globalPosition.x = xNew;
    globalPosition.y = yNew;
    globalPosition.orientation = orientationNew;
}

double PositionTracker::getX(){
    return globalPosition.x;
}

double PositionTracker::getY(){
    return globalPosition.y;
}

double PositionTracker::getTheta(){
    return globalPosition.orientation;
}

Point PositionTracker::getPoint(){
    return Point{globalPosition.x, globalPosition.y};
}

Point PositionTracker::getDueNorth(){
    return Point{globalPosition.x, 72.0};
}

Point PositionTracker::getDueSouth(){
    return Point{globalPosition.x, -72.0};
}

Point PositionTracker::getDueEast(){
    return Point{72.0, globalPosition.y};
}

Point PositionTracker::getDueWest(){
    return Point{-72.0, globalPosition.y};
}

void PositionTracker::logPosition(){
    FILE* usd_file_write = fopen("/usd/logPosition2.csv", "a");
    fprintf(usd_file_write, "%f, %f\n", this->getX(), this->getY());
    fclose(usd_file_write);
}

void PositionTracker::logPose(){
    FILE* usd_file_write = fopen("/usd/logState.csv", "a");
    fprintf(usd_file_write, "%f, %f, %f\n", this->getX(), this->getY(), this->getTheta());
    fclose(usd_file_write);
}

