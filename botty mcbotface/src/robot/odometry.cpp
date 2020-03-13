/**
 *  @file src/robot/odometry.cpp
 */
#include "main.h"

PositionTracker::PositionTracker(double initX, double initY, double initOrientation, double leftDiameter,
                                 double rightDiameter, double middleDiameter, double leftDist, double rightDist,
                                 double middleDist) {
    globalPosition.x = initX;
    globalPosition.y = initY;
    globalPosition.orientation = initOrientation;
    globalPosition.leftEncPrev, globalPosition.rightEncPrev, globalPosition.middleEncPrev = 0.0;

    

    leftWheelDiameter = leftDiameter;
    rightWheelDiameter = rightDiameter;
    middleWheelDiameter = middleDiameter;

    leftDistToCenter = leftDist;
    rightDistToCenter = rightDist;
    middleDistToCenter = middleDist;

    FILE* usd_file_write = fopen("/usd/positionTrackerConfig.txt", "a");
    fprintf(usd_file_write, "wheel diameters %f %f %f", leftWheelDiameter, rightWheelDiameter, middleWheelDiameter);
    fprintf(usd_file_write, "wheel distances %f %f %f\n", leftDistToCenter, rightDistToCenter, middleDistToCenter);
    fclose(usd_file_write);

}

PositionTracker::PositionTracker(Pose initial, ChassisConfig robot){
    globalPosition.x = initial.x;
    globalPosition.y = initial.y;
    globalPosition.orientation = initial.orientation;

    

    leftWheelDiameter = robot.lWheelDiameter;
    rightWheelDiameter = robot.rWheelDiameter;
    middleWheelDiameter = robot.mWheelDiameter;

    leftDistToCenter = robot.lDistToCenter;
    rightDistToCenter = robot.rDistToCenter;
    middleDistToCenter = robot.mDistToCenter;

    globalPosition.leftEncPrev, globalPosition.rightEncPrev, globalPosition.middleEncPrev = 0.0;
    FILE* usd_file_write = fopen("/usd/positionTrackerConfig.txt", "a");
    fprintf(usd_file_write, "wheel diameters %f %f %f", leftWheelDiameter, rightWheelDiameter, middleWheelDiameter);
    fprintf(usd_file_write, "wheel distances %f %f %f\n", leftDistToCenter, rightDistToCenter, middleDistToCenter);
    fclose(usd_file_write);
}

void PositionTracker::updatePosition(double left, double right, double middle) {
    //calculating change in robot's position since last position update, in inches
    double deltaL = (left - globalPosition.leftEncPrev) * (leftWheelDiameter * PI / 360.0);
    double deltaR = (right - globalPosition.rightEncPrev) * (rightWheelDiameter * PI / 360.0);
    double deltaM = (middle - globalPosition.middleEncPrev) * (middleWheelDiameter * PI / 360.0);

    //updating previous encoder values with the current values
    globalPosition.leftEncPrev = left;
    globalPosition.rightEncPrev = right;
    globalPosition.middleEncPrev = middle;

    double h; // the hypotenuse of the triangle formed by the robot's starting and ending position and the center of the circle it travels around
    double halfTheta; // half of the angle the robot turned
    double h2; // the same as h but using the middle wheel instead of the side wheels
    double deltaTheta = (deltaL - deltaR) / (leftDistToCenter + rightDistToCenter); //angle the robot turned, derived from arc length formula

    //check if robot has turned
    if (deltaTheta != 0){ //robot turned since last position update
        halfTheta = deltaTheta / 2.0;

        double radius = deltaR / deltaTheta; // the radius of the circle the robot travels around with the right side of the robot
        h = 2.0 * sin(halfTheta) * (radius + rightDistToCenter);

        double r2 = deltaM / deltaTheta; // the radius of the circle the robot travels around with the back of the robot
        h2 = 2.0 * sin(halfTheta) * (r2 + middleDistToCenter);

    }
    else{ //robot did not turn, deltaM is deltaX in local offset
        h = deltaR;
        halfTheta = 0;

        h2 = deltaM;
    }
    double avgTheta = halfTheta + globalPosition.orientation; // the global ending angle of the robot

    // update the global position by rotating vector

    //
    //Trying a thing to make robot move along x axis when orientation is 0, to match unit circle
    //
    globalPosition.y += h * sin(avgTheta);
    globalPosition.x += h * cos(avgTheta);

    globalPosition.y += h2 * cos(avgTheta); // cos(x) = cos(-x)
    globalPosition.x += h2 * -sin(avgTheta); // -sin(x) = sin(-x)

    // original code below
    /**
    globalPosition.y += h * cos(avgTheta);
    globalPosition.x += h * sin(avgTheta);

    globalPosition.y += h2 * -sin(avgTheta); // -sin(x) = sin(-x)
    globalPosition.x += h2 * cos(avgTheta); // cos(x) = cos(-x)
    **/
    globalPosition.orientation += deltaTheta;
}

void PositionTracker::updateVelocity(){
    /**
    int currTime = pros::millis();
    int deltaTime = currTime - globalState.lastUpdateTime;
    if(deltaTime >= 20){

    } **/
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

