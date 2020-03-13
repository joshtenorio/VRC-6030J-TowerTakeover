/**
 *  @file src/robot/purepursuit.cpp
 */


#include "main.h"
#include <sstream>
#include <fstream>

PurePursuitController::PurePursuitController(Pose initial, ChassisConfig robot, float maxAccel, double chassisWidth, double cv, double ca, double cp): tracker(PositionTracker(initial, robot)){

    maxAcceleration = maxAccel;
    trackWidth = chassisWidth;

    kv = cv;
    ka = ca;
    kp = cp;
}


std::array<int, 5> prevTimeRateLimited = {0, 0, 0, 0, 0};
std::array<double, 5> prevRateLimiterOutput = {0.0, 0.0, 0.0, 0.0, 0.0};
std::array<double, 5> rateLimiterOutput = {0.0, 0.0, 0.0, 0.0, 0.0};
double PurePursuitController::rateLimiter(double input, double maxRateOfChange, char pathIndex){
    double currTime = (double) pros::millis();
    double maxChange = (currTime - (double) prevTimeRateLimited.at(pathIndex)) * maxRateOfChange;
    rateLimiterOutput.at(pathIndex) += constrain(input - prevRateLimiterOutput.at(pathIndex), -maxChange, maxChange);

    prevRateLimiterOutput.at(pathIndex) = rateLimiterOutput.at(pathIndex);
    prevTimeRateLimited.at(pathIndex) = currTime;

    return rateLimiterOutput.at(pathIndex);
}


std::vector<Point> PurePursuitController::injectPoints(std::initializer_list<Point> waypoints, int spacing){
    std::vector<Point> givenPoints = waypoints;
    std::vector<Point> newPath;
    int numLineSegments = givenPoints.size() - 1;
    
    pros::lcd::print(7, "injecting points...");
    for(int i = 0; i < numLineSegments; i++){
        Point p = givenPoints.at(i);
        Point u = givenPoints.at(i + 1);

        Vector initLineSegment = Vector{u.x - p.x, u.y - p.y};

        //initLineSegment.x = u.x - p.x;
        //initLineSegment.y = u.y - p.y;

        int numPointsThatFit = ceil(vectorMagnitude(initLineSegment) / spacing);

        Vector finalLineSegment = vectorNormalize(initLineSegment);
        finalLineSegment.x *= spacing;
        finalLineSegment.y *= spacing;

        for(int j = 0; j < numPointsThatFit; j++){

            Point injectedPoint = Point{p.x + finalLineSegment.x * ((double) j),
                                        p.y + finalLineSegment.y * ((double) j)};

            newPath.push_back(injectedPoint);

            pros::delay(10);
        }

        pros::delay(10);
    }

    Point lastPoint = givenPoints.at(givenPoints.size() - 1);

    newPath.push_back(lastPoint);
    

    return newPath;
}

std::vector<Point> PurePursuitController::smoothPath(std::vector<Point> path, int runs){
    std::vector<Point> newPath = path;
    std::vector<Point> tmpPath = path;

    for(int n = 0; n < runs; n++){
        for(int i = 1; i < tmpPath.size() - 1; i++){
            double xMid = (tmpPath.at(i - 1).x + tmpPath.at(i + 1).x) / 2.0;
            double yMid = (tmpPath.at(i - 1).y + tmpPath.at(i + 1).y) / 2.0;

            Point midPoint = Point{xMid, yMid};
            newPath[i] = midPoint;
            pros::delay(10);
        }
        pros::delay(10);
        tmpPath = newPath;
    }
    
    
    return newPath;
}

Path PurePursuitController::computeVelocities(std::vector<Point> path, float maxVelocity){
    //compute distances from beginning of path to each point
    std::vector<double> distanceToEachPoint;
    distanceToEachPoint.push_back(0.0);
    for(int i = 1; i < path.size(); i++){
        double distToPoint = distanceToEachPoint.at(i - 1) + distanceFormula(path.at(i), path.at(i - 1));
        distanceToEachPoint.push_back(distToPoint);
        pros::delay(10);
    }

    //compute curvature at each point
    std::vector<double> curvatureAtPoint;
    curvatureAtPoint.push_back(0.0);
    for(int i = 1; i < path.size() - 1; i++){
        //the point before the point whose curvature is being computed
        Point q = path.at(i - 1);
        double x2 = q.x;
        double y2 = q.y;
        //the point whose curvature is being computed
        Point p = path.at(i); 
        double x1 = p.x + 0.001; //adding a small constant to avoid dividing by zero when x1 = x2
        double y1 = p.y;
        //the point after the point whose curvature is being computed
        Point r = path.at(i + 1); 
        double x3 = r.x;
        double y3 = r.y;

        double k1 = 0.5 * ((x1 * x1) + (y1 * y1) - (x2 * x2) - (y2 * y2)) / (x1 - x2);
        double k2 = (y1 - y2) / (x1 - x2);
        double bTerm = 0.5 * (squareNum(x2) - 2 * x2 * k1 + squareNum(y2) - squareNum(x3) + 2.0 * x3 * k1 - squareNum(y3)) / (x3 * k2 - y3 + y2 - x2 * k2);
        double aTerm = k1 - k2 * bTerm;

        double radius;
        if(std::isnan(sqrt(squareNum(x1 - aTerm) + squareNum(y1 - bTerm)))){
            //if formula is NaN, curvature = 0.0
            curvatureAtPoint.push_back(0.0);
        }
        else{
            //else, curvature is 1/r
            radius = sqrt(squareNum(x1 - aTerm) + squareNum(y1 - bTerm));
            curvatureAtPoint.push_back(1.0 / radius);
        }
        pros::delay(10);
    }
    curvatureAtPoint.push_back(0.0);
    
    std::vector<double> maxVelocityOfPoint;
    //compute max velocity based on curvature of path
    for(int i = 0; i < curvatureAtPoint.size(); i++){
        //check if curvature = zero to avoid dividing by zero
        if(curvatureAtPoint.at(i)){ //curvature is nonzero
            maxVelocityOfPoint.push_back(min((double) maxVelocity, 3.0 / curvatureAtPoint.at(i)));
        }
        else{ //curvature is zero
            maxVelocityOfPoint.push_back((double) maxVelocity);
        }
        pros::delay(10);
    }

    //adjust max velocity for robot's max acceleration
    maxVelocityOfPoint.at(maxVelocityOfPoint.size() - 1) = 0.0;
    for(int i = maxVelocityOfPoint.size() - 2; i > -1; i--){
        Point final = path.at(i + 1);
        Point initial = path.at(i);
        double distance = distanceFormula(final, initial);
        maxVelocityOfPoint.at(i) = min(maxVelocityOfPoint.at(i), sqrt(squareNum(maxVelocityOfPoint.at(i + 1)) + 2.0 * maxAcceleration * distance));
        pros::delay(10);
    }
    
    Path output;
    output.points = path;
    output.distanceToPoint = distanceToEachPoint;
    output.targetVelocity = maxVelocityOfPoint;
    return output;
}


std::array<int, 5> indexOfClosestPoint = {0, 0, 0, 0, 0};
double PurePursuitController::computeTargetVelocity(std::vector<Point> path, char pathIndex){
    int prevIndex = indexOfClosestPoint.at(pathIndex);
    std::vector<double> distanceToPoints;
    
    //compute distance from current robot position to all the points in the path
    for(int i = prevIndex; i < path.size(); i++){
        Point currPosition = tracker.getPoint();
        double distance = distanceFormula(currPosition, path.at(i));
        distanceToPoints.push_back(distance);
        pros::delay(10);
    }

    //check if this format works
    double closestDistance = distanceToPoints.at(0);
    for(int i = 0; i < distanceToPoints.size(); i++){
        double distance = distanceToPoints.at(i);

        if(distance < closestDistance){
            closestDistance = distance;

            //specifically setting the index make sure this math is correct
            indexOfClosestPoint.at(pathIndex) = prevIndex + i;
        }
        pros::delay(10);
    }
    Point closestPoint = path.at(indexOfClosestPoint.at(pathIndex));

    return listPaths.at(pathIndex).targetVelocity.at(indexOfClosestPoint.at(pathIndex)); 
}


double PurePursuitController::findIntersection(Point initial, Point final, double lookaheadDistance){
    Point robot = tracker.getPoint();
    Vector direction = Vector{final.x - initial.x, final.y - initial.y};
    Vector robotToLine = Vector{initial.x - robot.x, initial.y - robot.y};

    double aTerm = vectorDotProduct(direction, direction);
    double bTerm = 2.0 * vectorDotProduct(direction, robotToLine);
    double cTerm = vectorDotProduct(robotToLine, robotToLine) - squareNum(lookaheadDistance);
    double discriminant = squareNum(bTerm) - 4.0 * aTerm * cTerm;
    double tFirstIntersection = 0.0;
    double tSecondIntersection = 0.0;
    if(discriminant < 0){
        return 4.0; //no intersection
    }
    else{
        discriminant = sqrt(discriminant);
        tFirstIntersection = (-bTerm - discriminant) / (2.0 * aTerm);
        tSecondIntersection = (-bTerm + discriminant) / (2.0 * aTerm);

        if(tFirstIntersection >= 0.0 && tFirstIntersection <= 1.0){
            return tFirstIntersection;
        }
        if(tSecondIntersection >= 0.0 && tSecondIntersection <= 1.0){
            return tSecondIntersection;
        }
        return 4.0; //no intersection
    }
}

std::array<Point, 5> prevLookaheadPoints = {Point{0.0, 0.0}, Point{0.0, 0.0}, Point{0.0, 0.0}, Point{0.0, 0.0}, Point{0.0, 0.0}};
std::array<double, 5> prevFractionalIndexes = {0.0, 0.0, 0.0, 0.0, 0.0};
Point PurePursuitController::findLookaheadPoint(std::vector<Point> path, char pathIndex, double lookaheadDistance){
    std::vector<double> tValues;
    std::vector<double> fractionalIndexes;
    std::vector<Point> intersectionPoints;
    //calculate intersections and fractional indexes for every line segment/point in path
    for(int i = 1; i < path.size(); i++){
        Point initial = path.at(i - 1);
        Point final = path.at(i);
        double tValue = findIntersection(initial, final, lookaheadDistance);

        if(tValue == 4.0){
            //do nothing because an invalid intersection has been found
        }
        else{
            tValues.push_back(tValue);
            fractionalIndexes.push_back(tValue + (i - 1));
            Vector direction = Vector{final.x - initial.x, final.y - initial.y};
            Point intersection = Point{initial.x + direction.x * tValue, initial.y + direction.y * tValue};
            intersectionPoints.push_back(intersection);
        }
        pros::delay(10);
    }

    //determine first valid intersection
    bool newIntersectionAvailable;
    double currFractionalIndex = -1;
    Point lookaheadPoint = prevLookaheadPoints.at(pathIndex);
    for(int i = 0; i < path.size(); i++){
        double tValue = tValues.at(i);
        double currFractionalIndex = fractionalIndexes.at(i);
        //check if current fractional index is larger than the previous fractional index, and check if t is in [-1, 1]
        if(currFractionalIndex > prevFractionalIndexes.at(pathIndex) && (-1.0 <= tValue && tValue <= 1.0)){
            newIntersectionAvailable = true;
            lookaheadPoint = intersectionPoints.at(i);       
            break;
        }
        pros::delay(10);
    }

    //update previous lookahead point and fractional index
    prevFractionalIndexes.at(pathIndex) = currFractionalIndex;
    
    prevLookaheadPoints.at(pathIndex) = lookaheadPoint;
    //hopefully this works lol
    return lookaheadPoint;
}

std::array<double, 5> prevLeftTargetVelocities = {0.0, 0.0, 0.0, 0.0, 0.0};
std::array<double, 5> prevRightTargetVelocities = {0.0, 0.0, 0.0, 0.0, 0.0};
std::array<int, 5> prevLeftTime = {0, 0, 0, 0, 0};
std::array<int, 5> prevRightTime = {0, 0, 0, 0, 0};
double PurePursuitController::computeTargetAcceleration(double targetVelocity, bool isLeft, char pathIndex){
    double currTime = (double) pros::millis();
    double deltaV;
    double deltaT;
    //check if calculating for left or right side
    if(isLeft){ //is left
        deltaV = targetVelocity - prevLeftTargetVelocities.at(pathIndex);
        deltaT = currTime - (double) prevLeftTime.at(pathIndex) / (1000.0);
    }
    else{ //else, is right
        deltaV = targetVelocity - prevRightTargetVelocities.at(pathIndex);
        deltaT = currTime - (double) prevRightTime.at(pathIndex) / 1000.0;
    }

    double acceleration = deltaV / (deltaT);
    return acceleration;
}


void PurePursuitController::generatePath(std::initializer_list<Point> waypoints, int smoothFactor, float maxVelocity, char index){
    listPaths.at(index) = computeVelocities(smoothPath(injectPoints(waypoints, 1), smoothFactor), maxVelocity);
}


void PurePursuitController::preCompilePath(std::initializer_list<Point> waypoints, int smoothFactor, float maxVelocity, std::string &fileName){
    //WIP
    //save path to sd card
}


///print path data to a file on the usd card
void PurePursuitController::debugPath(char index, std::string fileName){
    Path path = listPaths.at(index);

    std::ostringstream fileOSS;
    fileOSS << "/usd/" << fileName << "PathLog" << ".csv";
    std::string fileIDTmp = fileOSS.str();
    const char* fileID = fileIDTmp.c_str();

    FILE* usd_file_write = fopen(fileID, "a");
    //print path to first 2 columns
    for(int i = 0; i < path.points.size(); i++){
        Point p = path.points.at(i);
        double distance = path.distanceToPoint.at(i);
        double velocity = path.targetVelocity.at(i);
        fprintf(usd_file_write, "%f, %f, %f, %f, %d\n", p.x, p.y, distance, velocity, i);
        pros::delay(10);
    }

    fclose(usd_file_write);
    pros::lcd::print(7, "path printed to sd card");
}


void PurePursuitController::runCompiledPath(std::string &fileName, float lookAheadDistance, bool backwards){

}


void PurePursuitController::runPath(char index, float lookAheadDistance, bool backwards){
    
    //finding distance to last point in the path, used for stop condition
    Point last = listPaths.at(index).points.at(listPaths.at(index).points.size() - 1);
    double distanceToLastPoint = distanceFormula(tracker.getPoint(), last);
    
    while(distanceToLastPoint > 3.0_in){
        tracker.updatePose(Chassis::getLeftPosition(), Chassis::getRightPosition());

        //find closest point's target velocity and lookahead point
        double velocityTarget = computeTargetVelocity(listPaths.at(index).points, index);
        Point lookaheadPoint = findLookaheadPoint(listPaths.at(index).points, index, lookAheadDistance);

        //calculate arc from robot position to lookahead point
        //getting robot position and orientation
        double angleRobot = tracker.getTheta();
        double xRobot = tracker.getX();
        double yRobot = tracker.getY();

        //point slope form to find "robot line"
        // yLook - yRobot = angleRobot(xLook - xRobot)
        //converting to form ax + by + c...
        double aTerm = -tan(angleRobot);
        double bTerm = 1.0;
        double cTerm = tan(angleRobot) * (xRobot - yRobot);

        //point line distance formula...
        double numerator = abs(aTerm * lookaheadPoint.x + bTerm * lookaheadPoint.y + cTerm);
        double denominator = sqrt(squareNum(aTerm) + squareNum(bTerm));
        double robotLineX = numerator/denominator;
        //calculate curvature, see journal for more equation manipulation
        double curvatureMagnitude = (2.0 * robotLineX) / squareNum(lookAheadDistance);

        //calculate side to turn to for curvatureSign
        double curvatureSign = (int) findSign(sin(angleRobot) * (lookaheadPoint.x - xRobot) - cos(angleRobot) * (lookaheadPoint.y - yRobot));

        double curvature = curvatureMagnitude * curvatureSign;

        //calculate wheel velocities from tank drive kinematics
        // V = (L + R) / 2
        // W = (L - R) / T
        // V = W / C
        //see journal for equation manipulation
        
        //adjust target velocity to obey robot's max acceleration
        double adjustedVelocityTarget = rateLimiter(velocityTarget, maxAcceleration, index);

        //calculate target velocities and accelerations
        double velocityLeftTarget = (adjustedVelocityTarget * (2.0 + curvature * trackWidth)) / 2.0;
        double velocityRightTarget = (adjustedVelocityTarget * (2.0 - curvature * trackWidth)) / 2.0;

        double accelerationLeftTarget = computeTargetAcceleration(velocityLeftTarget, true, index);
        double accelerationRightTarget = computeTargetAcceleration(velocityRightTarget, false, index);

        //get current wheel velocities
        double velocityLeftCurrent = Chassis::getLeftVelocity();
        double velocityRightCurrent = Chassis::getRightVelocity();
 
        //send wheel velocities to chassis
        double leftFeedForwardTerm = kv * velocityLeftTarget + ka * accelerationLeftTarget;
        double rightFeedForwardTerm = kv * velocityRightTarget + ka * accelerationRightTarget;

        double leftFeedBackTerm = kp * (velocityLeftTarget - velocityLeftCurrent);
        double rightFeedBackTerm = kp * (velocityRightTarget - velocityRightCurrent);

        double leftOutput = leftFeedForwardTerm + leftFeedBackTerm;
        double rightOutput = rightFeedForwardTerm + rightFeedBackTerm;

        if(backwards){ //if running path backwards, set output velocities to negative
            leftOutput *= -1;
            rightOutput *= -1;
        }
        Chassis::setVoltage(leftOutput, rightOutput);

        distanceToLastPoint = distanceFormula(tracker.getPoint(), last);
        pros::delay(5);
    }
}


void PurePursuitController::deletePath(char index){

}


