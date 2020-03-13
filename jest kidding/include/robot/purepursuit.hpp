/**
 *  @file include/robot/purepursuit.hpp
 */
#pragma once

typedef struct {
    //corresponding indexes in each vector references to the same point
    std::vector<Point> points;
    std::vector<double> distanceToPoint;
    std::vector<double> targetVelocity;

} Path;

class PurePursuitController{
private:
    std::array<Path, 5> listPaths;
    PositionTracker tracker;
    float maxAcceleration;
    double trackWidth;
    double kv;
    double ka;
    double kp;

    double rateLimiter(double input, double maxRateOfChange, char pathIndex);
    std::vector<Point> injectPoints(std::initializer_list<Point> waypoints, int spacing);
    std::vector<Point> smoothPath(std::vector<Point> path, int runs);
    Path computeVelocities(std::vector<Point> path, float maxVelocity);
    double computeTargetVelocity(std::vector<Point> path, char pathIndex);
    double findIntersection(Point initial, Point final, double lookaheadDistance);
    Point findLookaheadPoint(std::vector<Point> path, char pathIndex, double lookaheadDistance);
    double computeTargetAcceleration(double targetVelocity, bool isLeft, char pathIndex);

public:

    PurePursuitController(Pose initial, ChassisConfig robot, float maxAccel, double chassisWidth, double cv, double ca, double cp);
    
    void generatePath(std::initializer_list<Point> waypoints, int smoothFactor, float maxVelocity, char index);
    void preCompilePath(std::initializer_list<Point> waypoints, int smoothFactor, float maxVelocity, std::string &fileName);
    void debugPath(char index, std::string fileName);
    void runCompiledPath(std::string &fileName, float lookAheadDistance, bool backwards=false);
    void runPath(char index, float lookAheadDistance, bool backwards=false);
    void deletePath(char index);



};
