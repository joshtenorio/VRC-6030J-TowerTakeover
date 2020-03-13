/**
 *  @file src/robot/chassis.cpp
 */
#include "main.h"

okapi::Motor left(CHASSIS_LEFT_MOTORPORT);
okapi::Motor right(-CHASSIS_RIGHT_MOTORPORT);
okapi::Motor middle(CHASSIS_MIDDLE_MOTORPORT);

auto leftController = okapi::IterativeControllerFactory::posPID(0.5, 0.0, 0.2);
auto rightController = okapi::IterativeControllerFactory::posPID(0.5, 0.0, 0.2);
auto orientationController = okapi::IterativeControllerFactory::posPID(0.5, 0.0, 0.2);

okapi::ADIEncoder rightEnc(RIGHT_ODOM_SENSOR_CONFIG);
okapi::ADIEncoder leftEnc(LEFT_ODOM_SENSOR_CONFIG);
okapi::ADIEncoder midEnc(MIDDLE_ODOM_SENSOR_CONFIG);

void Chassis::initialize(){
    left.tarePosition();
    right.tarePosition();
    middle.tarePosition();

    rightEnc.reset();
    leftEnc.reset();
    midEnc.reset();
}

void Chassis::setVelocity(float leftPercentage, float rightPercentage){
  left.controllerSet(leftPercentage);
  right.controllerSet(rightPercentage);

  middle.controllerSet((leftPercentage + rightPercentage) / 2);
}

void Chassis::driver(okapi::Controller master){
    float left = okapi::deadband(master.getAnalog(okapi::ControllerAnalog::leftY), -0.1, 0.1);
    float right = okapi::deadband(master.getAnalog(okapi::ControllerAnalog::rightY), -0.1, 0.1);

    //by cubing the joystick outputs we can have more precise chassis control
    //pushing ~80% on the joystick will output ~50% chassis speed as opposed to ~80% speed
    //this allows us to be more precise with driving without losing top speed
    float leftOutput = left * left * left;
    float rightOutput = right * right * right;
    Chassis::setVelocity(leftOutput, rightOutput);
}

void Chassis::updatePosition(PositionTracker robot){
    pros::lcd::print(3, "left %f", leftEnc.get());
    pros::lcd::print(4, "right %f", rightEnc.get());
    pros::lcd::print(5, "middle %f", midEnc.get());
    robot.updatePosition(leftEnc.get(), rightEnc.get(), midEnc.get());
}

void Chassis::turnTo(Point target, PositionTracker robot, double threshold, long double maxAngularVelocity){
    Point curr = robot.getPoint();
    double xDiff = target.x - curr.x;
    double yDiff = target.y - curr.y;

    double targetAngle = std::atan2(yDiff, xDiff);
    pros::lcd::print(4, "angle to turn %f", targetAngle);
}

void Chassis::turnTo(double targetAngle, PositionTracker robot, double threshold, long double maxAngularVelocity){
    double curr = robot.getTheta();
    //double target = closestEquivAngle(curr, targetAngle);
    double target = curr + targetAngle;
    double diff = abs(target) - abs(curr);
    pros::lcd::print(4, "angle to turn %f", targetAngle);

    while(abs(diff) > 5.0_deg){
        robot.updatePosition(leftEnc.get(), rightEnc.get(), midEnc.get());
        curr = robot.getTheta();

        //one of these will need to be a negative
        double leftOutput = PID(curr, target, 40.0);
        double rightOutput = -PID(curr, target, 40.0);

        Chassis::setVelocity(leftOutput, rightOutput);

        diff = abs(target) - abs(curr);
        pros::delay(20);
    }
    pros::lcd::print(4, "finished turning");
    Chassis::setVelocity(0.0, 0.0);
}

void Chassis::driveTo(double leftTarget * 29, double rightTarget * 29, float maxSpeed){
    left.tarePosition();
    right.tarePosition();
    middle.tarePosition();

    double threshold = 180.0;
    double leftDiff = leftTarget - left.getPosition() / 29;
    double rightDiff = rightTarget - right.getPosition() / 29;

    while(abs(leftDiff) > threshold && abs(rightDiff) > threshold){
        double leftCurrent = left.getPosition() / 29;
        double rightCurrent = right.getPosition() / 29;

        float leftSpeed = (float) constrain(PID(leftCurrent, leftTarget, 0.5), -maxSpeed, maxSpeed);
        float rightSpeed = (float) constrain(PID(rightCurrent, rightTarget, 0.5), -maxSpeed, maxSpeed);

        leftDiff = leftTarget - left.getPosition();
        rightDiff = rightTarget - right.getPosition();

        Chassis::setVelocity(leftSpeed, rightSpeed);
        pros::lcd::print(5, "leftspeed %f rightspeed %f", leftSpeed, rightSpeed);
        pros::lcd::print(6, "%f left diff %f right diff", leftDiff, rightDiff);
        pros::delay(20);
    }
    Chassis::setVelocity(0.0, 0.0);
}


double prevLeftPos = 0.0;
double prevLeftPosTime = 0.0;
double Chassis::getLeftVelocity(){
    double currPos = left.getPosition();
    double currTime = (double) pros::millis();

    double deltaPos = (currPos - prevLeftPos) * (3.25 * PI / 360.0);
    double deltaTime = (currTime - prevLeftPosTime) / (1000.0);
    double velocity = (deltaPos / deltaTime);



    prevLeftPos = currPos;
    prevLeftPosTime = currTime;

    return velocity;
}

double prevRightPos = 0.0;
double prevRightPosTime = 0.0;
double Chassis::getRightVelocity(){
    double currPos = right.getPosition();
    double currTime = (double) pros::millis();

    double deltaPos = (currPos - prevRightPos) * (3.25 * PI / 360.0);
    double deltaTime = (currTime - prevRightPosTime) / (1000.0);
    double velocity = deltaPos / deltaTime;

    prevRightPos = currPos;
    prevRightPosTime = currTime;
    return velocity;
}

double prevLeftVel = 0.0;
double prevLeftVelTime = 0.0;
double Chassis::getLeftAcceleration(){
    double currVel = Chassis::getLeftVelocity();
    double currTime = (double) pros::millis();

    double deltaV = (currVel - prevLeftVel);
    double deltaTime = (currTime - prevLeftVelTime) / (1000.0);
    double acceleration = deltaV / (deltaTime);

    prevLeftVel = currVel;
    prevLeftVelTime = currTime;
    return acceleration;
}

double prevRightVel = 0.0;
double prevRightVelTime = 0.0;
double Chassis::getRightAcceleration(){
    double currVel = Chassis::getLeftVelocity();
    double currTime = (double) pros::millis();

    double deltaV = (currVel - prevRightVel);
    double deltaTime = (currTime - prevRightVelTime) / (1000.0);
    double acceleration = deltaV / (deltaTime);

    prevRightVel = currVel;
    prevRightVelTime = currTime;
    return acceleration;
}


std::vector<MotorData> Chassis::copyData(){

    std::vector<MotorData> dataOutput;
    dataOutput.push_back(MotorData{left.getPosition(), left.getTargetVelocity(), left.getActualVelocity()});

    dataOutput.push_back(MotorData{right.getPosition(), right.getTargetVelocity(), right.getActualVelocity()});
    return dataOutput;
}

void Chassis::runCopyCat(int velocityLeft, int velocityRight){
    float remappedLeft = (float) okapi::remapRange((double) velocityLeft, -200.0, 200.0, -1.0, 1.0);
    float remappedRight = (float) okapi::remapRange((double) velocityRight, -200.0, 200.0, -1.0, 1.0);
    Chassis::setVelocity(remappedLeft, remappedRight);
}
