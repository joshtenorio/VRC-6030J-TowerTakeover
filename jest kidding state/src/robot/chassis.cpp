/**
 *  @file src/robot/chassis.cpp
 */
#include "main.h"

//declaring motor groups
okapi::MotorGroup left({okapi::Motor(CHASSIS_LEFT_FRONT_MOTORPORT, true, okapi::AbstractMotor::gearset::blue),
                                     okapi::Motor(CHASSIS_LEFT_BACK_MOTORPORT, true, okapi::AbstractMotor::gearset::blue)});
okapi::MotorGroup right({okapi::Motor(CHASSIS_RIGHT_FRONT_MOTORPORT, false, okapi::AbstractMotor::gearset::blue),
                                     okapi::Motor(CHASSIS_RIGHT_BACK_MOTORPORT, false, okapi::AbstractMotor::gearset::blue)});
 
//declaring IMU sensor
pros::Imu imu(IMU_SENSORPORT);
double imuZeroValue = 0.0;

void Chassis::initialize(){
    left.tarePosition();
    right.tarePosition();
    imu.reset();
    pros::delay(2000);

    imuZeroValue = 0.0;
    JesterOS::write(1, "chassis initialized!");
}

void Chassis::setVelocity(float leftPercentage, float rightPercentage){
  left.controllerSet(leftPercentage);
  right.controllerSet(rightPercentage);
}

void Chassis::setVoltage(float leftVolt, float rightVolt){
    left.moveVoltage((int) (leftVolt * 12000.0));
    right.moveVoltage((int) (rightVolt * 12000.0));
}


float voltCap = 0.0; //voltage cap for slewing acceleration
void Chassis::setPID(double leftTarget, double rightTarget, float maxSpeed){
    float leftVolt = PID(left.getPosition(), leftTarget, PID_LEFT_CHASSIS);
    float rightVolt = PID(right.getPosition(), rightTarget, PID_RIGHT_CHASSIS);

    //sign variables are for multiplying volt cap by correct sign so robot travels in right direction
    int leftSign = 0;
    int rightSign = 0;
    if(leftVolt > 0) leftSign = 1;
    else leftSign = -1;
    if(rightVolt > 0) rightSign = 1;
    else rightSign = -1;

    //slew rate
    if(voltCap < maxSpeed) voltCap += 0.05;
    else voltCap = maxSpeed;

    if(abs(leftVolt) > voltCap) leftVolt = voltCap * (float) leftSign;
    if(abs(rightVolt) > voltCap) rightVolt = voltCap * (float) rightSign;

    Chassis::setVoltage(leftVolt, rightVolt);
}

void Chassis::updatePosition(PositionTracker robot){

}

void Chassis::turnTo(Point target, PositionTracker robot, double threshold, long double maxAngularVelocity){
    Point curr = robot.getPoint();
    double xDiff = target.x - curr.x;
    double yDiff = target.y - curr.y;

    double targetAngle = std::atan2(yDiff, xDiff);
   
}

void Chassis::turnTo(double targetAngle, PositionTracker robot, double threshold, long double maxAngularVelocity){
    double curr = robot.getTheta();
    //double target = closestEquivAngle(curr, targetAngle);
    double target = curr + targetAngle;
    double diff = abs(target) - abs(curr);
    

    while(abs(diff) > 5.0_deg){

        curr = robot.getTheta();

        //one of these will need to be a negative
        double leftOutput = PID(curr, target, PID_LEFT_CHASSIS);
        double rightOutput = -PID(curr, target, PID_RIGHT_CHASSIS);

        Chassis::setVelocity(leftOutput, rightOutput);

        diff = abs(target) - abs(curr);
        pros::delay(20);
    }
    
    Chassis::setVelocity(0.0, 0.0);
}

void Chassis::turnTo(double targetAngle, double threshold, int timeLimit){
    Chassis::resetGyro();
    double currAngle = Chassis::getRotation();
    double diff = abs(targetAngle) - abs(currAngle);
    
    const int startTime = pros::millis();
    int timer = pros::millis() - startTime;

    while(abs(diff) > threshold){
        currAngle = Chassis::getRotation();
        float left = PID(currAngle, targetAngle, PID_LEFT_CHASSIS);
        float right = -PID(currAngle, targetAngle, PID_RIGHT_CHASSIS);
        Chassis::setVoltage(left, right);

        diff = abs(targetAngle) - abs(currAngle);

        timer = pros::millis() - startTime;
        if(timer > timeLimit) break;

        pros::delay(20);
    }
    Chassis::setVoltage(0.0, 0.0);
}

void Chassis::driveTo(double leftTarget, double rightTarget, bool driveStraight, float maxSpeed, int timeLimit){

    left.tarePosition();
    right.tarePosition();
    Chassis::resetGyro();
    int startTime = pros::millis();
    double threshold = 180.0;
    double leftDiff = leftTarget - left.getPosition();
    double rightDiff = rightTarget - right.getPosition();

    int heading = Chassis::getRotation();

    int timer = pros::millis() - startTime;

    while(abs(leftDiff) > threshold || abs(rightDiff) > threshold){
        double leftCurrent = left.getPosition();
        double rightCurrent = right.getPosition();
        double currAngle = Chassis::getRotation();

        float leftSpeed = (float) constrain(PID(leftCurrent, leftTarget, PID_LEFT_CHASSIS), -maxSpeed, maxSpeed);
        float rightSpeed = (float) constrain(PID(rightCurrent, rightTarget, PID_RIGHT_CHASSIS), -maxSpeed, maxSpeed);

        leftDiff = leftTarget - left.getPosition();
        rightDiff = rightTarget - right.getPosition();

        if(driveStraight){
            leftSpeed -= PID(currAngle, heading, PID_TURN_CHASSIS);
            rightSpeed += PID(currAngle, heading, PID_TURN_CHASSIS);
            Chassis::setVoltage(leftSpeed, rightSpeed);
        }
        else Chassis::setPID(leftTarget, rightTarget, maxSpeed);



        timer = pros::millis() - startTime;
        if(timer > timeLimit) break;

        pros::delay(20);
    }
    Chassis::setVoltage(0.0, 0.0);
}


void Chassis::driveArc(float angle, float radius, bool isRight, bool isBackwards, float maxSpeed, float limit){
	left.tarePosition();
	right.tarePosition();
	Chassis::resetGyro();

    float velocityR, velocityL, radiusR, radiusL;

	if (isRight){
		radiusR = radius;
        radiusL = radius + 9.75; //9.75 in, update ROBOT_CHASSIS_WIDTH //ROBOT_CHASSIS_WIDTH;
        velocityL = (float) constrain(PID(Chassis::getRotation(), angle, PID_TURN_CHASSIS), -maxSpeed, maxSpeed);
        velocityR = (velocityL * radiusR) / radiusL;
	}
	else{
        radiusL = radius;
        radiusR = radius + 9.75;
        velocityR = (float) constrain(PID(Chassis::getRotation(), angle, PID_TURN_CHASSIS), -maxSpeed, maxSpeed);
        velocityL = (velocityR * radiusL) / radiusR;
	}
    float angleDiff = angle - Chassis::getRotation();

    const int startTime = pros::millis();
    int timer = pros::millis() - startTime;
	while(angleDiff > 1){
        Chassis::setVoltage(velocityL, velocityR);

        if(isRight){
            velocityL = (float) constrain(PID(Chassis::getRotation(), angle, PID_TURN_CHASSIS), -maxSpeed, maxSpeed);
            velocityR = (velocityL * radiusR) / radiusL;
        }
        else{
            velocityR = (float) constrain(PID(Chassis::getRotation(), angle, PID_TURN_CHASSIS), -maxSpeed, maxSpeed);
            velocityL = (velocityR * radiusL) / radiusR;
        }

        timer = pros::millis() - startTime;
        if(timer > (int) limit) break;

        angleDiff = angle - Chassis::getRotation();
		pros::delay(20);
	}
    Chassis::setVoltage(0.0, 0.0);
}


void Chassis::smartRobot(double leftTarget, double rightTarget, bool driveStraight, float maxDriveSpeed, float tilterTarget, float rollerSpeed, float armTarget, int timeLimit){
    left.tarePosition();
    right.tarePosition();
    
    int startTime = pros::millis();
    double threshold = 180.0;
    double leftDiff = leftTarget - left.getPosition();
    double rightDiff = rightTarget - right.getPosition();

    int timer = pros::millis() - startTime;

    while(abs(leftDiff) > threshold || abs(rightDiff) > threshold){
        double leftCurrent = left.getPosition();
        double rightCurrent = right.getPosition();

        float leftSpeed = (float) constrain(PID(leftCurrent, leftTarget, PID_LEFT_CHASSIS), -maxDriveSpeed, maxDriveSpeed);
        float rightSpeed = (float) constrain(PID(rightCurrent, rightTarget, PID_RIGHT_CHASSIS), -maxDriveSpeed, maxDriveSpeed);

        leftDiff = leftTarget - left.getPosition();
        rightDiff = rightTarget - right.getPosition();

        Chassis::setVoltage(leftSpeed, rightSpeed);

        Rollers::setVelocity(rollerSpeed);
        Tilter::setPID(tilterTarget);
        Arm::setPID(armTarget);

        timer = pros::millis() - startTime;
        if(timer > timeLimit) break;

        pros::delay(20);
    }
    Chassis::setVoltage(0.0, 0.0); 
}

double Chassis::getLeftPosition(){
    return left.getPosition();
}

double Chassis::getRightPosition(){
    return right.getPosition();
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

double Chassis::getRotation(){
    return imu.get_rotation() - imuZeroValue;
}

void Chassis::resetGyro(){
    imuZeroValue = imu.get_rotation();
}


std::vector<MotorData> Chassis::copyData(){
   
    std::vector<MotorData> dataOutput;
    dataOutput.push_back(MotorData{left.getPosition(), left.getTargetVelocity(), left.getActualVelocity()});

    dataOutput.push_back(MotorData{right.getPosition(), right.getTargetVelocity(), right.getActualVelocity()});
    return dataOutput;
}

void Chassis::runCopyCat(int velocityLeft, int velocityRight){
    float remappedLeft = (float) okapi::remapRange((double) velocityLeft, -600.0, 600.0, -1.0, 1.0);
    float remappedRight = (float) okapi::remapRange((double) velocityRight, -600.0, 600.0, -1.0, 1.0);
    Chassis::setVelocity(remappedLeft, remappedRight);
}

double Chassis::getLeftMotorTemperature(){
    return left.getTemperature();
}

double Chassis::getRightMotorTemperature(){
    return right.getTemperature();
}
