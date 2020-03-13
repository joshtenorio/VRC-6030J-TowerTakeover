/**
 *  @file include/robot/chassis.hpp
 */
#pragma once
#include "robot/odometry.hpp"
#include "robot/copycat.hpp"

namespace Chassis{
  void initialize();
  void setVelocity(float leftPercentage, float rightPercentage);
  void setVoltage(float leftVolt, float rightVolt);
  void setPID(double leftTarget, double rightTarget, float maxSpeed);
  void updatePosition(PositionTracker robot);
  void turnTo(Point target, PositionTracker robot, double threshold, long double maxAngularVelocity);
  void turnTo(double targetAngle, PositionTracker robot, double threshold, long double maxAngularVelocity);
  void turnTo(double targetAngle, double threshold, int timeLimit);
  void driveTo(double leftTarget, double rightTarget, bool driveStraight, float maxSpeed, int timeLimit);
  void driveArc(float angle, float radius, bool isRight, bool isBackwards, float maxSpeed, float limit);
  void smartRobot(double leftTarget, double rightTarget, bool driveStraight, float maxDriveSpeed, float tilterTarget, float rollerSpeed, float armTarget, int timeLimit);

  double getLeftPosition();
  double getRightPosition();
  double getLeftVelocity();
  double getRightVelocity();
  double getLeftAcceleration();
  double getRightAcceleration();
  double getRotation();
  void resetGyro();
  
  std::vector<MotorData> copyData();
  void runCopyCat(int velocityLeft, int velocityRight);

  double getLeftMotorTemperature();
  double getRightMotorTemperature();
}
