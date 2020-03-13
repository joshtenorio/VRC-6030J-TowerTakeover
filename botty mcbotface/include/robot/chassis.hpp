/**
 *  @file include/robot/chassis.hpp
 */
#pragma once
#include "robot/odometry.hpp"
#include "robot/copycat.hpp"

namespace Chassis{
  void initialize();
  void setVelocity(float leftPercentage, float rightPercentage);
  void driver(okapi::Controller controller);
  void updatePosition(PositionTracker robot);
  void turnTo(Point target, PositionTracker robot, double threshold, long double maxAngularVelocity);
  void turnTo(double targetAngle, PositionTracker robot, double threshold, long double maxAngularVelocity);
  void driveTo(double leftTarget, double rightTarget, float maxSpeed);

  double getLeftVelocity();
  double getRightVelocity();
  double getLeftAcceleration();
  double getRightAcceleration();
  std::vector<MotorData> copyData();
  void runCopyCat(int velocityLeft, int velocityRight);
}
