/**
 * @file include/robot/arm.hpp
 */
#pragma once

namespace Arm{
    void setVelocity(float speed);
    void setVoltage(float liftVolt);
    void reset();
    double getPosition();
    int getLimitSwitch();
    void setPID(float target);
    void setAngle(float angle);
    double getMotorTemperature();
}
