/**
 * @file include/robot/lift.hpp
 */
#pragma once

namespace Lift{
    void setVelocity(float speed);
    void driver(okapi::Controller master);
    void reset();
}
