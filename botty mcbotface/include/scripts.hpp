/**
 * @file include/scripts.hpp
 */ 
#pragma once

namespace Driver{
    void deploy(okapi::Controller master);
    void towerDrop(okapi::Controller master);
}

namespace Autonomous{
    void oneCube();
    void blueRow();
    void redRow();
}