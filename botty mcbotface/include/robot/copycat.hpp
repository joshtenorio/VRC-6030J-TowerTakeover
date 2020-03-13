/**
 * @file include/robot/copycat.hpp
 */ 
#pragma once


#define COPYCAT_MODE_POSITION     0
#define COPYCAT_MODE_T_VELOCITY   1
#define COPYCAT_MODE_A_VELOCITY   2

typedef struct {
    double position;
    int targetVelocity;
    double actualVelocity;
} MotorData;

//A note on copycat's .csv output:
//each line in the .csv is a new "step" to complete
//columns go like this:
//timestamp (ms), leftChassis<data>, rightChassis<data>, tilter<data>, rollers<data>
//where <data> is the data from MotorData struct, goes in like:
//position, targetVelocity, actualVelocity
class CopyCatController{
    private:
        std::vector<std::vector<int>> commands;
    public:
        CopyCatController();
        void copyDriver(okapi::Controller controller, std::string fileName);
        void initCopiedDriver(std::string fileName);
        void runDriver(std::string fileName, char mode);
    
};