/**
 * @file src/robot/copycat.cpp
 */ 
#include "main.h"
#include <sstream>
#include <fstream>


CopyCatController::CopyCatController(){
    std::vector<std::vector<int>> commands;
}

int startTime;
bool copyingDriver = false;

void CopyCatController::copyDriver(okapi::Controller controller, std::string fileName){
    //pressed copy button
    if(controller.getDigital(okapi::ControllerDigital::left)){
        //if already copying, stop copying
        if(copyingDriver){
            copyingDriver = false;
            controller.setText(0, 0, "stopped copying auto");
            //delay statement to avoid starting copying again
            pros::delay(1500);
            controller.setText(0, 0, "standard driver control");
            return;
        }
        //start copying 
        else{
            controller.setText(0, 0, "Copying auto in 3");
            pros::delay(1000);
            controller.setText(0, 0, "Copying auto in 2");
            pros::delay(1000);
            controller.setText(0, 0, "Copying auto in 1");
            Chassis::initialize();
            Rollers::initialize();
            Tilter::initialize();
            pros::delay(1000);
            controller.setText(0, 0, "COPYING AUTO NOW");
            controller.rumble("-");

            startTime = pros::millis();
            copyingDriver = true;
        }
        
    }
    
    if(copyingDriver){
        std::ostringstream fileOSS;
        fileOSS << "/usd/" << fileName << ".csv";
        std::string fileIDTmp = fileOSS.str();
        const char* fileID = fileIDTmp.c_str();

        int currTime = pros::millis() - startTime;
        std::ostringstream newLine;
        
        //get updated data from subsystems
        std::vector<MotorData> chassisData = Chassis::copyData();
        std::vector<MotorData> rollerData = Rollers::copyData();
        std::vector<MotorData> tilterData = Tilter::copyData();

        //record timestamp
        newLine << currTime << ", ";
        //record chassis data
        newLine << chassisData.at(0).position << ", " << chassisData.at(0).targetVelocity << ", " << chassisData.at(0).actualVelocity << ", ";
        newLine << chassisData.at(1).position << ", " << chassisData.at(1).targetVelocity << ", " << chassisData.at(1).actualVelocity << ", ";
        //record tilter data
        newLine << tilterData.at(0).position << ", " << tilterData.at(0).targetVelocity << ", " << tilterData.at(0).actualVelocity << ", ";
        //record roller data
        newLine << rollerData.at(0).position << ", " << rollerData.at(0).targetVelocity << ", " << rollerData.at(0).actualVelocity << "\n";

        std::string outputTmp = newLine.str();

        
        const char* output = outputTmp.c_str();
        FILE* usd_file_write = fopen(fileID, "a");
        fprintf(usd_file_write, output);
        fclose(usd_file_write);
    }
    
}

void CopyCatController::initCopiedDriver(std::string fileName){
    std::ifstream f;
    std::ostringstream fileOSS;
    fileOSS << "/usd/" << fileName << ".csv";
    std::string fileID = fileOSS.str();

    f.open(fileID);
    //if file can not be opened then exit function
    if(!f.is_open()){
        //pros::lcd::print(5, "FILE CAN NOT BE OPENED");
        printf("FILE CAN NOT BE OPENED\n");
        return;
    }
    
    //start reading csv file into 2d vector of int, will need to cast to double for some modes
    std::string line, val;                  
    //std::vector<std::vector<int>> commands;
    //pros::lcd::print(5, "start reading csv file");
    while(std::getline(f, line)){       
        std::vector<int> v;             
        std::stringstream s (line);         
        while(getline(s, val, ',')){      
            v.push_back(std::stoi(val));
            pros::delay(1);
        }  
        commands.push_back(v);
        pros::delay(1);
    }
    //pros::lcd::print(5, "Finished reading file");
}

void CopyCatController::runDriver(std::string fileName, char mode){
    
    std::ifstream f;
    std::ostringstream fileOSS;
    fileOSS << "/usd/" << fileName << ".csv";
    std::string fileID = fileOSS.str();

    f.open(fileID);
    //if file can not be opened then exit function
    if(!f.is_open()){
        //pros::lcd::print(5, "FILE CAN NOT BE OPENED");
        printf("FILE CAN NOT BE OPENED\n");
        return;
    }
    
    //start reading csv file into 2d vector of int, will need to cast to double for some modes
    std::string line, val;                  
    std::vector<std::vector<int>> commands;
    //pros::lcd::print(5, "start reading csv file");
    while(std::getline(f, line)){       
        std::vector<int> v;             
        std::stringstream s (line);         
        while(getline(s, val, ',')){      
            v.push_back(std::stoi(val));
            pros::delay(1);
        }  
        commands.push_back(v);
        pros::delay(1);
    }
    //pros::lcd::print(5, "Finished reading file");
    
    const int startAutoTime = pros::millis();
    while(pros::competition::is_autonomous()){
        for(int i = 0; i < commands.size(); i++){
            int currTime = pros::millis() - startAutoTime;
            std::vector<int> command = commands.at(i);
            //if timestamp is past current time run the command
            if(command.at(0) >= currTime){
                //run command here
                Chassis::runCopyCat(command.at(2), command.at(5));
                Tilter::runCopyCat(command.at(8));
                Rollers::runCopyCat(command.at(11));
                //pros::lcd::print(5, "running auto");
            }
            pros::delay(20);
        }
        pros::delay(20);
    }
    //pros::lcd::print(5, "FInish auto");
    
}

void CopyCatController::runSmartDriver(std::string fileName, char mode){
    std::ifstream f;
    std::ostringstream fileOSS;
    fileOSS << "/usd/" << fileName << ".csv";
    std::string fileID = fileOSS.str();

    f.open(fileID);
    //if file can not be opened then exit function
    if(!f.is_open()){
        //pros::lcd::print(5, "FILE CAN NOT BE OPENED");
        printf("FILE CAN NOT BE OPENED\n");
        return;
    }
    const int startAutoTime = pros::millis();

    std::string line, val;                  
    while(pros::competition::is_autonomous() && std::getline(f, line)){

        //get next command
        std::vector<int> command;             
        std::stringstream s (line);         
        while(getline(s, val, ',')){      
            command.push_back(std::stoi(val));
            pros::delay(1);
        }  


        int currTime = pros::millis() - startAutoTime;
        //if timestamp is past current time run the command
        if(command.at(0) >= currTime){
            //run command here
            Chassis::runCopyCat(command.at(2), command.at(5));
            Tilter::runCopyCat(command.at(8));
            Rollers::runCopyCat(command.at(11));
            //pros::lcd::print(5, "running auto");
            }

        pros::delay(20);
    }
    //pros::lcd::print(5, "FInish auto");



}

