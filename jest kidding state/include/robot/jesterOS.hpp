/**
 *  @file include/robot/jesterOS.hpp
 *  @brief Contains prototypes for JesterOS
 * 
 * @author Josh Tenorio, ACP Robotics
 * 
 */
#pragma once
#include "main.h"
#include "pros/apix.h"

/**
 * @brief controls the jesterOS utilities
 * @author Josh Tenorio, ACP Robotics
 * 
 * the JesterOS class, built using LVGL, is an autonomous
 * selector and debug menu. The autonomous selector selects
 * autonomi for competition and the debug menu will act like
 * the LLEMU.
 * 
 * DISCLAIMER: This isn't actually a real OS, just a GUI :)
 * NOTE: The write() function is adapted from 5090Z's 2018-2019 code. Their released code
 * was last updated on 11/17/2019, retrieved on 1/5/2020, and can be found at
 * https://github.com/timeconfusing/OkapiLVGL/blob/master/src/robot_gui.cpp
 * 
 */

namespace JesterOS{

      

        /**
         * @brief the callback function for selecting an autonomous program to run
         * 
         * This function is used to set the autonIdentifier variable to the
         * selected autonomous program.
         * 
         * @param btnm the pressed button's button matrix
         * @param txt the pressed button's text
         * 
         * @return LV_RES_OK
         */
        static lv_res_t autonSelect(lv_obj_t * btnm, const char *txt);


        /**
         * @brief the constructor for the JesterOS class
         * 
         * The constructor should be called in initialize().
         */
        void init();

        /**
         * @brief updates the selected autonomous text
         * 
         * This function updates the selected autonomous text, which is
         * displayed to the driveteam.
         */
        static void updateAuton();
        
        /**
         * @brief Writes a formatted string to the debug terminal in JesterOS
         * NOTE: this function has been adapted from 5090Z's 2018-2019 GUI code.
         */
        void write(int line, const char *format, ...);
        
        /**
         *  NOTE: these two functions are WIP, will let us monitor a specific telemetry of all motors at once,
         *        ie can see all motor temperature data at once
         */
        void motorMonitor(int line, const char *format, ...);
        void motorMonitorTemperature();
}