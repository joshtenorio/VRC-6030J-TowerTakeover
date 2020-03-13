/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2018, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_


#include "api.h"

/**
 * You should add more #includes here
 */
#include "okapi/api.hpp"
#include "settings.hpp"
#include "library.hpp"
#include "scripts.hpp"
#include "robot/chassis.hpp"
#include "robot/odometry.hpp"
#include "robot/purepursuit.hpp"
#include "robot/copycat.hpp"
#include "robot/rollers.hpp"
#include "robot/tilter.hpp"
#include "robot/lift.hpp"
#include "robot/Pid.hpp"
//#include "pros/api_legacy.h"

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
