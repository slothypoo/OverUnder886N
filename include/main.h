/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES

#define PROS_USE_LITERALS

#include "api.h"
#include "chassi.hpp"
#include "baseFunctions.hpp"

using namespace pros;
using namespace std;


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
 * You can add C++-only headers hereƒß
 */
//#include <iostream>
#endif

extern pros::Motor leftFront;
extern pros::Motor rightFront;
extern pros::Motor leftBack;
extern pros::Motor rightBack;
extern pros::Motor leftMiddle;
extern pros::Motor rightMiddle;
extern pros::Motor flywheelL;
extern pros::Motor flywheelR;

extern pros::Controller master;
extern pros::ADIDigitalOut wings;
extern pros::ADIDigitalOut blocker;
extern pros::ADIDigitalOut hang;


extern Imu imu;


#endif  // _PROS_MAIN_H_