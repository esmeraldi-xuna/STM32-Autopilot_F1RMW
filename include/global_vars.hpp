/*! \file global_vars.hpp 
    \brief Header gathering the global variables used in the Firmware

    Global variables are chosen upon uORB because yes.
*/

#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <mbed.h>

typedef enum {
    sys_init = 0,
    sys_startup,
    sys_fail,
    sys_safe,
    sys_run_auto,
    sys_run_manual
}FSM_STATES;

#define OVERRIDE_CONSOLE 0 /* if 0: console on usb, mavlink on D1, D0; if 1: console on D1, D0, mavlink on usb*/
#define CLI_ACTIVE 1
#define SD_MOUNTED 1

/////////////////////////////////  synch obj   /////////////////////////////////////

extern Mutex led_lock, print_lock;

/////////////////////////////////  commander   /////////////////////////////////////

#include "commander.hpp" 
extern Commander* main_commander;

/////////////////////////////////  global variables to display   /////////////////////////////////////

#include "GlobalData.hpp"
extern GlobalData* global_data;

#endif
