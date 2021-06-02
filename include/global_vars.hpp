/*! \file global_vars.hpp 
    \brief Header gathering the global variables used in the Firmware

    Global variables are chosen upon uORB because yes.
*/

#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <mbed.h>

/////////////////////////////////  synch obj   /////////////////////////////////////

extern Mutex led_lock, print_lock;

/////////////////////////////////  commander   /////////////////////////////////////

#include "commander.hpp" 
extern Commander* main_commander;

/////////////////////////////////  global variables to display   /////////////////////////////////////

#include "GlobalData.hpp"
extern GlobalData* global_data;

#endif
