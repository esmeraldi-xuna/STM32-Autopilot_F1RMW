/*! \file global_vars.hpp 
    \brief Header gathering the global variables used in the Firmware

    Global variables are chosen upon uORB because yes.
*/

#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <mbed.h>

#define OVERRIDE_CONSOLE 1 /* if 0: console on usb, mavlink on D1, D0; if 1: console on D1, D0, mavlink on usb*/
#define CLI_ACTIVE 1
#define SD_MOUNTED 1


#include "EthernetInterface.h"

extern EthernetInterface eth;
extern UDPSocket socket;
static const char *mbedIP = "192.168.1.10";     // IP
static const char *mbedMask = "255.255.255.0";  // Mask
static const char *mbedGateway = "192.168.1.1"; // Gateway

static const char *ltpndIP = "192.168.1.11"; //"169.254.85.139";
/////////////////////////////////  synch obj   /////////////////////////////////////

extern Mutex led_lock, print_lock;

/////////////////////////////////  commander   /////////////////////////////////////

#include "commander.hpp" 
extern Commander* main_commander;

/////////////////////////////////  global variables to display   /////////////////////////////////////

#include "GlobalData.hpp"
extern GlobalData* global_data;

#endif
