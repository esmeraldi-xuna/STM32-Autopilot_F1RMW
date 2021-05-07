/*! \file global_vars.hpp 
    \brief Header gathering the global variables used in the Firmware

    Global variables are chosen upon uORB because yes.
*/

#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H


#include <mbed.h>
// #include "MPU9250.h"


/////////////////////////////////  synch obj   /////////////////////////////////////

extern Semaphore semDecode, semEncode, semUDPNav, semNavContr, semContrPWM;

extern Semaphore sem_nav_ctrl, sem_ctrl_PWM, sem_PWM_sens, sem_sens_EKF, sem_EKF_NAV_ctrl;


extern Mutex led_lock, print_lock, displayData_lock;


/////////////////////////////////  commander   /////////////////////////////////////

// commander for arming/disaming
#include "commander.hpp" 
extern Commander* main_commander;


/////////////////////////////////  controller   /////////////////////////////////////

#include "PI_contr.h"
/**
 * The I/O variables of the controller are used as extern since their name is the same if a Simulink project is used. 
 * This allows to keep the Firmware unchanged.
 */

extern ExtU_PI_contr_T PI_contr_U;     // External inputs
extern ExtY_PI_contr_T PI_contr_Y;     // External outputs


/////////////////////////////////  EKF   /////////////////////////////////////
#include "Kalman_filter_conv.h"

extern ExtU_Kalman_filter_conv_T Kalman_filter_conv_U;// External inputs
extern ExtY_Kalman_filter_conv_T Kalman_filter_conv_Y;// External outputs


/////////////////////////////////  controller   /////////////////////////////////////
#include "APF_conver.h"

extern ExtU_APF_conver_T APF_conver_U; // External inputs
extern ExtY_APF_conver_T APF_conver_Y; // External outputs


/////////////////////////////////  communication   /////////////////////////////////////

#if MAVLINK
#include "mavlink/common/mavlink.h"
//extern bool flagMavlink;

extern mavlink_attitude_t att;
extern mavlink_odometry_t odom;
extern mavlink_set_position_target_local_ned_t setpointsTrajectoryPlanner;
#endif


/////////////////////////////////  global variables to display   /////////////////////////////////////
#include "DisplayData.hpp"

extern DisplayData* global_data;

#endif

/////////////////////////////////  unused   /////////////////////////////////////

// struct __UDPbuff {
//   uint8_t in_data[MAVLINK_MAX_PACKET_LEN];
//   bool in_data_full;
//   uint8_t out_buf[MAVLINK_MAX_PACKET_LEN];
//   bool out_buf_full;
// };

// #ifndef SERVO_1
// #define SERVO_1
// #include "Servo.h"
// extern Servo servo1;

// #endif

// #ifndef QUEUE_IO IT DOESN'T WORK!!!
// #define QUEUE_IO  

// extern EventQueue queue;

// #endif


// #ifndef ETH_COMM
// #define ETH_COMM

// extern EthernetInterface eth;
// extern UDPSocket socket;

// #endif

// #ifndef ACCMAG_VALUES
// #define ACCMAG_VALUES

// extern Data accmagValues;

// #endif



// // Calibration flags are used to notify whether a sensor has undergone the process of calibration successfully
// #ifndef CALIBRATION_FLAGS
// #define CALIBRATION_FLAGS

// extern bool magCalibFlag;

// #endif

// // Calibration values are used by the sensors at start up to correct errors
// #ifndef CALIBRATION_VALUES
// #define CALIBRATION_VALUES

// extern float minMag[3], maxMag[3];

// #endif
