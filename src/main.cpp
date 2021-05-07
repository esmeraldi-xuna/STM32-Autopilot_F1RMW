/*! @file main.cpp 

    @brief Entry point for mbed OS.

    This script creates and spawns threads and declare global variables defined
    in the header global_vars.hpp.
*/


// not edited
#include <mbed.h>
#include "Thread.h"


// edited
#include "apfInit.hpp"
#include "cli.hpp"
#include "cntrInit.hpp"
#include "commander.hpp"
#include "DisplayData.hpp"
#include "ekfInit.hpp"
#include "global_vars.hpp"
#include "navigator.hpp"
#include "outportInit.hpp"
#include "prognostic.hpp"
#include "sensInit.hpp"

using namespace events;
using namespace rtos;
using namespace ThisThread;
using namespace mbed;

#define OVERRIDE_CONSOLE 0
#define CLI_ACTIVE 1
#define MAVLINK 0

#if OVERRIDE_CONSOLE
FileHandle *mbed::mbed_override_console(int) {
  PinName pin_for_TX = //A0; to be defined
  PinName pin_for_RX = //A1; to be defined
  int baud_rate = 115200;

  static BufferedSerial my_serial(pin_for_TX, pin_for_RX, baud_rate);
  return &my_serial;
}
#endif

// commander for arming/disaming
Commander* main_commander = new Commander();

// struct for storing all data to be displayed
DisplayData* global_data = new DisplayData();

// init syncro objs
Semaphore semDecode(0), semEncode(0), semUDPNav(0), semNavContr(0), semContrPWM(0);

/*
sem_nav_ctrl        // NAV done -> start controller
sem_ctrl_PWM        // controller done -> activate motor
sem_PWM_sens        // motor activated -> start sensors. Starts with 1 to allow first sensors reading 
sem_sens_EKF        // sensors data acquired -> send to EKF
sem_EKF_NAV_ctrl    // EKF data available -> send to NAV and controller
*/
Semaphore sem_nav_ctrl(0), sem_ctrl_PWM(0), sem_PWM_sens(1), sem_sens_EKF(0), sem_EKF_NAV_ctrl(0);
          
Mutex led_lock, print_lock, displayData_lock;


// Defining global inputs/outputs of the controller and his thread 
ExtU_PI_contr_T PI_contr_U;     // External inputs
ExtY_PI_contr_T PI_contr_Y;     // External outputs

// EKF
ExtU_Kalman_filter_conv_T Kalman_filter_conv_U;// External inputs
ExtY_Kalman_filter_conv_T Kalman_filter_conv_Y;// External outputs


// traj_planner (APF)
ExtU_APF_conver_T APF_conver_U; // External inputs
ExtY_APF_conver_T APF_conver_Y; // External outputs


// init communication
// bool flagMavlink = false;
#if MAVLINK 
#include "mavlink/common/mavlink.h"
mavlink_attitude_t att;
mavlink_odometry_t odom;
mavlink_set_position_target_local_ned_t setpointsTrajectoryPlanner;
#endif

#if PIL_MODE
  #include "UDPPIL.hpp"
  /** This spawns the thread responsible for receving/sending simulation data from/to an external PC. It is enables only if 
   * the build flag PIL_MODE is set to 1. The thread function is in UDPComm.cpp
   */

  const char* UDP_PIL_thread_name = "UDPPIL";
  Thread UDPIO_PIL(osPriorityHigh,4096,nullptr,UDP_PIL_thread_name);
#endif

int main() 
{
  // defining threads
  const char* sensInit_thread_name = "sensInit";
  const char* outportInit_thread_name = "outportInit";
  // const char* navi_thread_name = "Navigator";
  // const char* prognostic_thread_name = "Prognostic";
  const char* cli_thread_name = "cli";
  const char* cntrInit_thread_name = "cntrInit";
  const char* ekfInit_thread_name = "ekfInit";
  const char* apfInit_thread_name = "apfInit";

  Thread SensorInit(osPriorityNormal,4096,nullptr,sensInit_thread_name);
  Thread OutputPortInit(osPriorityNormal,4096,nullptr,outportInit_thread_name);
  // Thread Navigator(osPriorityNormal,4096,nullptr,navi_thread_name);
  // Thread Prognostic(osPriorityNormal,8092,nullptr,prognostic_thread_name);
  Thread CommandLineInterface(osPriorityNormal,4096,nullptr,cli_thread_name);
  Thread ControllerInit(osPriorityHigh,4096,nullptr,cntrInit_thread_name);
  Thread APFInit(osPriorityNormal,4096,nullptr,apfInit_thread_name);
  Thread EKFInit(osPriorityNormal,4096,nullptr,ekfInit_thread_name);


  printf("\033[2J\033[1;1H"); // clear screen
  set_time(0);

  #if PIL_MODE // Start UDP communtication only if in PIL mode!
    UDPIO_PIL.start(UDPPIL);
  #endif

  print_lock.lock();
  printf("\n ====== Firmware is starting... ====== \n");

  printf("Spawning threads...\n");
  print_lock.unlock();

  /* // not used
  SDStorage.start(massStorage);
  print_lock.lock();
  printf("%s thread started\n", sdcard_thread_name);
  SDStorage.join();
  printf("Mass storage initialized\n"); 
  print_lock.unlock();
  */

  /*
  Prognostic.start(prognostic); // unused without battery
  */

  /*
  Navigator.start(navigator); // not used with APF
  */

  SensorInit.start(sensInit); // loop

  ThisThread::sleep_for(1s);

  OutputPortInit.start(outportInit); // loop

  ThisThread::sleep_for(1s);

  ControllerInit.start(cntrInit); // start another thread
  ControllerInit.join();

  ThisThread::sleep_for(1s);
  
  EKFInit.start(ekfInit); // start another thread
  EKFInit.join();

  ThisThread::sleep_for(1s);

  APFInit.start(apfInit); // start another thread
  APFInit.join();

  ThisThread::sleep_for(2s);

  #if CLI_ACTIVE
  print_lock.lock();
  printf("Command line available\n");
  print_lock.unlock();
  CommandLineInterface.start(cli); // loop (start others thread in some functions)
  #endif

  while(1) {
    ThisThread::sleep_for(5s);
  }  
}