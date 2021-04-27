/*! @file main.cpp 

    @brief Entry point for mbed OS.

    This script creates and spawns threads and declare global variables defined
    in the header global_vars.hpp.
*/


// not edited
#include <mbed.h>
// #include <BufferedSerial.h>
#include "Thread.h"
#include "mavlink/common/mavlink.h"

// edited
#include "cli.hpp"
#include "cntrInit.hpp"
#include "commander.hpp"
#include "global_vars.hpp"
#include "navigator.hpp"
#include "outportInit.hpp"
#include "prognostic.hpp"
#include "sensInit.hpp"

using namespace events;
using namespace rtos;
using namespace ThisThread;
using namespace mbed;


// defining threads
const char* sensInit_thread_name = "sensInit";
const char* outportInit_thread_name = "outportInit";
const char* navi_thread_name = "Navigator";
const char* prognostic_thread_name = "Prognostic";

Thread SensorInit(osPriorityNormal,8092,nullptr,sensInit_thread_name);
Thread OutputPortInit(osPriorityNormal,16184,nullptr,outportInit_thread_name);
Thread Navigator(osPriorityNormal,16184,nullptr,navi_thread_name);
Thread Prognostic(osPriorityNormal,8092,nullptr,prognostic_thread_name);


// commander for arming/disaming
Commander* main_commander = new Commander();


/* serial channel for command line */
// BufferedSerial* serial_channel = new BufferedSerial(USBTX,USBRX,115200);

const char* cli_thread_name = "cli";
Thread CommandLineInterface(osPriorityNormal,8092,nullptr,cli_thread_name);


// init syncro objs
Semaphore semDecode(0), semEncode(0), semUDPNav(0), semNavContr(0), semContrPWM(0);

Mutex led_lock, print_lock;


// Defining global inputs/outputs of the controller and his thread 
ExtU_PI_contr_T PI_contr_U;     // External inputs
ExtY_PI_contr_T PI_contr_Y;     // External outputs

const char* cntrInit_thread_name = "cntrInit";
Thread ControllerInit(osPriorityHigh,8092,nullptr,cntrInit_thread_name);


// init communication
// bool flagMavlink = false;
mavlink_attitude_t att;
mavlink_odometry_t odom;
mavlink_set_position_target_local_ned_t setpointsTrajectoryPlanner;

#if PIL_MODE
  #include "UDPPIL.hpp"
  /** This spawns the thread responsible for receving/sending simulation data from/to an external PC. It is enables only if 
   * the build flag PIL_MODE is set to 1. The thread function is in UDPComm.cpp
   */

  const char* UDP_PIL_thread_name = "UDPPIL";
  Thread UDPIO_PIL(osPriorityHigh,8092,nullptr,UDP_PIL_thread_name);
#endif

int main() 
{
  //printf("\033[2J");
  set_time(0);

  #if PIL_MODE // Start UDP communtication only if in PIL mode!
    UDPIO_PIL.start(UDPPIL);
  #endif

  print_lock.lock();
  printf("\n\n\n ====== Firmware is starting... ====== \n");

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

  SensorInit.start(sensInit);
  print_lock.lock();
  printf("%s thread started\n", sensInit_thread_name);
  print_lock.unlock();

  OutputPortInit.start(outportInit);
  print_lock.lock();
  printf("%s thread started\n", outportInit_thread_name);
  print_lock.unlock();
  
  /*
  Prognostic.start(prognostic); // unused without battery
  printf("%s thread started\n", prognostic_thread_name);

  Navigator.start(navigator); // do nothing
  printf("%s thread started\n", navi_thread_name);
  */

  ControllerInit.start(cntrInit);
  print_lock.lock();
  printf("%s thread started\n", cntrInit_thread_name);
  print_lock.unlock();

  ThisThread::sleep_for(5s);
  
  /* not added
  #if EKF_TASK
  EKFInit.start(ekfInit);
  printf("%s thread started\n", ekfInit_thread_name);
  #endif

  #if APF_TASK
  APFInit.start(apfInit);
  printf("%s thread started\n", apfInit_thread_name);
  #endif
  */

  CommandLineInterface.start(cli);
  print_lock.lock();
  printf("Command line available\n");
  print_lock.unlock();

  while(1) {
    ThisThread::sleep_for(5s);
    //printf("Hello\n");
  }  
}