/*! @file main.cpp 

    @brief Entry point for mbed OS.

    This script creates and spawns threads and declare global variables defined
    in the header global_vars.hpp.
*/


// not edited
#include <mbed.h>
#include "Thread.h"


// edited
#include "cli.hpp"
#include "cntrInit.hpp"
#include "commander.hpp"
#include "DisplayData.hpp"
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
sem_nav_PI       // NAV done -> start PI
sem_PI_PWM       // PI done -> activate motor
sem_PWM_sens     // motor activated -> start sensors. Starts with 1 to allow first sensors reading 
sem_sens_nav     // sensors data acquired -> send to NAV
sem_sens_PI      // sensors data acquired -> send to PI
*/
Semaphore sem_nav_PI(0), sem_PI_PWM(0), sem_PWM_sens(1), sem_sens_nav(0), sem_sens_PI(0);
          
Mutex led_lock, print_lock, displayData_lock;


// Defining global inputs/outputs of the controller and his thread 
ExtU_PI_contr_T PI_contr_U;     // External inputs
ExtY_PI_contr_T PI_contr_Y;     // External outputs

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
  const char* navi_thread_name = "Navigator";
  //const char* prognostic_thread_name = "Prognostic";
  const char* cli_thread_name = "cli";
  const char* cntrInit_thread_name = "cntrInit";

  Thread SensorInit(osPriorityNormal,4096,nullptr,sensInit_thread_name);
  Thread OutputPortInit(osPriorityNormal,4096,nullptr,outportInit_thread_name);
  Thread Navigator(osPriorityNormal,4096,nullptr,navi_thread_name);
  // Thread Prognostic(osPriorityNormal,8092,nullptr,prognostic_thread_name);
  Thread CommandLineInterface(osPriorityNormal,4096,nullptr,cli_thread_name);
  Thread ControllerInit(osPriorityHigh,4096,nullptr,cntrInit_thread_name);


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

  SensorInit.start(sensInit);
  print_lock.lock();
  printf("%s thread started\n", sensInit_thread_name);
  print_lock.unlock();

  ThisThread::sleep_for(2s);

  OutputPortInit.start(outportInit);
  print_lock.lock();
  printf("%s thread started\n", outportInit_thread_name);
  print_lock.unlock();

  ThisThread::sleep_for(2s);
  
  /*
  Prognostic.start(prognostic); // unused without battery
  printf("%s thread started\n", prognostic_thread_name);
  */

 ThisThread::sleep_for(2s);

  Navigator.start(navigator); // do nothing
  print_lock.lock();
  printf("%s thread started\n", navi_thread_name);
  print_lock.unlock();

  ThisThread::sleep_for(2s);

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

  #if CLI_ACTIVE
  CommandLineInterface.start(cli);
  print_lock.lock();
  printf("Command line available\n");
  print_lock.unlock();
  #endif

  while(1) {
    ThisThread::sleep_for(5s);
  }  
}