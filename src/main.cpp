/*! @file main.cpp 

    @brief Entry point for mbed OS.

    This script creates and spawns threads and declare global variables defined
    in the header global_vars.hpp.
*/


// not edited
#include <mbed.h>
#include "Thread.h"


// edited
#include "apf.hpp"
#include "cli.hpp"
#include "commander.hpp"
#include "ekf.hpp"
#include "GlobalData.hpp"
#include "global_vars.hpp"
#include "navigator.hpp"
#include "PI_controller.hpp"
#include "prognostic.hpp"
#include "PWM_port.hpp"
#include "read_write_lock.hpp"
#include "SD_log.hpp"
#include "sensors.hpp"

using namespace events;
using namespace rtos;
using namespace ThisThread;
using namespace mbed;

#define OVERRIDE_CONSOLE 0
#define CLI_ACTIVE 1
#define MAVLINK 0
#define SD_MOUNTED 0

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
GlobalData* global_data = new GlobalData();

//  syncro objs
          
Mutex led_lock, print_lock;

//  communication
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
  const char* sens_thread_name = "sens";
  const char* PWMport_thread_name = "PWM";
  // const char* navi_thread_name = "Navigator";
  // const char* prognostic_thread_name = "Prognostic";
  const char* cli_thread_name = "cli";
  const char* cntr_thread_name = "cntr";
  const char* ekf_thread_name = "ekf";
  const char* apf_thread_name = "apf";

  Thread Sensor(osPriorityNormal,4096,nullptr,sens_thread_name);
  Thread PWMPort(osPriorityNormal,4096,nullptr,PWMport_thread_name);
  // Thread Navigator(osPriorityNormal,4096,nullptr,navi_thread_name);
  // Thread Prognostic(osPriorityNormal,8092,nullptr,prognostic_thread_name);
  Thread CommandLineInterface(osPriorityNormal,4096,nullptr,cli_thread_name);
  Thread Controller(osPriorityNormal,4096,nullptr,cntr_thread_name);
  Thread APF(osPriorityNormal,4096,nullptr,apf_thread_name);
  Thread EKF(osPriorityNormal,4096,nullptr,ekf_thread_name);
  
  #if SD_MOUNTED
  Thread SD_log(osPriorityNormal,4096,nullptr,"SD-log");
  #endif

  printf("\033[2J\033[1;1H"); // clear screen
  set_time(0);

  #if PIL_MODE // Start UDP communtication only if in PIL mode!
    UDPIO_PIL.start(UDPPIL);
  #endif

  print_lock.lock();
  printf("\n ====== Firmware is starting... ====== \n");

  #if SD_MOUNTED
  file_sys_();
  #endif

  printf("Spawning threads...\n");
  print_lock.unlock();

  /*
  Prognostic.start(prognostic); // unused without battery
  */

  /*
  Navigator.start(navigator); // not used with APF
  */

  Sensor.start(sensors); // loop

  ThisThread::sleep_for(500ms);

  PWMPort.start(PWMport); // loop

  ThisThread::sleep_for(500ms);

  Controller.start(PI_controller); // loop

  ThisThread::sleep_for(500ms);
  
  EKF.start(ekf); // loop

  ThisThread::sleep_for(500ms);

  APF.start(apf); // loop

  ThisThread::sleep_for(1s);

  #if SD_MOUNTED
  SD_log.start(SD_log_loop);
  #endif

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