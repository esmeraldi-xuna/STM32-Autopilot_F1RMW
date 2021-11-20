/*! @file main.cpp 

    @brief Entry point for mbed OS.

    This script creates and spawns threads and declare global variables defined
    in the header global_vars.hpp.
*/


// not edited
#include <mbed.h>
#include "BufferedSerial.h"
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
#include "SBUS.h"
#include "SD_log.hpp"
#include "sensors.hpp"
#include "mavlink_serial.hpp"

using namespace events;
using namespace rtos;
using namespace ThisThread;
using namespace mbed;

#if OVERRIDE_CONSOLE
FileHandle *mbed::mbed_override_console(int) {
    PinName pin_for_TX = D1; 
    PinName pin_for_RX = D0; 
    int baud_rate = 9600;

    static BufferedSerial my_serial(pin_for_TX, pin_for_RX, baud_rate);
    return &my_serial;
}
#endif

#if PIL_MODE
    #include "UDPPIL.hpp"
    /** This spawns the thread responsible for receving/sending simulation data from/to an external PC. It is enables only if 
     * the build flag PIL_MODE is set to 1. The thread function is in UDPComm.cpp
     */

    const char* UDP_PIL_thread_name = "UDPPIL";
    Thread UDPIO_PIL(osPriorityHigh,4096,nullptr,UDP_PIL_thread_name);
#endif

// commander for arming/disaming
Commander* main_commander = new Commander();

// struct for storing all data to be displayed
GlobalData* global_data = new GlobalData();

//  syncro objs
Mutex led_lock, print_lock;

// final state machine active state
FSM_STATES active_state = SYS_INIT;

// boolean used by CLI to change state when in safe state
unsigned int new_state = 0;

int main(){

    // program starts here
    
    // allow the commander to access the acite state variable
    main_commander->set_p_to_FSM_state(&active_state);


    // variables declaration

    // mavlink communication
    PinName pin_tx =  PA_9 /* = D1*/, pin_rx = PA_10 /* = D0*/;
    
    #if OVERRIDE_CONSOLE // use mavlink over usb_serial
      pin_tx = USBTX;
      pin_rx = USBRX;
    #endif
    // open serial channel for mavlink
    static BufferedSerial mavlink_serial_ch(pin_tx, pin_rx, 9600);

    // SBUS (joystick)
    PinName sbus_tx = D6, sbus_rx = A5; // TODO: find correct PINs
    
    // THREADS
    Thread Sensor               (osPriorityNormal, 4096, nullptr, "sens");
    Thread PWMPort              (osPriorityNormal, 2048, nullptr, "PWM");
    Thread CommandLineInterface (osPriorityNormal, 2048, nullptr, "CLI");
    Thread Controller           (osPriorityNormal, 2048, nullptr, "control");
    Thread APF                  (osPriorityNormal, 4096, nullptr, "APF");
    Thread EKF                  (osPriorityNormal, 4096, nullptr, "EKF");
    Thread mavlink_RX           (osPriorityNormal, 4096, nullptr, "MAV_RX");
    Thread mavlink_TX           (osPriorityNormal, 4096, nullptr, "MAV_TX");

    // Thread Navigator            (osPriorityNormal, 4096, nullptr, "NAV");
    // Thread Prognostic           (osPriorityNormal, 2048, nullptr, "progn");
    
    #if SD_MOUNTED
    Thread SD_log               (osPriorityNormal, 2048, nullptr, "SD-log");
    #endif

    // START
    print_lock.lock();
    printf("\033[2J\033[1;1H"); // clear screen
    printf("\n ====== Firmware is starting... ====== \n");
    printf("System INIT...\n");
    print_lock.unlock();

    // filesytem 
    #if SD_MOUNTED
    file_sys_init();
    #endif

    // main loop for FSM
    while(1)
    {
        switch (active_state){
            case SYS_INIT:{
                
                // force PWM disabled untill RUN states
                main_commander->force_PWM_disable();

                // start CLI thread but show it only in safe state
                CommandLineInterface.start(cli);

                // start sensors
                Sensor.start(sensors);

                // start comm with joystick (TODO: define pin)
                /*
                if(sbus_init(sbus_tx, sbus_rx) != -1){
                    print_lock.lock();
                    printf("SBUS ok\n");
                    print_lock.unlock();
                    main_commander->set_flag_comm_joystick(true);
                }else{
                    main_commander->set_flag_comm_joystick(false);
                }
                */
                // Debug
                main_commander->set_flag_comm_joystick(true);
                ////////

                // start log
                #if SD_MOUNTED
                SD_log.start(SD_log_loop);
                #endif

                // wait untill all ok
                while(!main_commander->check_init()){
                    ThisThread::sleep_for(10ms);
                }

                
                // pass to next step
                print_lock.lock();
                printf("INIT OK\n");
                print_lock.unlock();
                active_state = SYS_STARTUP;

                break;
            }

            case SYS_STARTUP:{
                
                print_lock.lock();
                printf("System STARTUP...\n");
                print_lock.unlock();

                // start mavlink
                mavlink_RX.start(callback(mavlink_serial_RX, &mavlink_serial_ch));
                ThisThread::sleep_for(10ms);
                mavlink_TX.start(callback(mavlink_serial_TX, &mavlink_serial_ch));
                ThisThread::sleep_for(10ms);

                // start controller
                Controller.start(PI_controller);
                ThisThread::sleep_for(10ms);
                EKF.start(ekf);
                ThisThread::sleep_for(10ms);
                APF.start(apf);
                ThisThread::sleep_for(10ms);

                // start PWM
                PWMPort.start(PWMport);
                ThisThread::sleep_for(10ms);

                // wait untill all ok
                while(!main_commander->check_startup()){
                    ThisThread::sleep_for(10ms);
                }

                // pass to next step
                print_lock.lock();
                printf("STARTUP OK\n");
                print_lock.unlock();
                active_state = SYS_SAFE;
                break;
            }

            case SYS_SAFE:{

                print_lock.lock();
                printf("SAFE MODE\n");
                print_lock.unlock();

                // safe state: PWM disabled, CLI active only here
                main_commander->force_PWM_disable();

                // use CLI or joystick to enter in a run mode
                unsigned int raw_sbus[25], sbus_channels_data[16];

                while (new_state == 0){
                    if(sbus_get_data(raw_sbus)){
                        sbus_fill_channels(raw_sbus, sbus_channels_data);
                        sbus_use_channels_data(sbus_channels_data);

                        // how to get command to switch state? (TODO define which command)
                        if(sbus_channels_data[0] == 1000){
                            new_state = 1;
                            active_state = SYS_RUN_MANUAL;
                        }
                        if(sbus_channels_data[0] == 2000){
                            new_state = 1;
                            active_state = SYS_RUN_AUTO;
                        }
                    } 
                }

                break;
            }

            case SYS_RUN_AUTO:{

                print_lock.lock();
                printf("RUN AUTO MODE\n");
                print_lock.unlock();

                // auto mode: flight controlled by software, (use some joystick command to bypass?)
                main_commander->force_PWM_enable();
                
                if(main_commander->is_armed())
                {
                    // check status: main_commander->check_run_auto(); else -> active_state = sys_fail
                    if(main_commander->check_run_auto()){
                        // all ok, drone armed and active

                        // get sbus data to detect user command to stop the drone
                        unsigned int raw_sbus[25], sbus_channels_data[16];

                        if(sbus_get_data(raw_sbus)){
                            sbus_fill_channels(raw_sbus, sbus_channels_data);
                            sbus_use_channels_data(sbus_channels_data);

                            // how to get command to switch state? (TODO define which command)
                            if(sbus_channels_data[0] == 1000){
                                active_state = SYS_RUN_MANUAL;
                            }
                            if(sbus_channels_data[0] == 2000){
                                active_state = SYS_SAFE;
                            }
                            
                            // use controller output as PWM commands
                            // write pwm_value in controller thread
                        }
                    }else{
                        // some problem occurred, go to fail state
                        // differences between major and minor to be defined
                        if(1){
                            active_state = SYS_FAIL_MAJOR;
                        }else{
                            active_state = SYS_FAIL_MINOR;
                        }
                    }
                }
                else{
                    // not armed, try arming
                    if(!main_commander->arm()){
                        // some problem occurred, go to safe state
                        active_state = SYS_SAFE;
                    }
                }

                // DEBUG
                active_state = SYS_FAIL_MAJOR;
                ///////////////////

                break;
            }

            case SYS_RUN_MANUAL:{

                print_lock.lock();
                printf("RUN MANUAL MODE\n");
                print_lock.unlock();

                // manual mode: flight controlled by user (joystick)
                main_commander->force_PWM_enable();
                
                if(main_commander->is_armed())
                {
                    // check status: main_commander->check_run_manual(); else -> active_state = sys_fail
                    if(main_commander->check_run_manual()){
                        // all ok, drone armed and active

                        // get sbus data to detect user command to stop the drone
                        unsigned int raw_sbus[25], sbus_channels_data[16];

                        if(sbus_get_data(raw_sbus)){
                            struct_pwm_data pwm_out;
                            sbus_fill_channels(raw_sbus, sbus_channels_data);
                            sbus_use_channels_data(sbus_channels_data);

                            // how to get command to switch state? (TODO define which command)
                            if(sbus_channels_data[0] == 1000){
                                active_state = SYS_SAFE;
                            }
                            // example
                            pwm_out.motor1 = sbus_channels_data[1];
                            
                            // use SBUS messages as PWM commands ??????
                            // write pwm_value here
                            global_data->write_pwm(pwm_out);
                        }
                    }else{
                        // some problem occurred, go to fail state
                        // differences between major and minor to be defined
                        if(1){
                            active_state = SYS_FAIL_MAJOR;
                        }else{
                            active_state = SYS_FAIL_MINOR;
                        }
                    }
                }
                else{
                    // not armed, try arming
                    if(!main_commander->arm()){
                        // some problem occurred, go to safe state
                        active_state = SYS_SAFE;
                    }
                }

                // DEBUG
                active_state = SYS_FAIL_MAJOR;
                ///////////////////

                break;
            }

            case SYS_FAIL_MAJOR:{
                // failure major mode
                print_lock.lock();
                printf("FATAL ERROR OCCURRED\n");
                // main_commander->show_all_flags();
                print_lock.unlock();

                // procedure to recover from fail ??

                active_state = SYS_SAFE;
                break;
            }

            case SYS_FAIL_MINOR:{
                // failure minor mode
                print_lock.lock();
                printf("MINOR ERROR OCCURRED\n");
                // main_commander->show_all_flags();
                print_lock.unlock();

                // procedure to recover from fail ??

                active_state = SYS_SAFE;
                break;
            }

            default:{
                print_lock.lock();
                printf("FATAL ERROR OCCURRED\n");
                print_lock.unlock();
                active_state = SYS_SAFE;
                break;
            }
        }
        new_state = 0;
    }
    // never reach this point 
}

