#include <mbed.h>
#include "cli.hpp"
#include "cli_appereance.hpp"
#include "global_vars.hpp"
#include "sensors.hpp"

extern FSM_STATES active_state;
extern unsigned int new_state;

void cli()
{
    char cliBuffer[50];
    const char* prompt = "user@stm32 >> ";
    __command command;

    print_lock.lock();
    printf("Commnad line init, thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();

    // wait inizialization
    ThisThread::sleep_for(3s);

    /*
    print_lock.lock();
    printf("\033[2J\033[1;1H"); // clear screen
    printf("New Terminal\n");
    print_lock.unlock();
    */
    

    // console ready, start getting input
    while (1)
    {
        // printf("\nSTATE: %d", main_commander->get_main_FMS_state());
        if(main_commander->get_main_FMS_state() == sys_safe){
            ThisThread::sleep_for(100ms);
            print_lock.lock();
            printf("\n%s",prompt);
            print_lock.unlock();
            ThisThread::sleep_for(100ms);
            
            // get user input
            handle_input(cliBuffer);

            // debug printf
            /*
            print_lock.lock();
            printf("\nread: %s\n", cliBuffer);
            print_lock.unlock();
            */

            // get command
            command = string_to_command(cliBuffer);

            print_lock.lock();
            switch (command)
            {
            case cmd_sys_info:
                sysinfo();
                break;

            case cmd_thread_info:
                threadinfo();
                break;

            case cmd_return:
                break;

            case cmd_clear:
                printf("\033[2J\033[1;1H");
                break;

            case cmd_help:
                help();
                break;

            case cmd_top:
                top();
                break;

            case cmd_display_once:
                display_once();
                break;

            case cmd_display_repeat:
                display_repeat();
                break;

            case cmd_mag_calib:
                start_magnetometer_calibration();
                break;

            case cmd_arm_req:
                arm_request();
                break;

            case cmd_reset:
                reset();
                break;

            case cmd_run_man:
                active_state = sys_run_manual;
                new_state = 1;
                break;

            case cmd_run_auto:
                active_state = sys_run_auto;
                new_state = 1;
                break;

            case cmd_invalid:
            default:
                printf(RED("Type a valid command\n"));
                break;
            }
            print_lock.unlock();
        }
        else{
            ThisThread::sleep_for(100ms);
        }
    }
}

void handle_input(char* output){
    
    char tmp; 
    int i=0;

    // get one char at time, put it on console
    do{
        tmp=getchar();
        output[i] = tmp;
        i++;
        putchar(tmp);
    }while(tmp != '\n'); // end when return pressed

    // change \n with \0 (string terminator)
    output[i-1]='\0';

    return;
}

__command string_to_command(char* input){

    if (strcmp(input, "info")== 0){
        return cmd_sys_info;
    }
    if (strcmp(input, "thread")== 0){
        return cmd_thread_info;
    }
    if (strcmp(input, "clear")== 0){
        return cmd_clear;
    }
    if (strcmp(input, "help")== 0){
        return cmd_help;
    }
    if (strcmp(input, "return")== 0 || strcmp(input, "\0") == 0){
        return cmd_return;
    }
    if (strcmp(input, "top")== 0){
        return cmd_top;
    }
    if (strcmp(input, "reset")== 0){
        return cmd_reset;
    }
    if (strcmp(input, "display")== 0){
        return cmd_display_once;
    }
    if (strcmp(input, "display_r")== 0){
        return cmd_display_repeat;
    }
    if (strcmp(input, "arm") == 0){
        return cmd_arm_req;
    }
    if (strcmp(input, "calibration") == 0){
        return cmd_mag_calib;
    }
    if (strcmp(input, "auto") == 0){
        return cmd_run_auto;
    }
    if (strcmp(input, "manual") == 0){
        return cmd_run_man;
    }
    return cmd_invalid;
}

void start_magnetometer_calibration(void){
    printf("Calibration task\n");
    mag_calibration();
}

void arm_request(void){
    printf("Arming task\n");
    if (main_commander->arm()){
        printf("Arm ok!!!!");
    }
}