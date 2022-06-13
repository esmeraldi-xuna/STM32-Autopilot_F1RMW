#include <mbed.h>
#include "cli.hpp"
#include "cli_appereance.hpp"
#include "global_vars.hpp"
#include "sensors.hpp"

// used for testing FSM
extern FSM_STATES active_state;
extern unsigned int new_state;
////////////////////////////////////

void cli()
{   
    char cliBuffer[50];
    const char *prompt = "user@k64f >> ";
    __command command;

    /*
    print_lock.lock();
    printf("Commnad line init, thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();

    print_lock.lock();
    printf("\033[2J\033[1;1H"); // clear screen
    printf("New Terminal\n");
    print_lock.unlock();
    */

    // wait inizialization
    ThisThread::sleep_for(3s);

    // console ready, start getting input
    while (1)
    {
        // printf("\nSTATE: %d", main_commander->get_main_FMS_state());
        if (main_commander->get_main_FMS_state() == SYS_SAFE)
        {
            ThisThread::sleep_for(100ms);
            print_lock.lock();
            printf("\n%s", prompt);
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

            // switch command functions
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
                mag_calibration();
                break;

            case cmd_arm_req:
                arm_request();
                break;

            case cmd_reset:
                reset();
                break;
            case cmd_ls:
                cli_ls();
                break;
            case cmd_cat:
                cli_cat(cliBuffer);
                break;

            case cmd_run_man:
                
                active_state = SYS_RUN_MANUAL;
                new_state = 1;
                break;

            case cmd_run_auto:
                active_state = SYS_RUN_AUTO;
                new_state = 1;
                break;

            case cmd_invalid:
            default:
                printf(RED("Type a valid command\n"));
                break;
            }
            print_lock.unlock();
        }
        else
        {
            ThisThread::sleep_for(100ms);
        }
    }
}

void handle_input(char *output)
{

    char tmp;
    int i = 0;
    int junk = 0x7f;
    // get one char at time, put it on console
    do
    {
        tmp = getchar();
        if (tmp != junk)
        { // non ho schiacciato backspace -> salvo e stamp
            output[i] = tmp;
            i++;
            putchar(tmp);
        }
        else if (i > 0)
        { // ho già messo un carattere
            i--;
            putchar(tmp); // lo rimuovo dalla console
        }
        else
        { // non ho messo caratteri -> non devo aggiustare nulla sulla console
        }
    } while (tmp != '\n'); // end when return pressed

    // change \n with \0 (string terminator)
    output[i - 1] = '\0';

    return;
}

__command string_to_command(char *input)
{

    if (strcmp(input, "info") == 0)
    {
        return cmd_sys_info;
    }
    if (strcmp(input, "thread") == 0)
    {
        return cmd_thread_info;
    }
    if (strcmp(input, "clear") == 0)
    {
        return cmd_clear;
    }
    if (strcmp(input, "help") == 0)
    {
        return cmd_help;
    }
    if (strcmp(input, "return") == 0 || strcmp(input, "\0") == 0)
    {
        return cmd_return;
    }
    if (strcmp(input, "top") == 0)
    {
        return cmd_top;
    }
    if (strcmp(input, "reset") == 0)
    {
        return cmd_reset;
    }
    if (strcmp(input, "display") == 0)
    {
        return cmd_display_once;
    }
    if (strcmp(input, "display_r") == 0)
    {
        return cmd_display_repeat;
    }
    if (strcmp(input, "arm") == 0)
    {
        return cmd_arm_req;
    }
    if (strcmp(input, "calibration") == 0)
    {
        return cmd_mag_calib;
    }
    if (strcmp(input, "auto") == 0)
    {
        return cmd_run_auto;
    }
    if (strcmp(input, "manual") == 0)
    {
        return cmd_run_man;
    }
    if (strcmp(input, "ls") == 0)
    {
        return cmd_ls;
    }
    if (strncmp(input, "cat ", 4) == 0 && strlen(input) > 4)
        return cmd_cat;
    return cmd_invalid;
}

/* void start_magnetometer_calibration(void){
    printf("Calibration task\n");

}
 */
void arm_request(void)
{
    printf("Arming task\n");
    if (main_commander->arm())
    {
        printf("Arm ok!!!!");
    }
    else
    {
        printf("NOT ARMED!!!!");
    }
}