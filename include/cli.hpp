#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <mbed.h>

enum __command 
{
    cmd_return,
    cmd_top,
    cmd_sys_info,
    cmd_thread_info,
    cmd_clear,
    cmd_help,
    cmd_display_once,
    cmd_display_repeat,
    cmd_mag_calib,
    cmd_arm_req,
    cmd_reset,
    cmd_invalid,

    cmd_run_auto,
    cmd_run_man,
};

void cli(void);

void handle_input(char*);

__command string_to_command(char*);

///////////////// handlers for command /////////////////

void help(void);

int sysinfo(void);

int threadinfo(void);

int top(void);

void display_once(void);

void display_repeat(void);

void display_loop(void);

void start_magnetometer_calibration(void);

void arm_request(void);

void reset(void);

#endif