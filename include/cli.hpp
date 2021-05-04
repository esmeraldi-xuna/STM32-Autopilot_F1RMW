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
    cmd_display,
    cmd_reset,
    cmd_invalid,
};

void cli(void);

void handle_input(char*);

__command string_to_command(char*);

///////////////// handlers for command /////////////////

void help(void);

int sysinfo(void);

int threadinfo(void);

int top(void);

void display(void);

void reset(void);

#endif