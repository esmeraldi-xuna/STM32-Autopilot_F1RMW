#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <mbed.h>
// #include "BufferedSerial.h"


enum __command 
{
    cmd_return,
    cmd_top,
    cmd_sys_info,
    cmd_thread_info,
    cmd_clear,
    cmd_help,
    cmd_invalid,
};

void cli();

//__command handleInput(BufferedSerial *serial);
__command get_command(char*);

void help(void);

int sysinfo(void);

int threadinfo(void);

// int top(BufferedSerial *serial);

#endif