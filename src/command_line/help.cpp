#include <mbed.h>
#include "cli_appereance.hpp"
#include "cli.hpp"

// show commands for command line
void help(void)
{
    printf(GREEN("                                                     _____________\n"));
    printf(GREEN("____________________________________________________/             \\____________________________________________________\n"));
    printf(GREEN("##================================================== COMMAND  LIST ==================================================##\n"));
    printf(GREEN("##==================================================\\_____________/==================================================##\n"));

    printf(GREEN("|| "));
    printf("Available commands:");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("top");
    printf("\033[0m\033[15G");
    printf("Show CPU usage");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("info");
    printf("\033[0m\033[15G");
    printf("Show HW information");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("thread");
    printf("\033[0m\033[15G");
    printf("Show active threads");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("clear");
    printf("\033[0m\033[15G");
    printf("Clear the screen");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("help");
    printf("\033[0m\033[15G");
    printf("Show this help");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("display");
    printf("\033[0m\033[15G");
    printf("Display values only once");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("display_r");
    printf("\033[0m\033[15G");
    printf("Display values untill a button is pressed");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("arm");
    printf("\033[0m\033[15G");
    printf("Try arming");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("calibration");
    printf("\033[0m\033[15G");
    printf("Start magnetometer calibration");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("reset");
    printf("\033[0m\033[15G");
    printf("Perform software reset");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("auto");
    printf("\033[0m\033[15G");
    printf("Go to run_auto state");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("|| "));
    printf("\033[7m");
    printf("manual");
    printf("\033[0m\033[15G");
    printf("Go to run_manual state");
    printf("\033[118G");
    printf(GREEN("||\n"));

    printf(GREEN("##=================================================== ~~~~~~~~~~~ ===================================================##\n"));
}