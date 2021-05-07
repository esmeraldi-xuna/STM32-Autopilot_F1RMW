#include "mbed.h"
#include "DisplayData.hpp"
#include "cli_appereance.hpp"

void DisplayData::display(){

    printf(GREEN("\n ----------- global data -----------------\n\n"));

    printf(RED("  SENSORS\n"));
    printf("Altitude: %d\tAcceleration: %d\n\n", data.sensors.altitude, data.sensors.accel);

    printf(RED("  PI CONTROLLER\n"));
    printf(" Ctrl_U\tCtrl_Y\n\n");

    printf(RED("  TRAJECTORY PLANNER\n"));
    printf(" APF_U\tAPF_Y\n\n");

    printf(RED("  PWM:\n"));
    printf("Motor 1: %d\tMotor 2: %d\n\n", data.pwm.motor1, data.pwm.motor2);

    printf(GREEN("\n ---------------- end --------------------\n"));

    return;
}
