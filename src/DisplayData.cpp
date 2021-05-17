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


void DisplayData::write_on_SD(FILE* out_file){

    fprintf(out_file, "SENS: Alt: %d, Acc: %d; ", data.sensors.altitude, data.sensors.accel);

    fprintf(out_file, "CNTR: Ctrl_U, Ctrl_Y; ");

    fprintf(out_file, "TRJ_PL: APF_U, APF_Y; ");

    fprintf(out_file, "PWM: M1: %d, M2: %d.\n", data.pwm.motor1, data.pwm.motor2);

    return;
}
