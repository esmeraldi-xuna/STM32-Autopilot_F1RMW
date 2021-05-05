#include "mbed.h"
#include "DisplayData.hpp"
#include "cli_appereance.hpp"

void DisplayData::display(){

    printf(GREEN("\n ----------- global data -----------------\n\n"));

    printf(RED("  SENSORS\n"));
    printf("Altitude: %d\tAcceleration: %d\n\n", this->data.sensors.altitude, this->data.sensors.accel);

    printf(RED("  PI CONTROLLER\n"));
    printf(" PI_contr_U\tPI_contr_Y\n\n");

    printf(RED("  PWM:\n"));
    printf("Motor 1: %d\tMotor 2: %d\n\n", this->data.pwm.motor1, this->data.pwm.motor2);

    printf(GREEN("\n ---------------- end --------------------\n"));

    return;
}
