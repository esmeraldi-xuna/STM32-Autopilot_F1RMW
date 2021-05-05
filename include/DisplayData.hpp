#ifndef DISPLAY_DATA_H
#define DISPLAY_DATA_H

#include "mbed.h"
#include "PI_contr.h"


typedef struct data_type{
    struct z{
        int altitude=0;
        int accel=0;
    }sensors;

    struct y{
        ExtU_PI_contr_T PI_contr_U; 
        ExtY_PI_contr_T PI_contr_Y; 
    }controller;

    struct x{
        int motor1=0;
        int motor2=0;
    }pwm;
} data_t;


class DisplayData
{
    public:
    DisplayData(){};

    void display();
    
    data_t data;
};

#endif