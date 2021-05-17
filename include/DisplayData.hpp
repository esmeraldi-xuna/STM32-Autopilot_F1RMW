#ifndef DISPLAY_DATA_H
#define DISPLAY_DATA_H

#include "mbed.h"
#include "PI_contr.h"
#include "APF_conver.h"


typedef struct data_type{
    struct z{
        int altitude=0;
        int accel=0;
    }sensors;

    struct y{
        ExtU_PI_contr_T ctrl_U; 
        ExtY_PI_contr_T ctrl_Y; 
    }controller;

    struct r{
        ExtU_APF_conver_T APF_U;
        ExtY_APF_conver_T APF_Y;
    }traj_planner;

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

    void write_on_SD(FILE*);
    
    data_t data;
};

#endif