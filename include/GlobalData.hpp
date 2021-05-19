#ifndef DISPLAY_DATA_H
#define DISPLAY_DATA_H

#include "mbed.h"
#include "PI_contr.h"
#include "APF_conver.h"
#include "Kalman_filter_conv.h"
#include "read_write_lock.hpp"


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

    struct e{
        ExtU_Kalman_filter_conv_T EKF_U;
        ExtY_Kalman_filter_conv_T EKF_Y;
    }ekf;

    struct x{
        int motor1=0;
        int motor2=0;
    }pwm;
} data_t;


class GlobalData
{
    public:
    GlobalData(){};

    void display();

    void write_on_SD(FILE*);

    void read_sensor(); //todo
    void write_sensor(); //todo

    ExtY_Kalman_filter_conv_T read_ekf();
    void write_ekf(ExtU_Kalman_filter_conv_T, ExtY_Kalman_filter_conv_T);

    ExtY_APF_conver_T read_nav();
    void write_nav(ExtU_APF_conver_T, ExtY_APF_conver_T);

    ExtY_PI_contr_T read_cntr();
    void write_cntr(ExtU_PI_contr_T, ExtY_PI_contr_T);

    void read_pwm(); //todo
    void write_pwm(); //todo
    
    private:
    data_t data;

    Read_Write_Lock lock_sensor, lock_ekf, lock_nav, lock_cntr, lock_pwm;
};

#endif