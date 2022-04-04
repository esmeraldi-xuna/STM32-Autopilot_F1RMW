#ifndef DISPLAY_DATA_H
#define DISPLAY_DATA_H

#include "mbed.h"
/* #include "PI_contr.h"
#include "APF_conver.h"
#include "Kalman_filter_conv.h" */
#include "read_write_lock.hpp"


typedef enum {
    SYS_INIT = 0,
    SYS_STARTUP,
    SYS_FAIL_MAJOR,
    SYS_FAIL_MINOR,
    SYS_SAFE,
    SYS_RUN_AUTO,
    SYS_RUN_MANUAL
}FSM_STATES;

typedef struct a{
    float x = 0;
    float y = 0;
    float z = 0;
}tuple_t;

typedef struct t{
    //Internal sensors K64F, acc-mag
    tuple_t a;
    tuple_t m;
    /* float ax = 0;
    float ay = 0;
    float az = 0;
    float mx = 0;
    float my = 0;
    float mz = 0; */
    //External IMU
    tuple_t a_ext;
    tuple_t g_ext;
/*     float gx = 0;
    float gy = 0;
    float gz = 0;
    float pitch = 0;
    float roll = 0;
    float yaw = 0;
    float altitude = 0;  */
    // 
    int32_t posL = 0;
    int32_t posR = 0;
    float speedL = 0;
    float speedR = 0;
}struct_sensors_data;

typedef struct p{
    int motorL = 0;
    int motorR = 0;
}struct_pwm_data;


typedef struct data_type{
    struct_sensors_data sensors;

/*     struct y{
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
    }ekf; */

    struct_pwm_data pwm;
} data_t;


class GlobalData
{
    public:
    GlobalData(){};

    void display();

    void write_on_SD(FILE*);
    void write_on_SD_as_csv(FILE*);

    struct_sensors_data read_sensor();
    void write_sensor(struct_sensors_data);

    /* ExtY_Kalman_filter_conv_T read_ekf_Y();
    ExtU_Kalman_filter_conv_T read_ekf_U();
    void write_ekf(ExtU_Kalman_filter_conv_T, ExtY_Kalman_filter_conv_T);

    ExtY_APF_conver_T read_nav_Y();
    ExtU_APF_conver_T read_nav_U();
    void write_nav(ExtU_APF_conver_T, ExtY_APF_conver_T);

    ExtY_PI_contr_T read_cntr_Y();
    ExtU_PI_contr_T read_cntr_U();
    void write_cntr(ExtU_PI_contr_T, ExtY_PI_contr_T);
 */
    struct_pwm_data read_pwm();
    void write_pwm(struct_pwm_data);
    
    private:
    data_t data;

    Read_Write_Lock lock_sensor,/* , lock_ekf, lock_nav, lock_cntr, */ lock_pwm;
};

#endif