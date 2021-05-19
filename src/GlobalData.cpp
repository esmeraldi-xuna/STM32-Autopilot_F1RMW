#include "mbed.h"
#include "GlobalData.hpp"
#include "cli_appereance.hpp"
#include "PI_contr.h"
#include "APF_conver.h"
#include "Kalman_filter_conv.h"
#include "read_write_lock.hpp"

void GlobalData::display(){

    printf(GREEN("\n ----------- global data -----------------\n\n"));

    this->lock_sensor.read_lock();
    printf(RED("  SENSORS\n"));
    printf("Altitude: %d\tAcceleration: %d\n\n", data.sensors.altitude, data.sensors.accel);
    this->lock_sensor.read_unlock();

    this->lock_cntr.read_lock();
    printf(RED("  PI CONTROLLER\n"));
    printf(" Ctrl_U\tCtrl_Y\n\n");
    this->lock_cntr.read_unlock();

    this->lock_nav.read_lock();
    printf(RED("  TRAJECTORY PLANNER\n"));
    printf(" APF_U\tAPF_Y\n\n");
    this->lock_nav.read_unlock();

    this->lock_pwm.read_lock();
    printf(RED("  PWM:\n"));
    printf("Motor 1: %d\tMotor 2: %d\n\n", data.pwm.motor1, data.pwm.motor2);
    this->lock_pwm.read_unlock();

    printf(GREEN("\n ---------------- end --------------------\n"));

    return;
}


void GlobalData::write_on_SD(FILE* out_file){

    this->lock_sensor.read_lock();
    fprintf(out_file, "SENS: Alt: %d, Acc: %d; ", data.sensors.altitude, data.sensors.accel);
    this->lock_sensor.read_unlock();

    this->lock_cntr.read_lock();
    fprintf(out_file, "CNTR: Ctrl_U, Ctrl_Y; ");
    this->lock_cntr.read_unlock();

    this->lock_nav.read_lock();
    fprintf(out_file, "TRJ_PL: APF_U, APF_Y; ");
    this->lock_nav.read_unlock();

    this->lock_pwm.read_lock();
    fprintf(out_file, "PWM: M1: %d, M2: %d.\n", data.pwm.motor1, data.pwm.motor2);
    this->lock_pwm.read_unlock();

    return;
}

void GlobalData::read_sensor(){ // todo
    this->lock_sensor.read_lock();

    this->lock_sensor.read_unlock();
    return;
};

void GlobalData::write_sensor(){ // todo
    this->lock_sensor.write_lock();

    this->lock_sensor.write_unlock();
    return;
};

void GlobalData::read_pwm(){ // todo
    this->lock_pwm.read_lock();

    this->lock_pwm.read_unlock();
    return;
};

void GlobalData::write_pwm(){ // todo
    this->lock_pwm.write_lock();

    this->lock_pwm.write_unlock();
    return;
};

ExtY_Kalman_filter_conv_T GlobalData::read_ekf(){
    ExtY_Kalman_filter_conv_T tmp;

    this->lock_ekf.read_lock();
    tmp = this->data.ekf.EKF_Y;
    this->lock_ekf.read_unlock();
    return tmp;
};
  
void GlobalData::write_ekf(ExtU_Kalman_filter_conv_T U, ExtY_Kalman_filter_conv_T Y){
    this->lock_ekf.write_lock();
    this->data.ekf.EKF_U = U;
    this->data.ekf.EKF_Y = Y;
    this->lock_ekf.write_unlock();
    return;
};

ExtY_APF_conver_T GlobalData::read_nav(){
    ExtY_APF_conver_T tmp;

    this->lock_nav.read_lock();
    tmp = this->data.traj_planner.APF_Y;
    this->lock_nav.read_unlock();
    return tmp;
};

void GlobalData::write_nav(ExtU_APF_conver_T U, ExtY_APF_conver_T Y){
    this->lock_nav.write_lock();
    this->data.traj_planner.APF_U = U;
    this->data.traj_planner.APF_Y = Y;
    this->lock_nav.write_unlock();
    return;
};

ExtY_PI_contr_T GlobalData::read_cntr(){
    ExtY_PI_contr_T tmp;

    this->lock_cntr.read_lock();
    tmp = this->data.controller.ctrl_Y;
    this->lock_cntr.read_unlock();
    return tmp;
};

void GlobalData::write_cntr(ExtU_PI_contr_T U, ExtY_PI_contr_T Y){
    this->lock_cntr.write_lock();
    this->data.controller.ctrl_U = U;
    this->data.controller.ctrl_Y = Y;
    this->lock_cntr.write_unlock();
    return;
};

