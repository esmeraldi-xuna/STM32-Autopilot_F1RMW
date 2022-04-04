#include "mbed.h"
#include "GlobalData.hpp"
#include "cli_appereance.hpp"
/* #include "PI_contr.h"
#include "APF_conver.h"
#include "Kalman_filter_conv.h" */
#include "read_write_lock.hpp"

void GlobalData::display(){ 

    printf(GREEN("\n ----------- global data -----------------\n\n"));

    this->lock_sensor.read_lock();
    printf(RED("  SENSORS\n"));

   /*  printf("\033[2K"); // clear line
    printf("Altitude:    %.2f\n\n", data.sensors.altitude); */

    printf("\033[2K"); // clear line
    printf("FXOS8700CQ-Acc:     X:    %8.2f;    Y:    %8.2f;    Z:    %8.2f\n\n", data.sensors.a.x, data.sensors.a.y, data.sensors.a.z);
    printf("\033[2K"); // clear line
    printf("FXOS8700CQ-Mag:     X:    %8.2f;    Y:    %8.2f;    Z:    %8.2f\n\n", data.sensors.m.x, data.sensors.m.y, data.sensors.m.z);
    printf("\033[2K"); // clear line
    printf("ADXL345-Acc:    X:    %8.2f;    Y:    %8.2f;    Z:    %8.2f\n\n", data.sensors.a_ext.x, data.sensors.a_ext.y, data.sensors.a_ext.z);
    printf("\033[2K"); // clear line
    printf("ITG3200-Gyr:    X:    %8.2f;    Y:    %8.2f;    Z:    %8.2f\n\n", data.sensors.g_ext.x, data.sensors.g_ext.y, data.sensors.g_ext.z);
    //TODO: posL,posR ? 

    /* printf("\033[2K"); // clear line
    printf("Roll:   %8.2f;    Pitch:   %8.2f;    Yaw:   %8.2f\n\n", data.sensors.roll, data.sensors.pitch, data.sensors.yaw); */
    this->lock_sensor.read_unlock();

/*
    this->lock_cntr.read_lock();
    printf(RED("  PI CONTROLLER\n"));
    printf(" Ctrl_U\tCtrl_Y\n\n");
    this->lock_cntr.read_unlock();

    this->lock_nav.read_lock();
    printf(RED("  TRAJECTORY PLANNER\n"));
    printf(" APF_U\tAPF_Y\n\n");
    this->lock_nav.read_unlock();

    this->lock_ekf.read_lock();
    printf(RED("  EKF\n"));
    printf(" EKF_U\tEKF_Y\n\n");
    this->lock_ekf.read_unlock();
*/

    this->lock_pwm.read_lock();
    printf(RED("  PWM:\n"));
    printf("\033[2K"); // clear line
    printf("Motor L:    %d;   Motor R:    %d\n\n", data.pwm.motorL, data.pwm.motorR);
    this->lock_pwm.read_unlock();

    printf(GREEN("\n ---------------- end --------------------\n"));

    return;
}


void GlobalData::write_on_SD(FILE* out_file){

    this->lock_sensor.read_lock();
    fprintf(out_file, "FXOS8700CQ-Acc: X:%f Y:%f Z:%f", data.sensors.a.x, data.sensors.a.y, data.sensors.a.z);
    fprintf(out_file, "FXOS8700CQ-Mag: X:%f Y:%f Z:%f", data.sensors.m.x, data.sensors.m.y, data.sensors.m.z);
    fprintf(out_file, "ADXL345-Acc: X:%f Y:%f Z:%f", data.sensors.a_ext.x, data.sensors.a_ext.y, data.sensors.a_ext.z);
    fprintf(out_file, "ITG3200-Gyr: X:%f Y:%f Z:%f", data.sensors.g_ext.x, data.sensors.g_ext.y, data.sensors.g_ext.z);
    
    this->lock_sensor.read_unlock();

/*     this->lock_cntr.read_lock();
    fprintf(out_file, "CNTR: Ctrl_U, Ctrl_Y; ");
    this->lock_cntr.read_unlock();

    this->lock_nav.read_lock();
    fprintf(out_file, "TRJ_PL: APF_U, APF_Y; ");
    this->lock_nav.read_unlock();

    this->lock_ekf.read_lock();
    fprintf(out_file, "EKF: EKF_U, EKF_Y; ");
    this->lock_ekf.read_unlock(); */

    this->lock_pwm.read_lock();
    fprintf(out_file, "PWM: M1: %d, M2: %d.\n", data.pwm.motorL, data.pwm.motorR);
    this->lock_pwm.read_unlock();

    return;
}

void GlobalData::write_on_SD_as_csv(FILE* out_file){

    this->lock_sensor.read_lock();
    fprintf(out_file, "%f,%f,%f,", data.sensors.a.x, data.sensors.a.y, data.sensors.a.z);
    fprintf(out_file, "%f,%f,%f,", data.sensors.m.x, data.sensors.m.y, data.sensors.m.z);
    fprintf(out_file, "%f,%f,%f,",data.sensors.a_ext.x, data.sensors.a_ext.y, data.sensors.a_ext.z);
    fprintf(out_file, "%f,%f,%f,", data.sensors.g_ext.x, data.sensors.g_ext.y, data.sensors.g_ext.z);
    this->lock_sensor.read_unlock();

 /*    this->lock_cntr.read_lock();
    fprintf(out_file, "Ctrl_U,Ctrl_Y,");
    this->lock_cntr.read_unlock();

    this->lock_nav.read_lock();
    fprintf(out_file, "APF_U,APF_Y,");
    this->lock_nav.read_unlock();

    this->lock_ekf.read_lock();
    fprintf(out_file, "EKF_U,EKF_Y,");
    this->lock_ekf.read_unlock();
 */
    this->lock_pwm.read_lock();
    fprintf(out_file, "%d,%d\n", data.pwm.motorL, data.pwm.motorR);
    this->lock_pwm.read_unlock();

    return;
}

struct_sensors_data GlobalData::read_sensor(){
    struct_sensors_data tmp;
    this->lock_sensor.read_lock();
    tmp = this->data.sensors;
    this->lock_sensor.read_unlock();
    return tmp;
};

void GlobalData::write_sensor(struct_sensors_data data_in){
    this->lock_sensor.write_lock();
    this->data.sensors = data_in;
    this->lock_sensor.write_unlock();
    return;
};

struct_pwm_data GlobalData::read_pwm(){
    struct_pwm_data tmp;
    this->lock_pwm.read_lock();
    tmp = this->data.pwm;
    this->lock_pwm.read_unlock();
    return tmp;
};

void GlobalData::write_pwm(struct_pwm_data data_in){
    this->lock_pwm.write_lock();
    this->data.pwm = data_in;
    this->lock_pwm.write_unlock();
    return;
};

/* ExtY_Kalman_filter_conv_T GlobalData::read_ekf_Y(){
    ExtY_Kalman_filter_conv_T tmp;
    this->lock_ekf.read_lock();
    tmp = this->data.ekf.EKF_Y;
    this->lock_ekf.read_unlock();
    return tmp;
};

ExtU_Kalman_filter_conv_T GlobalData::read_ekf_U(){
    ExtU_Kalman_filter_conv_T tmp;
    this->lock_ekf.read_lock();
    tmp = this->data.ekf.EKF_U;
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

ExtY_APF_conver_T GlobalData::read_nav_Y(){
    ExtY_APF_conver_T tmp;
    this->lock_nav.read_lock();
    tmp = this->data.traj_planner.APF_Y;
    this->lock_nav.read_unlock();
    return tmp;
};

ExtU_APF_conver_T GlobalData::read_nav_U(){
    ExtU_APF_conver_T tmp;
    this->lock_nav.read_lock();
    tmp = this->data.traj_planner.APF_U;
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

ExtY_PI_contr_T GlobalData::read_cntr_Y(){
    ExtY_PI_contr_T tmp;
    this->lock_cntr.read_lock();
    tmp = this->data.controller.ctrl_Y;
    this->lock_cntr.read_unlock();
    return tmp;
};

ExtU_PI_contr_T GlobalData::read_cntr_U(){
    ExtU_PI_contr_T tmp;
    this->lock_cntr.read_lock();
    tmp = this->data.controller.ctrl_U;
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
 */
