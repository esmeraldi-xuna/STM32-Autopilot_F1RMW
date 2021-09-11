// commander class

#include "commander.hpp"
#include "global_vars.hpp"

Commander::Commander()
{
    this->set_all_flags_to_zero();
};

void Commander::set_all_flags_to_zero(){

    flag_armed = false;

    // sensors state flags
    all_flags.sensor.flag_AK8963_online = false;
    all_flags.sensor.flag_AK8963_calibrated = false;
    all_flags.sensor.flag_BMP180_online = false;
    all_flags.sensor.flag_MPU9250_online = false;
    all_flags.sensor.flag_mavlink_communication = false;

    // motor state flags
    all_flags.PWM.active = false;
    all_flags.PWM.force_enable = false;
    all_flags.PWM.force_disable = false;
    all_flags.PWM.system_enable = false;

    // controller state flags
    all_flags.controller_active = false;
    all_flags.ekf_active = false;
    all_flags.apf_active = false;

    // communications state flags
    all_flags.comm_mavlink_rx = false;
    all_flags.comm_mavlink_tx = false;
    all_flags.comm_joystick = false;
}

bool Commander::arm(){
    bool can_arm = true;

    this->lock_flags.lock();
    
    // check communication
    if( all_flags.sensor.flag_mavlink_communication == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: communication\n");
        print_lock.lock();
    }

    // check sensors
    if( all_flags.sensor.flag_MPU9250_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor MPU9250 not online\n");
        print_lock.lock();
    }

    if( all_flags.sensor.flag_BMP180_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor BMP180 not online\n");
        print_lock.lock();
    }

    if( all_flags.sensor.flag_AK8963_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor AK8963 not online\n");
        print_lock.lock();
    }

    if( all_flags.sensor.flag_AK8963_calibrated == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor AK9863 not calibrated\n");
        print_lock.lock();
    }

    // if all ok -> arm
    if(can_arm){
        this->flag_armed = true;
    }
    else{
        this->flag_armed = false;
        print_lock.lock();
        printf("Arming denied, something gone wrong...\n");
        print_lock.lock();
    }
    this->lock_flags.unlock();
    return this->flag_armed;
}

bool Commander::check_mandatory(){
    return true;
}

bool Commander::check_startup(){
    return true;
}

bool Commander::check_run_auto(){
    return true;
}

bool Commander::check_run_manual(){
    return true;
}

bool Commander::show_all_flags(){
    return true;
}