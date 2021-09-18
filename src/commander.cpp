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

    if(all_flags.PWM.force_disable){

        print_lock.lock();
        printf("Arming error: PWM force disable == TRUE\n");
        print_lock.lock();
        return false;
    }

    this->lock_flags.lock();
    
    // check communication
    if( all_flags.comm_mavlink_rx == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: communication MAV rx\n");
        print_lock.lock();
    }

    if( all_flags.comm_mavlink_tx == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: communication MAV tx\n");
        print_lock.lock();
    }

    if( all_flags.comm_joystick == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: communication JOY\n");
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

void Commander::set_p_to_FSM_state(FSM_STATES* pointer_to_main_state){
    p_main_FSM_state = pointer_to_main_state;
}

FSM_STATES Commander::get_main_FMS_state(){
    return (*p_main_FSM_state);    
}

bool Commander::check_mandatory(){

    bool all_ok = true;

    // check sensors
    if(!all_flags.sensor.flag_MPU9250_online)
        all_ok = false;

    if(!all_flags.sensor.flag_AK8963_online)
        all_ok = false;

    if(!all_flags.sensor.flag_BMP180_online)
        all_ok = false;

    
    // check joystick communication
    if(!all_flags.comm_joystick)
        all_ok = false;

    return all_ok;
}

bool Commander::check_startup(){
    
    bool all_ok = true;

    // check mavlink
    if(!all_flags.comm_mavlink_rx)
        all_ok = false;

    if(!all_flags.comm_mavlink_tx)
        all_ok = false;

    // check controller
    if(!all_flags.controller_active)
        all_ok = false;

    if(!all_flags.ekf_active)
        all_ok = false;
    
    if(!all_flags.apf_active)
        all_ok = false;

    // check pwm thread
    if(!all_flags.PWM.active)
        all_ok = false;

    return all_ok;
}

bool Commander::check_run_auto(){
    
    bool all_ok = true;

    // check mavlink
    if(!all_flags.comm_mavlink_rx)
        all_ok = false;

    if(!all_flags.comm_mavlink_tx)
        all_ok = false;

    // check controller
    if(!all_flags.controller_active)
        all_ok = false;

    if(!all_flags.ekf_active)
        all_ok = false;
    
    if(!all_flags.apf_active)
        all_ok = false;

    // check pwm thread
    if(!all_flags.PWM.active)
        all_ok = false;

    return all_ok;
}

bool Commander::check_run_manual(){
    
    bool all_ok = true;

    // check joystick communication
    if(!all_flags.comm_joystick)
        all_ok = false;

    // check pwm thread
    if(!all_flags.PWM.active)
        all_ok = false;

    return all_ok;
}

void Commander::force_PWM_disable(){
    all_flags.PWM.force_enable = false;
    all_flags.PWM.force_disable = true;
}

void Commander::force_PWM_enable(){
    all_flags.PWM.force_disable = false;
    all_flags.PWM.force_enable = true;
} 

bool Commander::show_all_flags(){
    return true;
}


