// commander class

#include "commander.hpp"
#include "global_vars.hpp"

Commander::Commander()
{
    this->set_all_flags_to_zero();
    p_main_FSM_state = nullptr;
};

void Commander::set_all_flags_to_zero(){

    lock_flags.write_lock();

    flag_armed = false;

    // sensors state flags
    all_flags.sensor.flag_ADXL345_online = false;
    all_flags.sensor.flag_ADXL345_calibrated = false;
    all_flags.sensor.flag_FXOS8700CQ_online = false;
    all_flags.sensor.flag_FXOS8700CQ_calibrated = false;
    all_flags.sensor.flag_ITG3200_online = false;

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

    lock_flags.write_unlock();
}

//BUGGED? all print locks not unlocked!!!!
bool Commander::arm(){
    bool can_arm = true;

    lock_flags.read_lock();

    if(all_flags.PWM.force_disable){

        print_lock.lock();
        printf("Arming error: PWM force disable == TRUE\n");
        print_lock.lock();
        return false;
    }
    
    // check communication
    if( all_flags.comm_mavlink_rx == false){
        can_arm = false || true; //CAMBIARE IN FALSE SOLO, forzato l'or per passare i controlli
        print_lock.lock();
        printf("Arming error: communication MAV rx\n");
        print_lock.lock();
    }

    if( all_flags.comm_mavlink_tx == false){
        can_arm = false || true;
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
    if( all_flags.sensor.flag_ADXL345_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor ADXL345 not online\n");
        print_lock.lock();
    }

    if( all_flags.sensor.flag_FXOS8700CQ_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor FXOS8700CQ not online\n");
        print_lock.lock();
    }

    if( all_flags.sensor.flag_ITG3200_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor ITG3200 not online\n");
        print_lock.lock();
    }

    if( all_flags.sensor.flag_ADXL345_calibrated == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor ADXL345 not calibrated\n");
        print_lock.lock();
    }
    if( all_flags.sensor.flag_FXOS8700CQ_calibrated == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor FXOS8700CQ not calibrated\n");
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
    lock_flags.read_unlock();
    return this->flag_armed;
}

void Commander::set_p_to_FSM_state(FSM_STATES* pointer_to_main_state){
    p_main_FSM_state = pointer_to_main_state;
}

FSM_STATES Commander::get_main_FMS_state(){
    if(p_main_FSM_state != nullptr)
        return (*p_main_FSM_state);
    else 
        return (FSM_STATES)-1;    
}

bool Commander::check_init(){

    lock_flags.read_lock();

    bool all_ok = true;

     // check joystick communication
    if(!all_flags.comm_joystick)
        all_ok = false || true;
    // check sensors
    if(!all_flags.sensor.flag_ADXL345_online)
        all_ok = false; //TO DO 

    if(!all_flags.sensor.flag_FXOS8700CQ_online)
        all_ok = false;

    if(!all_flags.sensor.flag_ITG3200_online)
        all_ok = false;

    
   

    lock_flags.read_unlock();

    return all_ok;
}

bool Commander::check_startup(){
    
    lock_flags.read_lock();

    bool all_ok = true;

    // check mavlink
    if(!all_flags.comm_mavlink_rx)
        all_ok = false || true;

    if(!all_flags.comm_mavlink_tx)
        all_ok = false  || true;

    // check controller
    if(!all_flags.controller_active)
        all_ok = false || true;

    if(!all_flags.ekf_active)
        all_ok = false || true;
    
    if(!all_flags.apf_active)
        all_ok = false || true;

    // check pwm thread
    if(!all_flags.PWM.active){
        printf("PWM active false\n");
        all_ok = false;
    }

    lock_flags.read_unlock();

    return all_ok;
}

bool Commander::check_run_auto(){
    
    lock_flags.read_lock();

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

    lock_flags.read_unlock();

    return all_ok;
}

bool Commander::check_run_manual(){
    
    lock_flags.read_lock();

    bool all_ok = true;

    // check joystick communication
    if(!all_flags.comm_joystick)
        all_ok = false;

    // check pwm thread
    if(!all_flags.PWM.active)
        all_ok = false;

    lock_flags.read_unlock();

    return all_ok;
}

void Commander::force_PWM_disable(){
    
    lock_flags.write_lock();
    
    all_flags.PWM.force_enable = false;
    all_flags.PWM.force_disable = true;

    lock_flags.write_unlock();
}

void Commander::force_PWM_enable(){

    lock_flags.write_lock();

    all_flags.PWM.force_disable = false;
    all_flags.PWM.force_enable = true;

    lock_flags.write_unlock();
} 

bool Commander::show_all_flags(){

    lock_flags.read_lock();

    lock_flags.read_unlock();

    return true;
}

// getter - setter for flags
bool Commander::get_flag_ADXL345_online()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.sensor.flag_ADXL345_online;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_ADXL345_online(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.sensor.flag_ADXL345_online = value_to_set;
    lock_flags.write_unlock();
    return;
}
bool Commander::get_flag_FXOS8700CQ_online()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.sensor.flag_FXOS8700CQ_online;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_FXOS8700CQ_online(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.sensor.flag_FXOS8700CQ_online = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_ITG3200_online()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.sensor.flag_ITG3200_online;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_ITG3200_online(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.sensor.flag_ITG3200_online = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_ADXL345_calibrated()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.sensor.flag_ADXL345_calibrated;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_ADXL345_calibrated(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.sensor.flag_ADXL345_calibrated = value_to_set;
    lock_flags.write_unlock();
    return;
}
bool Commander::get_flag_FXOS8700CQ_calibrated()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.sensor.flag_FXOS8700CQ_calibrated;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_FXOS8700CQ_calibrated(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.sensor.flag_FXOS8700CQ_calibrated = value_to_set;
    lock_flags.write_unlock();
    return;
}


bool Commander::get_pwm_active()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.PWM.active;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_pwm_active(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.PWM.active = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_force_pwm_enable()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.PWM.force_enable;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_force_pwm_enable(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.PWM.force_enable = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_force_pwm_disable()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.PWM.force_disable;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_force_pwm_disable(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.PWM.force_disable = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_system_enable()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.PWM.system_enable;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_system_enable(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.PWM.system_enable = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_controller_active()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.ekf_active;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_controller_active(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.controller_active = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_ekf_active()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.ekf_active;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_ekf_active(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.ekf_active = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_apf_active()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.apf_active;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_apf_active(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.apf_active = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_comm_mavlink_rx()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.comm_mavlink_rx;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_comm_mavlink_rx(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.comm_mavlink_rx = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_comm_mavlink_tx()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.comm_mavlink_tx;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_comm_mavlink_tx(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.comm_mavlink_tx = value_to_set;
    lock_flags.write_unlock();
    return;
}

bool Commander::get_flag_comm_joystick()
{
    bool tmp = false;
    lock_flags.read_lock();
    tmp = all_flags.comm_joystick;
    lock_flags.read_unlock();

    return tmp;
}
void Commander::set_flag_comm_joystick(bool value_to_set)
{
    lock_flags.write_lock();
    all_flags.comm_joystick = value_to_set;
    lock_flags.write_unlock();
    return;
}

