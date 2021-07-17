// commander class

#include "commander.hpp"
#include "global_vars.hpp"

Commander::Commander()
{
    curr_state=SYSTEM_ERROR;
    flag_armed = false;

    all_flags.flag_AK8963_online = false;
    all_flags.flag_AK8963_calibrated = false;
    
    all_flags.flag_BMP180_online = false;
    
    all_flags.flag_MPU9250_online = false;

    all_flags.flag_mavlink_communication = false;
};

int Commander::changeState(system_states_t new_state)
{
    this->lock_state.lock();
    curr_state = new_state;
    this->lock_state.unlock();
    return 1;
};

bool Commander::arm(){
    bool can_arm = true;

    this->lock_flags.lock();
    
    // check communication
    if( all_flags.flag_mavlink_communication == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: communication\n");
        print_lock.lock();
    }

    // check sensors
    if( all_flags.flag_MPU9250_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor MPU9250 not online\n");
        print_lock.lock();
    }

    if( all_flags.flag_BMP180_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor BMP180 not online\n");
        print_lock.lock();
    }

    if( all_flags.flag_AK8963_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor AK8963 not online\n");
        print_lock.lock();
    }

    if( all_flags.flag_AK8963_calibrated == false){
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