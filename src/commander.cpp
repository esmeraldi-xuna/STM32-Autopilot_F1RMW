// commander class

#include "commander.hpp"
#include "global_vars.hpp"

Commander::Commander()
{
    curr_state=COMMANDER_NOT_READY;
    flag_armed = false;
};

int Commander::changeState(commander_sates_t new_state)
{
    this->lock_state.lock();
    curr_state = new_state;
    this->lock_state.unlock();
    return 1;
};

void Commander::arm(){
    bool can_arm = true;

    this->lock_flags.lock();
    if( all_flags.flag_mav_comm == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: communication\n");
        print_lock.lock();
    }

    if( all_flags.flag_sensor_online == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor not online\n");
        print_lock.lock();
    }

    if( all_flags.flag_sensor_calibrated == false){
        can_arm = false;
        print_lock.lock();
        printf("Arming error: sensor not calibrated\n");
        print_lock.lock();
    }

    if(can_arm){
        this->flag_armed = true;
    }
    else{
        this->flag_armed = false;
        print_lock.lock();
        printf("Arming error, something gone wrong...\n");
        print_lock.lock();
    }
    this->lock_flags.unlock();
    return;
}

void Commander::set_mav_comm_flag(bool new_value){
    this->lock_flags.lock();
    all_flags.flag_mav_comm = new_value;
    this->lock_flags.unlock();
    return;
}