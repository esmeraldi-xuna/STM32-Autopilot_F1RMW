#ifndef COMMANDER_H
#define COMMANDER_H

#include "mbed.h"

typedef enum {
	COMMANDER_NOT_READY  = 0,
	COMMANDER_READY      = 1,
} commander_sates_t;

struct flags{
    bool flag_sensor_online = false;
    bool flag_sensor_calibrated = false;
    bool flag_mav_comm = false;
};

class Commander
{
    public:
        Commander();
        
        commander_sates_t get_current_state() {return curr_state;}
        int changeState(commander_sates_t new_state);
        
        void arm();
        void disarm() {flag_armed = false;};

        bool is_armed() {return flag_armed;};

        void set_mav_comm_flag(bool);
        
    private:
        commander_sates_t curr_state;
        bool flag_armed;
        struct flags all_flags;
        rtos::Mutex lock_flags, lock_state;
};

#endif