#ifndef COMMANDER_H
#define COMMANDER_H

#include "mbed.h"

typedef enum {
	SYSTEM_WAKE_UP = 0,
	SYSTEM_READY,
    SYSTEM_ERROR
} system_states_t;

struct flags{
    bool flag_MPU9250_online = false;

    bool flag_AK8963_online = false;
    bool flag_AK8963_calibrated = false;

    bool flag_BMP180_online = false;

    bool flag_mavlink_communication = false;
};

class Commander
{
    public:
        Commander();

        struct flags all_flags;

        system_states_t get_current_state() {return curr_state;};
        int changeState(system_states_t new_state);
        
        bool arm();
        bool disarm() {flag_armed = false; return true;};

        bool is_armed() {return flag_armed;};

    private:
        system_states_t curr_state;
        bool flag_armed;
        rtos::Mutex lock_flags, lock_state;
};

#endif