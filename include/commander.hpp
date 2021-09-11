#ifndef COMMANDER_H
#define COMMANDER_H

#include "mbed.h"

struct flags{

    // sensors state flags
    struct sensors_t{
        bool flag_MPU9250_online = false;

        bool flag_AK8963_online = false;
        bool flag_AK8963_calibrated = false;

        bool flag_BMP180_online = false;

        bool flag_mavlink_communication = false;
    } sensor;

    // motor state flags
    struct PWM_t{
        bool active = false;
        bool force_enable = false;
        bool force_disable = false;
        bool system_enable = false; // same as armed_flag ?
    } PWM;

    // controller state flags
    bool controller_active = false;
    bool ekf_active = false;
    bool apf_active = false;

    // communications state flags
    bool comm_mavlink_rx = false;
    bool comm_mavlink_tx = false;
    bool comm_joystick = false;
};

class Commander
{
    public:
        struct flags all_flags;

        Commander();

        void set_all_flags_to_zero();
        
        bool arm();
        bool disarm() {flag_armed = false; return true;};

        bool is_armed() {return flag_armed;};

        bool check_mandatory();
        bool check_startup();
        bool check_run_auto();
        bool check_run_manual();
        
        bool show_all_flags();

    private:
        //system_states_t curr_state;
        bool flag_armed;
        rtos::Mutex lock_flags, lock_state;
};

#endif