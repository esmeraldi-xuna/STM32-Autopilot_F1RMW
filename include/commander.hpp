#ifndef COMMANDER_H
#define COMMANDER_H

#include "mbed.h"
#include "GlobalData.hpp"
#include "rtos.h"
struct flags{

    // sensors state flags
    struct sensors_t{
        bool flag_ADXL345_online = false;
        bool flag_ADXL345_calibrated = false;

        bool flag_FXOS8700CQ_online = false;
        bool flag_FXOS8700CQ_calibrated = false;

        bool flag_ITG3200_online = false;
        //bool flag_ITG3200_calibrated = false;
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

        Commander();

        void set_all_flags_to_zero();
        
        bool arm();
        bool disarm() {flag_armed = false; return true;};

        bool is_armed() {return flag_armed;};

        bool check_init();
        bool check_startup();
        bool check_run_auto();
        bool check_run_manual();
        
        void set_p_to_FSM_state(FSM_STATES*);
        FSM_STATES get_main_FMS_state();
        bool show_all_flags();

        void force_PWM_disable();
        void force_PWM_enable(); 

        // getter - setter for online/calibration flags
        bool get_flag_ADXL345_online();
        void set_flag_ADXL345_online(bool);
        bool get_flag_FXOS8700CQ_online();
        void set_flag_FXOS8700CQ_online(bool);
        bool get_flag_ITG3200_online();
        void set_flag_ITG3200_online(bool);  

        bool get_flag_ADXL345_calibrated();
        void set_flag_ADXL345_calibrated(bool);
        bool get_flag_FXOS8700CQ_calibrated();
        void set_flag_FXOS8700CQ_calibrated(bool);
/*         bool get_flag_ITG3200_calibrated();
        void set_flag_ITG3200_calibrated(bool); */

        bool get_pwm_active();
        void set_pwm_active(bool);

        bool get_force_pwm_enable();
        void set_force_pwm_enable(bool);

        bool get_force_pwm_disable();
        void set_force_pwm_disable(bool);

        bool get_system_enable();
        void set_system_enable(bool);
        
        bool get_flag_controller_active();
        void set_flag_controller_active(bool);
        
        bool get_flag_ekf_active();
        void set_flag_ekf_active(bool);
        
        bool get_flag_apf_active();
        void set_flag_apf_active(bool);

        bool get_flag_comm_mavlink_rx();
        void set_flag_comm_mavlink_rx(bool);
        
        bool get_flag_comm_mavlink_tx();
        void set_flag_comm_mavlink_tx(bool);
        
        bool get_flag_comm_joystick();
        void set_flag_comm_joystick(bool);

    private:

        struct flags all_flags;
        
        bool flag_armed;
        rtos::Mutex lock_state;
        Read_Write_Lock lock_flags;
        FSM_STATES* p_main_FSM_state;
};

#endif