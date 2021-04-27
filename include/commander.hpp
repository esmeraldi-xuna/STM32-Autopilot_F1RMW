#ifndef COMMANDER_H
#define COMMANDER_H

typedef enum {
	COMMANDER_NOT_READY  = 0,
	COMMANDER_READY      = 1,
} commander_sates_t;

class Commander
{
    public:
        Commander();
        
        commander_sates_t get_current_state() {return curr_state;}
        int changeState(commander_sates_t new_state);
        
        void arm() {flag_armed = true;};
        void disarm() {flag_armed = false;};

        bool is_armed() {return flag_armed;};

        
    private:
        commander_sates_t curr_state;
        bool flag_armed;
};

#endif