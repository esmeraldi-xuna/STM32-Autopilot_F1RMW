// commander class

#include "commander.hpp"

Commander::Commander()
{
    curr_state=COMMANDER_NOT_READY;
    flag_armed = false;
};

int Commander::changeState(commander_sates_t new_state)
{
    curr_state = new_state;
    return 1;
};