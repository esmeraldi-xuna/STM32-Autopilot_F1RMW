#include <mbed.h>
#include "cli.hpp"
#include "global_vars.hpp"

void reset(void){

    print_lock.lock();
    printf("Waiting for system reset\n");
    print_lock.unlock();

    ThisThread::sleep_for(1s);

    NVIC_SystemReset();
    return;
}