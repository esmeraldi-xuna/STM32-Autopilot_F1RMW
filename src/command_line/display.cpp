#include <mbed.h>
#include "cli_appereance.hpp"
#include "cli.hpp"
#include "global_vars.hpp"

void display_event(Mutex*);

void display_once(void){

    // lock for global_data obj
    displayData_lock.lock();
    global_data->display();
    displayData_lock.unlock();

    return;
}

void display_repeat(void){
    
    EventQueue *stats_queue = mbed_event_queue();
    int id;
    rtos::Mutex* lock_screen;
    lock_screen = new Mutex();

    id = stats_queue->call_every(1s, callback(display_event, lock_screen));

    printf("\033[2J\033[1;1H"); // Clear terminal (2J) and set cursor to 1,1 (1;1H) --> CSI codes wikipedia!!!
    
    printf("user@stm32 >> display_r");
    
    if (getchar()){ // press any button to exit
        // stop queue and exit
        stats_queue->cancel(id);
    }
    ThisThread::sleep_for(50ms);

    // wait untill last print is ended
    lock_screen->lock();
    printf("\n");
    lock_screen->unlock();
    
    return;
}

void display_event(Mutex* lock_screen){

    printf("\033[2;1H\033[5K"); // Set cursor to row 2 column 1 and clear that line

    // lock for synch end printing
    lock_screen->lock();
    
    // lock for global_data obj
    displayData_lock.lock();
    global_data->display();
    displayData_lock.unlock();
    lock_screen->unlock();

    return;
}