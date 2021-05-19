#include <mbed.h>
#include "cli_appereance.hpp"
#include "cli.hpp"
#include "global_vars.hpp"

void display_loop(void);
bool exit_flag = false;

void display_once(void){

    global_data->display();
    
    return;
}

void display_repeat(void){
    Thread cycle_th(osPriorityNormal, 2048, nullptr, "thread");
    
    exit_flag = false;
    
    cycle_th.start(display_loop);

    if (getchar()){ // press any button to exit
        // exit
        exit_flag=true;
        cycle_th.join();
    }
    ThisThread::sleep_for(50ms);

    return;
}

void display_loop(void){

    printf("\033[2J"); // clear screen
    printf("\033[1;1H"); // Set cursor to row 1 column 1
    printf("user@stm32 >> display_r");

    while(!exit_flag){
    
        printf("\033[2;1H"); // Set cursor to row 2 column 1
        
        global_data->display();

        ThisThread::sleep_for(1s);
    }

    return;
}