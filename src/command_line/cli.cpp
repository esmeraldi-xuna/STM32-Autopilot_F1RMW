/*! @file cli2.cpp

    @brief Command line interface for hardware resource inspection.

    @details The command line interface (cli) allows the developer to check the CPU and memory usage, the number of
    active threads and display the main variables.
    The available commands are:
        - top           Display CPU usage;
        - info          Display hardware information;
        - thread        Show active threads state, priority and their resources;
        - clear         Clear screen;
        - help          Show available commands.

    
*/

// TODO Since Serial.h is deprecated in Mbed OS 6, the cli code has to be modified. read and write methods have to be used
// to get and put char. These methods can be used with callbacks (handle_command) and events to stop reading (hitting enter key)

#include "cli.hpp"
#include "cli_appereance.hpp"
#include "global_vars.hpp"

int ii;

const char* prompt = "user@stm32 >> ";

char tmp;

char cliBuffer[20];

__command command;

void cli(void)
{
    int i=0;

    print_lock.lock();
    printf("Commnad line init, thread ID: %d\n\r", (int)ThisThread::get_id());
    print_lock.unlock();

    // wait inizialization
    ThisThread::sleep_for(5s);

    print_lock.lock();
    // printf("\033[2J\033[1;1H");
    std::printf("New Terminal\n");
    print_lock.unlock();
    

    // start getting input
    while (1)
    {

        // console ready
        ThisThread::sleep_for(100ms);
        print_lock.lock();
        printf("\n%s",prompt);
        print_lock.unlock();
        ThisThread::sleep_for(100ms);
        
        i=0;
        do{
            tmp=getchar();
            cliBuffer[i]=tmp;
            i++;
            putchar(tmp);
        }while(tmp != '\n');
        cliBuffer[i-1]='\0';

        //printf("\nget: %s\n", cliBuffer);
        command = get_command(cliBuffer);

        print_lock.lock();
        switch (command)
        {
        case cmd_sys_info:
            sysinfo();
            break;

        case cmd_thread_info:
            threadinfo();
            break;

        case cmd_return:
            break;

        case cmd_clear:
            printf("\033[2J\033[1;1H");
            break;

        case cmd_help:
            help();
            break;

        case cmd_invalid:
            printf(RED("\nType a valid command\n"));
            break;
        }
        print_lock.unlock();
    }
}


__command get_command(char* str){

    if (strcmp(str, "info")== 0){
        return cmd_sys_info;
    }
    if (strcmp(str, "thread")== 0){
        return cmd_thread_info;
    }
    if (strcmp(str, "clear")== 0){
        return cmd_clear;
    }
    if (strcmp(str, "help")== 0){
        return cmd_help;
    }
    if (strcmp(str, "return")== 0){
        return cmd_return;
    }
    if (strcmp(str, "top")== 0){
        return cmd_top;
    }
    return cmd_invalid;
}

/*
void cli(BufferedSerial *serial)
{
    // serial->getc();
    // serial->read(buff, sizeof(buff));
    // serial->set_blocking(0);

    printf("Commnad line init, thread ID: %d\n\r", ThisThread::get_id());

    // wait inizialization
    ThisThread::sleep_for(5s);
    serial->read(firstbyte,sizeof(firstbyte));

    // ThisThread::sleep_for(5s);
    // printf("\033[2J\033[1;1H");
    // printf(GREEN_B("New Terminal\n"));
    
    // console ready
    // ThisThread::sleep_for(100);
    std::printf("%s",prompt);
    // ThisThread::sleep_for(100);

    // start getting input
    while (1)
    {
        // get one char at time untill press enter
        command = handleInput(serial);

        switch (command)
        {
        case cmd_top:
            printf("\033[8m"); // Blink off
            top(serial);
            printf("\033[28m"); // Blink on
            printf("\n%s",prompt);
            break;
            
        case cmd_sys_info:
            sysinfo();
            std::printf(prompt);
            break;

        case cmd_thread_info:
            threadinfo();
            printf("\n%s",prompt);
            break;

        case cmd_return:
            // serial->write(prompt, sizeof(prompt));
            std::printf("caz");
            break;

        case cmd_clear:
            printf("\033[2J\033[1;1H");
            printf("%s",prompt);
            break;

        case cmd_help:
            help();
            printf("\n%s",prompt);
            break;

        case cmd_invalid:
            printf(RED("\nType a valid command\n"));
            printf("%s",prompt);
            break;
        }
    }
    

}

// TODO: add cases for soft reboot (NVIC_SystemReset()), for help listing available commands and for displaying values from sensors!
// define a function in the sensor values struct that prints its fields
__command handleInput(BufferedSerial *serial)
{
    char tmp_byte[1]; 

    // memset(cli_buff,'\0', sizeof(cli_buff));
    cliBuffer.clear();

    // By default, the enter command is set.
    cmd_enum = cmd_return; 
    
    ii = 0;
    while (ii < 127) // max len of buffer
    {
        // char byte = serial->getc();
        // char byte[1];
        tmp_byte[0] = '\0';
        
        // printf("\tgetting single char\t");
        serial->read(tmp_byte, sizeof(tmp_byte));
        
        // write char (for debug)
        // serial->putc(byte);
        serial->write(tmp_byte, sizeof(tmp_byte));
        // cout << byte;
        
        // if pressed enter return
        if (!strcmp(tmp_byte,"\r")) // 
        {
            if(ii == 0)
            {
                cliBuffer = "return";
            }
            ii = 0;
            break;
        }
        
        // add char to buffer
        cliBuffer.append(tmp_byte);
        // cli_buff[ii] = byte;
        ii++;
    }
    if(!cliBuffer.compare("top"))                       cmd_enum = cmd_top;
    else if(!cliBuffer.compare("info"))                 cmd_enum = cmd_sys_info;
    else if(!cliBuffer.compare("thread"))               cmd_enum = cmd_thread_info;
    else if(!cliBuffer.compare("return"))               cmd_enum = cmd_return;
    else if(!cliBuffer.compare("clear"))                cmd_enum = cmd_clear;
    else if(!cliBuffer.compare("help"))                 cmd_enum = cmd_help;
    else                                                cmd_enum = cmd_invalid;

    cliBuffer.clear();

    return cmd_enum;
}*/