#include <mbed.h>
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include "SD_log.hpp"
#include "global_vars.hpp"

#define FORCE_REFORMAT 0

// define pin for SD (change values on 'mbed_app.json')
SDBlockDevice sd_block(MBED_CONF_SD_SPI_MOSI, MBED_CONF_SD_SPI_MISO, MBED_CONF_SD_SPI_CLK, MBED_CONF_SD_SPI_CS); 

// Filesystem
FATFileSystem file_sys("sd");

void file_sys_init(void){

  if (FORCE_REFORMAT){
    printf("Force formatting... ");
    fflush(stdout);

    int err = file_sys.reformat(&sd_block);
    printf("%s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        printf("error: reformat\n");
    }
  }
  else{
    printf("Mounting the filesystem... ");
    fflush(stdout);

    int err = file_sys.mount(&sd_block);
    printf("%s\n", (err ? "Fail :(" : "OK"));
    if (err) {
        printf("No filesystem found, formatting... ");
        fflush(stdout);

        err = file_sys.reformat(&sd_block);
        printf("%s\n", (err ? "Fail :(" : "OK"));
        if (err) {
            printf("error: reformat\n");
        }
    }
  }

  FILE* f = fopen("sd/log.txt", "w+");
  printf("%s\n", (!f ? "Fail :(" : "Created File for log"));
  if (!f) {
    printf("error file open\n");
  }
  else
    fclose(f);

  return;
}


void SD_log_loop(void){

  Kernel::Clock::time_point log_time;
  std::chrono::milliseconds log_step = 500ms;

  print_lock.lock();
  printf("Start LOG_SD thread ID: %d\n", (int)ThisThread::get_id());
  print_lock.unlock();

  FILE* log_file = fopen("sd/log.txt", "w+");
  if (!log_file) {
    print_lock.lock();
    printf("Error opening log file\n");
    print_lock.unlock();
    return;
  }

  // write first line for csv file
  fprintf(log_file, "TIME (ms), ACC, CNTRL, insert all fields separated by , ");

  while (1)
  {
    log_time = Kernel::Clock::now();
    
    // wrinte on file
    fprintf(log_file, "%u,", Kernel::get_ms_count());
    
    // global_data->write_on_SD(log_file);
    global_data->write_on_SD_as_csv(log_file);
    
    ThisThread::sleep_until(log_time + log_step);
  }
}