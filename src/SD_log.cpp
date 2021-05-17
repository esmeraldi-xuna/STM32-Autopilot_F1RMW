#include <mbed.h>
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include "SD_log.hpp"
#include "global_vars.hpp"

#define FORCE_REFORMAT 0

// define pin for SD (change values on 'mbed_app.json')
SDBlockDevice sd(MBED_CONF_SD_SPI_MOSI, MBED_CONF_SD_SPI_MISO, MBED_CONF_SD_SPI_CLK, MBED_CONF_SD_SPI_CS); 

// Filesystem
FATFileSystem file_sys("fs");


void file_sys_init(void){

  // init SD card
  if (sd.init() != 0) {
      printf("SD Init failed \n");
      // return -1;
  }
  else{
    printf("SD card initialized with size: %llu\n", sd.size());
  }

  // Set the frequency
  if (sd.frequency(5000000) != 0) {
      printf("Error setting frequency \n");
  }

  // get some info
  /*
  printf("sd size: %llu\n",         sd.size());
  printf("sd read size: %llu\n",    sd.get_read_size());
  printf("sd program size: %llu\n", sd.get_program_size());
  printf("sd erase size: %llu\n",   sd.get_erase_size());
  */

  // mount filesystem
  printf("Mounting the filesystem...\n");

  fflush(stdout);
  int err = file_sys.mount(&sd);
  printf("%s\n", (err ? "Fail :(" : "Filesystem mounted"));
    
  if (err || FORCE_REFORMAT) {
    // Reformat if we can't mount the filesystem
    if(err)
      printf("Some error cause formatting\n");
      else
    printf("FORCE_REFORMAT flag cause formatting\n");
    
    fflush(stdout);
    err = file_sys.reformat(&sd);
    printf("%s\n", (err ? "Fail :(" : "Formattig OK"));
    if (err) {
      error("error: %s (%d)\n", strerror(-err), err);
    }

    // add file for logging
    printf("Creating a new file\n");
    fflush(stdout);
    FILE* f = fopen("/fs/log.txt", "w+");
    printf("%s\n", (!f ? "Fail :(" : "File created"));
    if (!f) {
      error("error: %s (%d)\n", strerror(errno), -errno);
    }
    else
      fclose(f);
  }

  return;
}


void SD_log_loop(void){

    Kernel::Clock::time_point log_time;
    std::chrono::milliseconds log_step = 500ms;

    FILE* log_file = fopen("/fs/log.txt", "w+");
    if (!log_file) {
      print_lock.lock();
      printf("Error opening log file\n");
      print_lock.unlock();
      return;
    }

    while (1)
    {
        log_time = Kernel::Clock::now();
        
        // wrinte on file
        fprintf(log_file, "new line\n");
        displayData_lock.lock();
        global_data->write_on_SD(log_file);
        displayData_lock.unlock();

        ThisThread::sleep_until(log_time + log_step);
    }
}