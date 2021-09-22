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

// Log file name
char name[30];

void file_sys_init(void){

  int tmp = 0, max = 0;

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

  //////////////////////////////////////////////////////////////

  // search highest value of log file
  DIR *d = opendir("sd/");
  if (!d) {
      printf("error open dir\n");
  }

  while (true) {
      struct dirent *e = readdir(d);
      if (!e) {
          break;
      }
      // printf("    %s\n", e->d_name);
      sscanf(e->d_name, "log%d.txt", &tmp);
      if (tmp > max)
        max = tmp;
  }

  int err = closedir(d);
  if (err < 0) {
      printf("error close dir\n");
  }

  sprintf(name, "sd/log%d.txt", max+1);
  //printf("%s\n", name);

  /////////////////////////////////////////////////////////////

  FILE* f = fopen(name, "w+");
  printf("%s", (!f ? "Fail :(\n" : "Created File for log: "));
  if (!f) {
    printf("error file open\n");
  }
  else{
    printf("%s\n", name);
    fclose(f);
  }
  
  return;
}


void SD_log_loop(void){

  Kernel::Clock::time_point log_time;
  std::chrono::milliseconds log_step = 500ms;

  print_lock.lock();
  printf("Start LOG_SD thread ID: %d\n", (int)ThisThread::get_id());
  print_lock.unlock();

  FILE* log_file = fopen(name, "w+");
  if (!log_file) {
    print_lock.lock();
    printf("Error opening log file\n");
    print_lock.unlock();
    return;
  }

  // write first line for csv file
  fprintf(log_file, "TIME (ms), ALT, ACC, CNTRL, insert all fields separated by , ");

  Kernel::Clock::time_point start_log = Kernel::Clock::now();
  while (1)
  {
    log_time = Kernel::Clock::now();
    
    // wrinte on file
    fprintf(log_file, "%lld,", (log_time - start_log).count());
    
    // global_data->write_on_SD(log_file);
    global_data->write_on_SD_as_csv(log_file);
    
    ThisThread::sleep_until(log_time + log_step);
  }
}