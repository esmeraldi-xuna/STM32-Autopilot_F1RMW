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
      return;
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
        return;
      }
    }
  }

  // init OK

  // search highest value of log file

  // open folder
  DIR *d = opendir("sd/");
  if (!d) {
      printf("error open dir\n");
  }

  // read all filename
  while (true) {
    struct dirent *e = readdir(d);
    if (!e) {
        break;
    }
    // printf("    %s\n", e->d_name);

    // get filename number
    sscanf(e->d_name, "log%d.txt", &tmp);

    // save max value
    if (tmp > max)
      max = tmp;
  }

  // close folder
  int err = closedir(d);
  if (err < 0) {
      printf("error close dir\n");
  }

  // add a new file

  // setup new filename
  sprintf(name, "sd/log%d.txt", max+1);
  //printf("%s\n", name);

  // create file
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
  printf("Start LOG\n"); //_SD thread ID: %d\n", (int)ThisThread::get_id());
  print_lock.unlock();

  // open log file
  FILE* log_file = fopen(name, "w+");
  if (!log_file) {
    print_lock.lock();
    printf("Error opening log file\n");
    print_lock.unlock();
    return;
  }

  // write first line for csv file
  fprintf(log_file, "TIME (ms), ALTITUDE, AX, AY, AZ, GX, GY, GZ, MX, MY, MZ, ROOL, PITCH, YAW, CTRL_U, CRTL_Y, APF_U, APF_Y, EKF_U, EKF_Y, PWM_1, PWM_2, PWM_3, PWM_4");

  // start loop for writing file
  Kernel::Clock::time_point start_log = Kernel::Clock::now();
  while (1)
  {
    // get actual time
    log_time = Kernel::Clock::now();
    
    // wrinte time on file
    fprintf(log_file, "%lld,", (log_time - start_log).count());
    
    // write other fields
    // global_data->write_on_SD(log_file);
    global_data->write_on_SD_as_csv(log_file);
    
    // wait untill next step
    ThisThread::sleep_until(log_time + log_step);
  }
}

