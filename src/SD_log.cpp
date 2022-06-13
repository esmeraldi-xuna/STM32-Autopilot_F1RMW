#include <mbed.h>
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include "SD_log.hpp"
#include "global_vars.hpp"

#define FORCE_REFORMAT 0

// define pin for SD (change values on 'mbed_app.json')
//TO DO: vedi conf.
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
  //remove("sd/log105.txt");
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
/*
int getCalibValues(void){
    FILE *fc=fopen("sd/calib.txt","r");
    if(fc <= 0 ) {
      printf("No calib\n"); return -1;
    }
    float data_in_min[3], data_in_max[3];
    fscanf(fc,"Mag values minXYZ, maxXYZ: %f,%f,%f,%f,%f,%f\n",&data_in_min[0],
      &data_in_min[1],&data_in_min[2],&data_in_max[0],&data_in_max[1],&data_in_max[2]);
    fclose(fc);
    magCal.setExtremes(data_in_min,data_in_max);
    printf("Mag values minXYZ, maxXYZ: %f,%f,%f,%f,%f,%f\n",data_in[0],
      data_in[1],data_in[2],data_in[3],data_in[4],data_in[5]);
      return 1;
}
*/
// Deletes and rewrites the file in which parameters are stored updating their values e.g. after a calibration
void calibUpdate(float *data_in)
{ 
    remove("sd/calib.txt");
    FILE *fc=fopen("sd/calib.txt","w");
    printf("WritingMag values minXYZ, maxXYZ: %f,%f,%f,%f,%f,%f\n",data_in[0],
      data_in[1],data_in[2],data_in[3],data_in[4],data_in[5]);
    fprintf(fc,"Mag values minXYZ, maxXYZ: %f,%f,%f,%f,%f,%f\n",data_in[0],
      data_in[1],data_in[2],data_in[3],data_in[4],data_in[5]);
    fclose(fc);

   
   /*  ifstream calibration_file("/fs/calib.txt");
    std::string line;//, data_in_str;//, temp_buffer;
    // Make a copy of the original calib file into a string buffer that'll be modified

    std::stringstream buffer;
    buffer << calibration_file.rdbuf();

    std::string temp_buffer(buffer.str()); */

    // printf("Content of the file:\n");
    // std::string texteditor(buffer.str());
    // printf(texteditor.c_str());
    // printf("\n\nEnd content of the file.\n\n");
    
    // Here I find which is entry I'm interested in and I replace the values

    /*
    Magnetometer calibration values.
    */
/* 
    if(!strcmp(field_name, "Magnetometer extremes [minXYZ; maxXYZ]\n"))
    {
        // Checking if the required field exists
        size_t posMag = temp_buffer.find(field_name);
        if(posMag == string::npos)
        {
            printf("Required field doesn't exist. Try formatting.\n");
            return -1;
        }
        // Converting floats to strings
        std::ostringstream ss;
        for(int kk = 0; kk < 6; kk++)
        {
            // printf("qui %d", kk);
            ss << data_in[kk];
            ss << " ";
        }
        std::string data_in_str(ss.str()); // String in which num values are put
        data_in_str.insert(data_in_str.begin(),'\t');
        data_in_str.append("\n");
        // Overwriting file with new values
        size_t posNewline = temp_buffer.find("\n", posMag + sizeof("Magnetometer extremes [minXYZ; maxXYZ]\n"));
        if(posNewline == string::npos)
        {
            temp_buffer.insert(temp_buffer.end(), ' ');
            temp_buffer.replace(temp_buffer.end()-1, temp_buffer.end(), data_in_str);
        }
        else
        {
            // temp_buffer.replace(posMag + sizeof("Magnetometer extremes [minXYZ; maxXYZ]\n"), \
            //     posNewline - (posMag + sizeof("Magnetometer extremes [minXYZ; maxXYZ]\n")), " ");
            // temp_buffer.replace(posMag + sizeof("Magnetometer extremes [minXYZ; maxXYZ]\n") - 1, sizeof(data_in_str), data_in_str);
            temp_buffer.replace(posMag + sizeof("Magnetometer extremes [minXYZ; maxXYZ]\n") - 1, \
                    posNewline + 1 - (posMag + sizeof("Magnetometer extremes [minXYZ; maxXYZ]\n") - 1), data_in_str); // add +1 at posnewline!
        }
        printf("Content of the file:\n");
        printf(temp_buffer.c_str());
        printf("END\n");
    }
    calibration_file.close();

    // Create new empty calib.txt in which to copy the updated file temp_buff
    ofstream updated_calibration_file("/fs/calib.txt");
    updated_calibration_file << temp_buffer;
    updated_calibration_file.close();
    return MBED_SUCCESS; */
}

