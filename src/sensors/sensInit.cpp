/*! \file sensInit.cpp 
    \brief Thread initializing sensors read at the given frequency

    Creates a timer that calls an interrupt with the given frequency
*/

//TODO Check mag bias and calibration!!!! (line 115)

#include <mbed.h>
#include "EventQueue.h"
#include "Event.h"
#include "math.h"

#include "MPU9250.h"
#include "SonarMaxBotix.h"

#include "sensInit.hpp"
#include "global_vars.hpp"
#include "commander.hpp"

// #include "massStorage.hpp"
// #include "magCalibrate.hpp"

#define MPU6050_ADDRESS             0x69
#define MPU9250_FREQ                200ms         // in milliseconds!

// Create imu object 
MPU9250 imu(PA_10,PA_9);

// Create sonar object
SonarMaxBotix sonar(A0); 

// timer (used in read_sensors_eventHandler)
// Timer timerSesnInt;

/*
float roll,pitch,mag_norm;
float magValues[3], magValues_filt[3], minExtremes[3], maxExtremes[3], minMag[3], maxMag[3];
int measurements_count = 0, id_calib;
char f_buff[100], f_buff_disc[100], temp_char;
float mag_extremes[6];*/
float altitude; 

EventQueue queue;
Event<void(void)> read_sensors_event(&queue, read_sensors_eventHandler);


void sensInit()
{
    bool flag_MPU9250_online = false, flag_MPU9250_calibrated = false;
    bool flag_AK8963_online = false, flag_AK8963_calibrated = false; 

    print_lock.lock();
    printf("Start sensors thread ID: %d\n", ThisThread::get_id());
    print_lock.unlock();

    // Accelerometer and gyroscope initialization
    uint8_t whoami = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-925
    if (whoami==0x71)
    {
        print_lock.lock();
        printf("MPU9250 online\n");
        print_lock.unlock();
        flag_MPU9250_online = true; 

        // Calibrate IMU 
        imu.calibrateMPU9250(imu.gyroBias, imu.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        flag_MPU9250_calibrated = true;

        // Initialize accelerometer and gyroscope
        imu.initMPU9250();        
        imu.getAres(); // Get accelerometer sensitivity
        imu.getGres(); // Get gyro sensitivity

        // Initialize quaternion and step time
        imu.q[0] = 1; imu.q[1] = 0; imu.q[2] = 0; imu.q[3] = 0; imu.deltat = 0.01;
        
        // printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<imu.Ascale));
        // printf("Gyroscope full-scale range = %f  g\n\r", 2.0f*(float)(1<<imu.Gscale));
        // printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/imu.aRes);
        // printf("Gyroscope sensitivity is %f LSB/g \n\r", 1.0f/imu.gRes);
    }
    else
    {
        print_lock.lock();
        printf("Cannot reach MPU9250!\n");
        print_lock.unlock();

        flag_MPU9250_online = false;
        flag_MPU9250_calibrated = false;
    }
    
    // Magnetometer initialization
    whoami = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (whoami==0x48) 
    {
        print_lock.lock();
        printf("AK8963 online\n");
        print_lock.unlock();
        flag_AK8963_online = true;    

        imu.initAK8963(imu.magCalibration);
        flag_MPU9250_calibrated = true; // ???????????
        
        imu.getMres(); // Get mag sensitivity
        imu.magbias[0] = +470.;
        imu.magbias[1] = +120.;
        imu.magbias[2] = +125.;
    }
    else
    {
        print_lock.lock();
        printf("Cannot reach AK8963!\n");
        print_lock.unlock();

        flag_AK8963_online = false;
        flag_AK8963_calibrated = false;
    }

    // Launch events if all sensors are online
    if (flag_MPU9250_online && flag_AK8963_online)
    {
        // setup events and post on queue
        postSensorEvent();

        if(flag_MPU9250_calibrated && flag_MPU9250_calibrated){
            // all sensors online and calibrated
            main_commander->arm();
        }

        // start dipatching
        queue.dispatch_forever();

        // reach this point only if queue is stopped
    }
    else
    {
        print_lock.lock();
        printf("Error with sensors\n"); 
        print_lock.unlock();

        if (main_commander->is_armed())
            main_commander->disarm(); 
    }

    print_lock.lock();
    printf("End sensors thread ID: %d\n\r", ThisThread::get_id());
    print_lock.unlock();

    return;
}

void postSensorEvent(void)
{
    // setup event params and post
    read_sensors_event.period(MPU9250_FREQ); 
    read_sensors_event.delay(200ms);
    read_sensors_event.post();
}

// TODO: add semaphore to protect the write-to-buffer operation in the following event!
void read_sensors_eventHandler(void) // Event to copy sensor value from its register to extern variable
{
    // read data from MPU9250 and AK8963
    if(imu.readByte(MPU9250_ADDRESS,INT_STATUS & 0x01))     // if there are new data
    {
        // imu.deltat=timerSesnInt.read();
        imu.readAccelData(imu.accelCount);
        imu.ax = ((float)imu.accelCount[0])*imu.aRes - imu.accelBias[0];
        imu.ay = (float)imu.accelCount[1]*imu.aRes - imu.accelBias[1];
        imu.az = (float)imu.accelCount[2]*imu.aRes - imu.accelBias[2];

        imu.readGyroData(imu.gyroCount);
        imu.gx = (float)imu.gyroCount[0]*imu.gRes - imu.gyroBias[0];
        imu.gy = (float)imu.gyroCount[1]*imu.gRes - imu.gyroBias[1];
        imu.gz = (float)imu.gyroCount[2]*imu.gRes - imu.gyroBias[2];
        
        imu.readMagData(imu.magCount);
        imu.mx = (float)imu.magCount[0]*imu.magCalibration[0]*imu.mRes - imu.magbias[0];
        imu.my = (float)imu.magCount[1]*imu.magCalibration[1]*imu.mRes - imu.magbias[1];
        imu.mz = (float)imu.magCount[2]*imu.magCalibration[2]*imu.mRes - imu.magbias[2];

        // imu.MadgwickQuaternionUpdate(imu.ax,imu.ay,imu.az,imu.gx*PI/180.0f,imu.gy*PI/180.0f,imu.gz*PI/180.0f,imu.mx,imu.my,imu.mz);
        // imu.quat2eul();

        imu.pitch = atan2(imu.ax,sqrt(imu.ay*imu.ay + imu.az*imu.az));
        imu.roll = atan2(-imu.ay,sqrt(imu.ax*imu.ax + imu.az*imu.az));
        imu.yaw = atan2(-imu.my*cos(imu.roll) - imu.mz*sin(imu.roll),imu.mx*cos(imu.pitch) + imu.my*sin(imu.pitch)*sin(imu.roll) - imu.mz*sin(imu.pitch)*cos(imu.roll));


        // how to pass data to other thread?????


        // Print data
        // printf("")
        // printf("roll: %.2f \t pitch: %.2f \t yaw: %.2f \t \n",imu.roll*180./PI,imu.pitch*180./PI, imu.yaw*180/PI);
        // printf("magCount: %e \t%e\t%e\t\n",imu.magCount[0],imu.magCount[1],imu.magCount[2]); 
        // printf("magCalibration: %f\t%f\t%f\t\n",imu.magCalibration[0],imu.magCalibration[1],imu.magCalibration[2]); 
        // printf("mres: %f\n",imu.mRes); 
        // printf("%f\t%f\t%f\t\n",imu.mx,imu.my,imu.mz);
    }

    // read data from sonar
    altitude = sonar.distance_analog();
    printf("Altitude: %.2f\n",altitude);

    // for manual calibration, handled with interrupt (button)
    // irq.rise(calib_irq_handle);
}











//////////////////////////////////// for calibration ///////////////////////////////

/*
// for manual calibration, handled with interrupt (button)
InterruptIn irq(PTA4);
Event<void(void)> calibrationEvent(mbed_event_queue(),calibration);
CalibrateMagneto magCal;
DigitalOut calib_led(LED_GREEN,1), controllerLedSensorThread(LED_BLUE,1);
*/

// Interrupt handler that starts the calibration
// void calib_irq_handle(void)
// {
//     // printf("break queue\n");
//     irq.rise(NULL);
//     queue.break_dispatch();                         // Stop the dispatch of sensor queue while calibrating
//     calibrationEvent.period(MPU9250_FREQ);
//     id_calib = calibrationEvent.post();
// }

// Calibration event
// void calibration(void)
// {
//     // printf("Thread name: %s; Thread id: %d", ThisThread::get_name(), ThisThread::get_id());
//     if(measurements_count == 0)
//     {
//         controllerLedSensorThread = 1;
//         led_lock.lock();
//     }
//     calib_led = 0;
//     measurements_count++;
//     printf("measurm %d\n",measurements_count);
//     accmagValues = accmag.get_values();
//     magValues[0] = accmagValues.mx;
//     magValues[1] = accmagValues.my;
//     magValues[2] = accmagValues.mz;
//     magCal.run(magValues,magValues_filt);
//     // When reached the number of initial points the calibration is complete
//     if (measurements_count == INITIAL_POINTS)
//     {
//         // It means the magnetometer is calibrated, so I raise a flag signaling that
//         measurements_count = 0;
//         SDStorageAccess.start(refreshParamFileSD);
//         led_lock.unlock();
//     }
    







//////////////////////////////////// for SD ///////////////////////////////

/*
// if SD is mounted
EventQueue SDaccessQueue(8096);
FILE *f_calib;
const char* sdcard_access_thread_name = "SDStorageAccess";
Thread SDStorageAccess(osPriorityNormal,16184,nullptr,sdcard_access_thread_name);
*/


// }
// void refreshParamFileSD(void)
// {
//     mbed_event_queue()->cancel(id_calib);
//     printf("Updating the parameters file on the SD card...\n");
//     magCal.getExtremes(minMag, maxMag);
//     float magData_in[6] = {minMag[0], minMag[1], minMag[2], maxMag[0], maxMag[1], maxMag[2]};
//     if(parametersUpdate(magData_in, "Magnetometer extremes [minXYZ; maxXYZ]\n") == MBED_SUCCESS)
//     {
//         printf("Done updating params!\n");
//         calib_led = 1;
//         queue.dispatch();           // Re-dispatch the sensor queue
//         irq.rise(calib_irq_handle); // Re/enable the rise interrupt on the button to avoid multiple calibrations
//         SDStorageAccess.join();
//     }
// }

// FIXME DEAD CODE!!
// void writeOnSD(void)
// {
//     mbed_event_queue()->cancel(id_calib);
//     printf("Writing vals...\n");
//     magCal.getExtremes(minMag, maxMag);
//     fflush(stdout);
//     f_calib = fopen("/fs/calib.txt","a+");
//     printf("%s\n", (!f_calib ? "Fail :(" : "OK"));
//     fflush(stdout);
//     rewind(f_calib);
//     long line_begin = ftell(f_calib); // Beginning of the line
//     while(!feof(f_calib)) // Now writing into the file on the SD card
//     {
//         printf("Getting char... \n");
//         temp_char = fgetc(f_calib);
//         fflush(stdout);
//         if (temp_char == '#' || temp_char == '\t')  // Skip the line
//         {
//             fgets(f_buff_disc,100,f_calib); // Discard the line
//             line_begin = ftell(f_calib);    // Set new beginning of the line
//             printf("Line discarded\n");
//             // memset(f_buff,0,sizeof(f_buff));
//             fflush(stdout);
//         }
//         else // Here I look for the field I'm interested in and I fill it with data
//         {
//             fseek(f_calib,line_begin,SEEK_SET);
//             fgets(f_buff, 100, f_calib);
//             // FIXME Since opening file as a+ allows output oper to file reposition the cursor at the end of the file, I have to do ftell() here and 
//             // a fseek() right before the fprintf which write on the file in the if below! BUT I cannot overwrite things... I can only append! So the
//             // best way is to completely rewrite the params file each time a modification occurs, implementing this function in mass storage.
//             printf(f_buff);
//             printf("qui\n");
//             fflush(stdout);
//             if (!strcmp(f_buff,"Magnetometer extremes [minXYZ; maxXYZ]\n"))
//             {
//                 fprintf(f_calib,"\t%.3e %.3e %.3e\n",minMag[0], minMag[1], minMag[2]);
//                 fprintf(f_calib,"\t%.3e %.3e %.3e\n", maxMag[0], maxMag[1], maxMag[2]);
//                 line_begin = ftell(f_calib);
//                 // fflush(stdout);
//                 printf("done\n");
//                 fflush(stdout);
//                 break;

//             }
//         }
//     }
//     fclose(f_calib); // Important!
//     calib_led = 1;
//     queue.dispatch();           // Re-dispatch the sensor queue
// }

