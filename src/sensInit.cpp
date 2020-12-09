/*! \file sensInit.cpp 
    \brief Thread initializing sensors read at the given frequency

    Creates a timer that calls an interrupt with the given frequency
*/

//TODO Check mag bias and calibration!!!! (line 115)

#include <mbed.h>
#include "MPU9250.h"
#include "BMP085.h"
#include "global_vars.hpp"
// #include "massStorage.hpp"

#include "sensInit.hpp"
#include "EventQueue.h"
#include "Event.h"
#include "math.h"
// #include "magCalibrate.hpp"
#include <ThisThread.h>
#include <Thread.h>
#include <rtos.h>

#define MPU6050_ADDRESS             0x69
#define MPU9250_FREQ                200         // in milliseconds!

// using namespace events;
// using namespace rtos;
// using namespace ThisThread;
// using namespace mbed;

// Create imu object 
MPU9250 imu(PA_10,PA_9);

// Create baro object
BMP085 barometer(PA_10,PA_9);

// CalibrateMagneto magCal;
// DigitalOut calib_led(LED_GREEN,1), controllerLedSensorThread(LED_BLUE,1);

// FILE *f_calib;

float roll,pitch,mag_norm;
float magValues[3], magValues_filt[3], minExtremes[3], maxExtremes[3], minMag[3], maxMag[3];
int measurements_count = 0, id_calib;
char f_buff[100], f_buff_disc[100], temp_char;
float mag_extremes[6];
float pressure = 0.0f, temperature = 0.0f;

EventQueue queue;
// EventQueue SDaccessQueue(8096);
Event<void(void)> accmagreadEvent(&queue,AccMagRead);
// Event<void(void)> calibrationEvent(mbed_event_queue(),calibration);

// const char* sdcard_access_thread_name = "SDStorageAccess";
// Thread SDStorageAccess(osPriorityNormal,16184,nullptr,sdcard_access_thread_name);

// InterruptIn irq(PTA4);
Timer timerSesnInt;
Thread SensorRead(osPriorityNormal,8092,nullptr,"sensRead");

void sensInit()
{
    bool flag = false; 

    // Accelerometer and gyroscope initialization
    uint8_t whoami = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-925
    if (whoami==0x71)
    {
        printf("MPU9250 online\n\r");
        flag = true; 

        // Calibrate IMU 
        imu.calibrateMPU9250(imu.gyroBias, imu.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        // Initialize accelerometer and gyroscope
        imu.initMPU9250();        
        imu.getAres(); // Get accelerometer sensitivity
        imu.getGres(); // Get gyro sensitivity
        imu.getMres(); // Get mag sensitivity

        // Initialize quaternion and step time
        imu.q[0] = 1; imu.q[1] = 0; imu.q[2] = 0; imu.q[3] = 0; imu.deltat = 0.01;
        
        // printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<imu.Ascale));
        // printf("Gyroscope full-scale range = %f  g\n\r", 2.0f*(float)(1<<imu.Gscale));
        // printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/imu.aRes);
        // printf("Gyroscope sensitivity is %f LSB/g \n\r", 1.0f/imu.gRes);
    }
    else
    {
        printf("Cannot reach MPU9250!\n\r");
    }
    
    // Magnetometer initialization
    whoami = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (whoami==0x48) 
    {
        printf("AK8963 online\n\r");
        flag = flag && true;  
        // Open file with params and get them...
        // if (readFromSD(mag_extremes, "Magnetometer extremes [minXYZ; maxXYZ]\n") < 0)
        // {
        //     // MBED_WARNING(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_FAILED_OPERATION),"SD card has no calib.txt file to open! Please format the SD card and calibrate\n");
        //     printf("SD card has no calib.txt file to open! Please format the SD card and calibrate\n");
        // }
        // else
        // {
        //     minExtremes[0] = mag_extremes[0];
        //     minExtremes[1] = mag_extremes[1];
        //     minExtremes[2] = mag_extremes[2];
        //     maxExtremes[0] = mag_extremes[3];
        //     maxExtremes[1] = mag_extremes[4];
        //     maxExtremes[2] = mag_extremes[5];
        //     magCal.setExtremes(minExtremes,maxExtremes);
        //     for (int ii = 0; ii < 6; ii++)
        //     {
        //         printf("data out mag extr %f\n", mag_extremes[ii]);
        //     }
            
        // }
        imu.initAK8963(imu.magCalibration);
        imu.magbias[0] = +470.; imu.magbias[1] = +120.; imu.magbias[2] = +125.;
    }
    else
    {
        flag = flag && false;
        printf("Cannot reach AK8963!\n\r");
    }
    printf("\033[2J");

    // Barometer initialization


    // Launch event if MPU9250 and AK8963 are ok
    if (flag)
    {
        SensorRead.start(postSensorEvent);
        queue.dispatch();
    }
    else
    {
        printf("Error with MPU9250 sensor\n\r"); 
        return; 
    }
    // SDaccessQueue.dispatch();
}

void postSensorEvent(void)
{
    // Write here the sensor read events to post them into the queue!
    accmagreadEvent.period(MPU9250_FREQ); 
    accmagreadEvent.delay(200);
    accmagreadEvent.post();
    // queue.call_every(200,AccMagRead);
}

// TODO: add semaphore to protect the write-to-buffer operation in the following event!
void AccMagRead(void) // Event to copy sensor value from its register to extern variable
{
    // accmagValues = accmag.get_values();
    // magValues[0] = accmagValues.mx;
    // magValues[1] = accmagValues.my;
    // magValues[2] = accmagValues.mz;
    // magCal.run(magValues,magValues_filt);
    // // mag_norm=0.00001;
    // mag_norm = sqrt(accmagValues.mx*accmagValues.mx + accmagValues.my*accmagValues.my + accmagValues.mz*accmagValues.mz);
    // accmagValues.mx = accmagValues.mx/mag_norm;
    // accmagValues.my = accmagValues.my/mag_norm;
    // accmagValues.mz = accmagValues.mz/mag_norm;
    // pitch = atan2(imu.ax,sqrt(imu.ay*imu.ay + imu.az*imu.az));
    // roll = atan2(-imu.ay,sqrt(imu.ax*imu.ax + imu.az*imu.az));
    // // feedback_control_U.psi_est = atan2(-accmagValues.my*cos(roll) - accmagValues.mz*sin(roll),accmagValues.mx*cos(pitch) \
    //                             + accmagValues.my*sin(pitch)*sin(roll) - accmagValues.mz*sin(pitch)*cos(roll))*180/3.14;
    // PI_contr_U.psi_odom = atan2(magValues_filt[1],magValues_filt[0])*180/3.14;
    // printf("yaw: %f\n",feedback_control_U.psi_est);
    // printf("ax: %.2f ay: %.2f az: %.2f pitch: %.2f roll: %.2f yaw: %.2f mx: %.2f my: %.2f mz: %.2f\n", \ 
            // accmagValues.ax, accmagValues.ay, accmagValues.az, pitch*180/3.14, roll*180/3.14, feedback_control_U.psi_est, accmagValues.mx, accmagValues.my, accmagValues.mz);
    // feedback_control_U.reference = (accmagValues.ax + 1)/2; // Normalized between 0 and 1
    // feedback_control_U.estimated = 0;//servo1.read();
    // printf("\033[2;1H");
    // printf("acc read: %f servo read: %f\n", feedback_control_U.reference,feedback_control_U.estimated);
    // printf("%f\n", accmagValues.ax);

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
        // imu.yaw = 

        barometer.update();
        pressure = barometer.get_pressure();
        temperature = barometer.get_temperature();
        printf("Pressure: %.2f [hPa] -  Temperature: %.2f [C] - Altitude %.2f [m]\n", pressure, temperature);

        // Print data
        // printf("roll: %.2f\tpitch: %.2f\t\n",imu.roll*180./PI,imu.pitch*180./PI);
    }

    // irq.rise(calib_irq_handle);
}

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

