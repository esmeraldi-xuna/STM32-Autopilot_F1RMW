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

#include "sensors.hpp"
#include "global_vars.hpp"
#include "commander.hpp"

// #include "massStorage.hpp"
// #include "magCalibrate.hpp"

// #define MPU6050_ADDRESS             0x69
#define MPU9250_FREQ                200ms         // in milliseconds!

// Create imu object 
MPU9250 imu(I2C_SDA, I2C_SCL);

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


void sensors()
{
    bool flag_MPU9250_online = false, flag_MPU9250_calibrated = false;
    bool flag_AK8963_online = false, flag_AK8963_calibrated = false; 

    print_lock.lock();
    printf("Start sensors thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();

    // Accelerometer and gyroscope initialization
    uint8_t whoami = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-925
          
    print_lock.lock();
    printf("whami 1: 0x%x\n", whoami);
    printf("MPU9250 online\n");
    print_lock.unlock();
    flag_MPU9250_online = true; 
   
    // Magnetometer initialization
    whoami = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    
    print_lock.lock();
    printf("whami 2: 0x%x\n", whoami);
    printf("AK8963 online\n");
    print_lock.unlock();
    flag_AK8963_online = true;    

    imu.initAll();

    imu.getAres(); // Get accelerometer sensitivity
    imu.getGres(); // Get gyro sensitivity
    imu.getMres(); // Get magnetometer sensitivity
    imu.magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    imu.magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    imu.magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
 

    ////////////////////////////////////// for debug///////////////////////////////////////////////
    
    flag_MPU9250_online = false;
    flag_AK8963_online = false;

    flag_AK8963_calibrated = false;
    flag_MPU9250_calibrated = false;

    ///////////////////////////////////////////////////////////////////////////////////////////////

    // Launch events if all sensors are online
    if (flag_MPU9250_online && flag_AK8963_online)
    {
        // setup events and post on queue
        postSensorEvent();

        if(flag_AK8963_calibrated && flag_MPU9250_calibrated){
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
    printf("End sensors thread ID: %d\n", (int)ThisThread::get_id());
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

// Event to copy sensor value from its register to extern variable
void read_sensors_eventHandler(void) 
{
    struct_sensors_data data_in;

    // read data from MPU9250 and AK8963
    if(imu.readByte(MPU9250_ADDRESS,INT_STATUS & 0x01))     // if there are new data
    {
        // imu.deltat=timerSesnInt.read();
        imu.readAccelData(imu.accelCount);
        data_in.ax = imu.ax = ((float)imu.accelCount[0])*imu.aRes - imu.accelBias[0];
        data_in.ay = imu.ay = (float)imu.accelCount[1]*imu.aRes - imu.accelBias[1];
        data_in.az = imu.az = (float)imu.accelCount[2]*imu.aRes - imu.accelBias[2];

        imu.readGyroData(imu.gyroCount);
        data_in.gx = imu.gx = (float)imu.gyroCount[0]*imu.gRes - imu.gyroBias[0];
        data_in.gy = imu.gy = (float)imu.gyroCount[1]*imu.gRes - imu.gyroBias[1];
        data_in.gz = imu.gz = (float)imu.gyroCount[2]*imu.gRes - imu.gyroBias[2];
        
        imu.readMagData(imu.magCount);
        data_in.mx = imu.mx = (float)imu.magCount[0]*imu.magCalibration[0]*imu.mRes - imu.magbias[0];
        data_in.my = imu.my = (float)imu.magCount[1]*imu.magCalibration[1]*imu.mRes - imu.magbias[1];
        data_in.mz = imu.mz = (float)imu.magCount[2]*imu.magCalibration[2]*imu.mRes - imu.magbias[2];

        // imu.MadgwickQuaternionUpdate(imu.ax,imu.ay,imu.az,imu.gx*PI/180.0f,imu.gy*PI/180.0f,imu.gz*PI/180.0f,imu.mx,imu.my,imu.mz);
        // imu.quat2eul();

        data_in.pitch = imu.pitch = atan2(imu.ax,sqrt(imu.ay*imu.ay + imu.az*imu.az));
        data_in.roll = imu.roll = atan2(-imu.ay,sqrt(imu.ax*imu.ax + imu.az*imu.az));
        data_in.yaw = imu.yaw = atan2(-imu.my*cos(imu.roll) - imu.mz*sin(imu.roll),imu.mx*cos(imu.pitch) + imu.my*sin(imu.pitch)*sin(imu.roll) - imu.mz*sin(imu.pitch)*cos(imu.roll));

        // Print data
        // printf("")
        // printf("roll: %.2f \t pitch: %.2f \t yaw: %.2f \t \n",data_in.roll*180./PI,data_in.pitch*180./PI, data_in.yaw*180/PI);
        // printf("magCount: %e \t%e\t%e\t\n",imu.magCount[0],imu.magCount[1],imu.magCount[2]); 
        // printf("magCalibration: %f\t%f\t%f\t\n",imu.magCalibration[0],imu.magCalibration[1],imu.magCalibration[2]); 
        // printf("mres: %f\n",imu.mRes); 
        // printf("%f\t%f\t%f\t\n",imu.ax,imu.ay,imu.az);
    }

    // read data from sonar
    data_in.altitude = altitude = sonar.distance_analog();
    
    // write data when all available
    global_data->write_sensor(data_in);
    
    //printf("Altitude: %.2f\n",altitude);
}
