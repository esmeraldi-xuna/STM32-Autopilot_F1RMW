/*! \file sensInit.cpp 
    \brief Thread initializing sensors read at the given frequency

    Creates a timer that calls an interrupt with the given frequency
*/

#include <mbed.h>
#include "EventQueue.h"
#include "Event.h"
#include "I2C.h"
#include "math.h"

#include "sensors.hpp"
#include "global_vars.hpp"
#include "commander.hpp"

#include "magCalibrate.hpp"

#include "FXOS8700CQ.h"
#include "ADXL345_I2C.h"
#include "ITG3200.h"
#include "rtos.h"

#include "ManualSwitch.hpp"
#include "RotaryEncoder.h"
#define SENS_FREQ                100ms         // in milliseconds!
#define CALIBRATION_STEPS_NEEDED  100

I2C i2c(PTE25,PTE24);

FXOS8700CQ accmag(PTE25,PTE24); 
ADXL345_I2C acc_ext(PTE25,PTE24); 
ITG3200 gyro_ext(PTE25,PTE24,0x68); 


CalibrateMagneto magCal; // calibration object for magnetometer
ManualSwitch switchEnc(PTD3);
Encoder encoderL(PTB18, PTB19, true);
Encoder encoderR(PTC1, PTC8, true);
DigitalOut ledPin(PTD2);


EventQueue queue;
Event<void(void)> read_sensors_event(&queue, read_sensors_eventHandler);
//Event<void(void)> calibrationEvent(mbed_event_queue(),calibration);

bool flag_FXOS8700CQ_online = false, flag_FXOS8700CQ_calibrated = false;
bool flag_ADXL345_online = false, flag_ADXL345_calibrated = false; 
bool flag_ITG3200_online = false;

void sensors()
{
    /*
    print_lock.lock();
    printf("Start sensors thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();
    */
    
    // scan channel if needed to ensure all sensors are detected
    accmag.init();
    ThisThread::sleep_for(500ms);
    I2C_scan();
    //FX
    
    //if(accmag.get_who_am_i()==0x0D ){
        flag_FXOS8700CQ_online=true;
        main_commander->set_flag_FXOS8700CQ_online(true);
        Data v = accmag.get_values();
        printf("%.2f,%.2f,%.2f\n",v.ax,v.ay,v.az);
    //}
    ThisThread::sleep_for(50ms);
acc_ext.setPowerControl(0x00);//standbymode
        acc_ext.setDataFormatControl(0x0B);//Full resolution, +/-16g, 4mg/LSB.
        acc_ext.setDataRate(ADXL345_3200HZ);//3.2kHz data rate.
        acc_ext.setPowerControl(0x08); //Measurement mode.
        flag_ADXL345_online=true;
        main_commander->set_flag_ADXL345_online(true);
        main_commander->set_flag_ITG3200_online(true);
        main_commander->set_flag_ADXL345_calibrated(true);//OCCHIO CALIB! FORZATA PER PASSARE CONTROLLI
        main_commander->set_flag_FXOS8700CQ_calibrated(true);
        if(main_commander->get_flag_ADXL345_online()) printf("Correctly put online adxl\n");
    //adxl+gyro
    /* if(acc_ext.getDeviceID()==0x53){
        flag_ADXL345_online=true;
        main_commander->set_flag_ADXL345_online(true);
        acc_ext.setPowerControl(0x00);//standbymode
        acc_ext.setDataFormatControl(0x0B);//Full resolution, +/-16g, 4mg/LSB.
        acc_ext.setDataRate(ADXL345_3200HZ);//3.2kHz data rate.
        acc_ext.setPowerControl(0x08); //Measurement mode.
    }
    ThisThread::sleep_for(500ms); */


    // Launch events if all sensors are online
    if (flag_FXOS8700CQ_online)//CONTROLLO SOLO QUESTO PER ORA
    {
        main_commander->set_flag_comm_joystick(true); //lo forzo a true per superare checkinit
        // setup events and post on queue
        postSensorEvent();

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
    read_sensors_event.period(SENS_FREQ); 
    read_sensors_event.delay(200ms);
    read_sensors_event.post();

    /* update_quaternions.period(3ms);
    update_quaternions.delay(3ms);
    update_quaternions.post(); */
}

// Event to copy sensor value from its register to extern variable
void read_sensors_eventHandler(void) 
{
    
    float mag_values[3],mag_values_filt[3], magnorm;
    float accel_resol = 256;
    float gyro_resol = 14.375; 
    struct_sensors_data all_data;
    Data accmag_v; 
    int16_t readings[3];
    int32_t val;
    int32_t posL, posR;
    accel_resol=accel_resol; gyro_resol=gyro_resol; readings[0]=0;
    val = posL = posR = 0;
    val++;

    //TO DO: MODIFICARE SEGNI
    accmag_v = accmag.get_values();
    all_data.a.x = accmag_v.ax;
    all_data.a.y = accmag_v.ay;
    all_data.a.z = accmag_v.az;
    mag_values[0] = accmag_v.mx;
    mag_values[1] = accmag_v.my;
    mag_values[2] = accmag_v.mz;
    magCal.run(mag_values,mag_values_filt);
    magnorm = sqrt(accmag_v.mx*accmag_v.mx + accmag_v.my*accmag_v.my + accmag_v.mz*accmag_v.mz);
    all_data.m.x = accmag_v.mx/magnorm;
    all_data.m.y = accmag_v.my/magnorm;
    all_data.m.z = accmag_v.mz/magnorm;

    acc_ext.getOutput(readings);
    all_data.a_ext.x = (float)readings[1]/accel_resol;
    all_data.a_ext.y = -(float)readings[0]/accel_resol;
    all_data.a_ext.z = (float)readings[2]/accel_resol;
    all_data.g_ext.x = gyro_ext.getGyroY()/gyro_resol;
    all_data.g_ext.y = -gyro_ext.getGyroX()/gyro_resol;
    all_data.g_ext.z = gyro_ext.getGyroZ()/gyro_resol;
   

    val = switchEnc.getState();
    if (val == 1) {ledPin = 1;}
    
    else {
        ledPin = 0;     
        encoderL.Reset(true);   // True is for resetting the encoder timer too
        encoderR.Reset(true);   // True is for resetting the encoder timer too
    }

    all_data.posL = -encoderL.getPosition()*360/(1920);
    all_data.posR = encoderR.getPosition()*360/(1920);
    
    all_data.speedL = encoderL.getSpeed()*60; // rpm
    all_data.speedR = encoderR.getSpeed()*60;
    global_data->write_sensor(all_data);
    return;
}



int I2C_scan(void){
    
    int count = 0;

    print_lock.lock();
	
    // check all adresses
	for (int address = 0; address < 127; address++)
	{
		if (!i2c.write(address << 1, NULL, 0)) // 0 returned is ok
		{																								  
			printf("I2C device found at address 0x%02X (0x%02X in 8-bit)\n", address, address << 1);
			count++;
		}
        ThisThread::sleep_for(50ms);
	}
	if (count)
		printf("%d", count);
	else
		printf("No");
	printf(" device found\n\n");


    print_lock.unlock(); 

    return count;
}