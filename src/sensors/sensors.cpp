/*! \file sensInit.cpp 
    \brief Thread initializing sensors read at the given frequency

    Creates a timer that calls an interrupt with the given frequency
*/

#include <mbed.h>
#include "EventQueue.h"
#include "Event.h"
#include "I2C.h"
#include "math.h"

#include "MPU9250.h"
#include "SonarMaxBotix.h"

#include "sensors.hpp"
#include "global_vars.hpp"
#include "commander.hpp"

#include "BMP180.h"
#include "magCalibrate.hpp"

#define MPU9250_FREQ                200ms         // in milliseconds!
#define CALIBRATION_STEPS_NEEDED  100


// I2C channel
I2C* i2c_ch = new I2C(I2C_SDA, I2C_SCL); //D4, D5

// Create imu object 
MPU9250 imu;

// calibration object for magnetometer
CalibrateMagneto magCal;

// Create sensor BMP180 obj
BMP180 sens_bmp(i2c_ch);

// Create sonar object
// SonarMaxBotix sonar(A0);

EventQueue queue;
Event<void(void)> read_sensors_event(&queue, read_sensors_eventHandler);

bool flag_MPU9250_online = false, flag_MPU9250_calibrated = false;
bool flag_AK8963_online = false, flag_AK8963_calibrated = false; 
bool flag_BMP180_online = false;

void sensors()
{
    print_lock.lock();
    printf("Start sensors thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();
    
    // scan channel if needed to ensure all sensors are detected
    // I2C_scan();

    //////////////////////////////////// MPU9250 //////////////////////////////////////////////

    flag_MPU9250_online = init_accel_gyro();

    if(flag_MPU9250_online){
        main_commander->all_flags.flag_MPU9250_online = true;
    }

    ThisThread::sleep_for(500ms);

    flag_AK8963_online = init_magn();

    if(flag_AK8963_online){
        main_commander->all_flags.flag_AK8963_online = true;
    }

    ThisThread::sleep_for(500ms);

    ///////////////////////////////// BMP180 //////////////////////////////////////////////

    flag_BMP180_online = init_baro();
    if(flag_BMP180_online){
        main_commander->all_flags.flag_BMP180_online = true;
    }
   
    ////////////////////////////////////// for debug///////////////////////////////////////////////
/*   
    flag_MPU9250_online = true;
    flag_AK8963_online = true;
    flag_BMP180_online = true;


    flag_AK8963_calibrated = false;
    flag_MPU9250_calibrated = false;
*/
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // Launch events if all sensors are online
    if (flag_MPU9250_online && flag_AK8963_online && flag_BMP180_online)
    {
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
    read_sensors_event.period(MPU9250_FREQ + 100ms); 
    read_sensors_event.delay(200ms);
    read_sensors_event.post();
}

// Event to copy sensor value from its register to extern variable
void read_sensors_eventHandler(void) 
{
    struct_sensors_data all_data;
    float temp, tmp[3], mag_norm;
    int pressure;

    int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
    int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
    float magValues[3];     // Stores mag values after filtering

    /////////////////////////// BMP180 ///////////////////////////////////
    
    // reading temperature
    sens_bmp.startTemperature();
    ThisThread::sleep_for(5ms);     // Wait for conversion to complete
    
    if(sens_bmp.getTemperature(&temp) != 0) {
        print_lock.lock();
        printf("Error getting temperature\n");
        print_lock.unlock();
    }
    // printf("Temperature = %f C\n", temp);


    // reading pressure
    sens_bmp.startPressure(BMP180::ULTRA_HIGH_RESOLUTION);
    ThisThread::sleep_for(80ms);    // Wait for conversion to complete (75 = MAX value, OK for all precision, see datasheet)

    if(sens_bmp.getPressure(&pressure) != 0) {
        print_lock.lock();
        printf("Error getting pressure\n");
        print_lock.unlock();
    }
 
    // printf("Pressure = %d Pa\n", pressure);

    // conversion to altitude: alt = (R * T * ln(P0/P)) / g
    pressure = pressure / 100; // hPa
    float p0 = 1000; // sea level pressure (hPa)
    all_data.altitude = (8314/9.81 * temp * log(p0/pressure)); // meters
    printf("Altitude = %f m\n", all_data.altitude);

    /////////////////////////// mpu9250 ///////////////////////////////////

    if(imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) { // if data available
        
        imu.readAccelData(accelCount);  // Read the x/y/z adc values   
        // Calculate the accleration value into actual g's
        all_data.ax = ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        all_data.ay = ay = (float)accelCount[1]*aRes - accelBias[1];   
        all_data.az = az = (float)accelCount[2]*aRes - accelBias[2];  
    
        imu.readGyroData(gyroCount);  // Read the x/y/z adc values
        // Calculate the gyro value into actual degrees per second
        all_data.gx = gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
        all_data.gy = gy = (float)gyroCount[1]*gRes - gyroBias[1];  
        all_data.gz = gz = (float)gyroCount[2]*gRes - gyroBias[2];   
    
        imu.readMagData(magCount);  // Read the x/y/z adc values  
        // use magCalibrate class to get correct value
        tmp[0]=(float)magCount[0];
        tmp[1]=(float)magCount[1];
        tmp[2]=(float)magCount[2];

        if(flag_AK8963_calibrated){
            magCal.run(tmp, magValues);
        }

        mx = magValues[0];
        my = magValues[1];
        mz = magValues[2];

        // normalize mag values
        mag_norm = sqrt(mx*mx + my*my + mz*mz);
        all_data.mx = mx = mx/mag_norm;
        all_data.my = my = my/mag_norm;
        all_data.mz = mz = mz/mag_norm;
    }

    // calculate roll, pitch, yaw from sensor data
    roll = atan2(-ay,sqrt(ax*ax + az*az));
    pitch = atan2(ax,sqrt(ay*ay + az*az));
    yaw = atan2(-my*cos(roll) - mz*sin(roll),mx*cos(pitch) + my*sin(pitch)*sin(roll) - mz*sin(pitch)*cos(roll));

    // converting to degrees, add declination factor 
    roll  *= 180.0f / PI;
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   -= 2.62f; // Declination at Torino, Italy is 2 degrees 37 minutes

/*
    // calculate roll, pitch, yaw through quaternions (to review, not working)
    // Pass gyro rate as rad/s
    imu.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
    // imu.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
 

    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    
    // converting to degrees, add declinations factor to yaw
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 2.62f; // Declination at Torino, Italy is 2 degrees 37 minutes
    // yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
*/
    all_data.roll = roll;
    all_data.pitch = pitch;
    all_data.yaw = yaw;


    /////////////////////////// print debug ///////////////////////////////////
/*
    printf("ax = %f", 1000*ax); 
    printf(" ay = %f", 1000*ay); 
    printf(" az = %f g;  ", 1000*az); 

    printf("gx = %f", gx); 
    printf(" gy = %f", gy); 
    printf(" gz = %f  deg/s;  ", gz); 
    
    printf("mx = %f", mx); 
    printf(" my = %f", my); 
    printf(" mz = %f  mG\n", mz); 

    printf("Roll, Pitch, Yaw:\t\t%.3f,\t\t%.3f,\t\t%.3f\n", roll, pitch, yaw);
*/
    /////////////////////////////////////////////////////////////////////////////
/*
    // read data from sonar
    data_in.altitude = sonar.distance_analog();
    printf("Altitude: %.2f\n",altitude);
*/  
    // write data when all available
    global_data->write_sensor(all_data);
}

int I2C_scan(void){
    
    int count = 0;

    print_lock.lock();
	
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

bool init_accel_gyro(){

    uint8_t whoami = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
          
    print_lock.lock();
    
    // printf("whami 1: 0x%x\n", whoami);
    if(whoami == 0x71){
        printf("MPU9250 online\n");
        print_lock.unlock();
    }else{
        printf("Error communicating with MPU9250\n");
        print_lock.unlock();
        return false;
    }
    
    imu.resetMPU9250(); // Reset registers to default in preparation for device calibration

    // DO NOT SWITCH calibrate and init, magnetometer AK8963 will not be detected

    imu.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    flag_MPU9250_calibrated = true;

    imu.initMPU9250(); 

    imu.getAres(); // Get accelerometer sensitivity
    imu.getGres(); // Get gyro sensitivity

    return true;
}

bool init_magn(){
    
    float min_mag_extr[3], max_mag_extr[3];

    uint8_t whoami = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    
    print_lock.lock();
    
    // printf("whami 2: 0x%x\n", whoami);
    if(whoami == 0x48){
        printf("AK8963 online\n");
        print_lock.unlock();
    }else{
        printf("Error communicating with AK8963\n");
        print_lock.unlock();
        return false;
    }
    
    imu.initAK8963(magCalibration);
        
    imu.getMres(); // Get magnetometer sensitivity

    
    #if SD_MOUNTED // try get calibration value from SD

    if (try_get_calibration_values(min_mag_extr, max_mag_extr)){
        magCal.setExtremes(min_mag_extr, max_mag_extr);
        flag_AK8963_calibrated = true;
        main_commander->all_flags.flag_AK8963_calibrated = true;
        return true;
    }else{
        flag_AK8963_calibrated = false;
    }

    #else // no SD, do calibration

    flag_AK8963_calibrated = false;

    #endif

    return true;
}

bool init_baro(){
    
    print_lock.lock();
    if (sens_bmp.init() != 0) {
        printf("Error communicating with BMP180\n");
        print_lock.unlock();
        return false;
    } else {
        printf("BMP180 online\n");
        print_lock.unlock();
    }
    return true;
}

void mag_calibration(void){

    float min_mag_extr[3], max_mag_extr[3];

    print_lock.lock();
    printf("Strarting calibration!!\n");
    
    for (int i=0; i<CALIBRATION_STEPS_NEEDED; i++){
        do_calibration_step();
        ThisThread::sleep_for(MPU9250_FREQ);
    }
    
    printf("Calibration done!!\n");
    print_lock.unlock();

    flag_AK8963_calibrated = true;
    main_commander->all_flags.flag_AK8963_calibrated = true;

    magCal.getExtremes(min_mag_extr, max_mag_extr);

    #if SD_MOUNTED // save calibration data on SD

        save_calib_values(min_mag_extr, max_mag_extr);

    #endif
    

}

void do_calibration_step(){

    int16_t magValues[3];
    float tmp[3]; // output not used during calibration

    imu.readMagData(magValues);
    tmp[0]=(float)magValues[0];
    tmp[1]=(float)magValues[1];
    tmp[2]=(float)magValues[2];

    // printf("Read: %f, %f, %f\n", tmp[0], tmp[1], tmp[2]); 

    magCal.run(tmp, tmp);

    return;
}

bool try_get_calibration_values(float* min_ext, float* max_ext){
    
    FILE* f = fopen("/sd/calibration_values.txt", "r");
    char buffer[100];

    if (!f) {
        print_lock.lock();
        printf("Error opening calibration file\n");
        print_lock.unlock();
        fclose(f);
        return false;

        f = fopen("/sd/calibration_values.txt", "w+");
        if (!f) {
            print_lock.lock();
            printf("Error creating calibration file\n");
            print_lock.unlock();
        }

    }else{
        if (fgets(buffer, 100, f) == NULL){ // read first line, do nothing
            print_lock.lock();
            printf("empty file on SD\n");
            print_lock.unlock();
            fclose(f);
            return false;
        }
        else{
            // if(EOF == fscanf(f, "%f, %f, %f, %f, %f, %f", &min_ext[0], &min_ext[1], &min_ext[2], &max_ext[0], &max_ext[1], &max_ext[2])){
            // fscanf does not work with %f. why??????

            if(fgets(buffer, 100, f) == NULL){ // get all line in a buffer
                print_lock.lock();
                printf("Error reading calibration file\n");
                print_lock.unlock();
                return false;
            }
            else{
                // sscanf(buffer, "%f, %f, %f, %f, %f, %f", &min_ext[0], &min_ext[1], &min_ext[2], &max_ext[0], &max_ext[1], &max_ext[2]);
                // sscanf does not work with %f. why??????

                // get data from buffer 'by hands'. read first float and then read every new space. must read 6 values 
                int y = 0;
                for (int i = 0; i < (int)strlen(buffer); i++){

                    if(buffer[i] == ' ' || i==0){
                        if(y < 3){
                            min_ext[y] = atof(&buffer[i]);
                            y++;
                        }else{
                            max_ext[y-3] = atof(&buffer[i]);
                            y++;
                        }
                    }
                }
                if (y != 6){
                    print_lock.lock();
                    printf("Error reading calibration file\n");
                    print_lock.unlock();
                    return false;
                }else{
                    print_lock.lock();
                    printf("Correct reading calibration file\n");
                    printf("read: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", min_ext[0], min_ext[1], min_ext[2], max_ext[0], max_ext[1], max_ext[2]);
                    print_lock.unlock();
                }
                fclose(f);
            }
        }
    }
    return true;
}

void save_calib_values(float* min_ext, float* max_ext){

    FILE* f = fopen("/sd/calibration_values.txt", "w+");
    if (!f) {
        print_lock.lock();
        printf("Error saving calibration values\n");
        print_lock.unlock();
    }else{
        fputs("Magnetometer extremes [minXYZ; maxXYZ]\n", f);
        fprintf(f, "%f, %f, %f, %f, %f, %f", min_ext[0], min_ext[1], min_ext[2], max_ext[0], max_ext[1], max_ext[2]);
        print_lock.lock();
        printf("Claibration values saved: %f, %f, %f, %f, %f, %f", min_ext[0], min_ext[1], min_ext[2], max_ext[0], max_ext[1], max_ext[2]);
        print_lock.unlock();
        fclose(f);
    }
    return;
}