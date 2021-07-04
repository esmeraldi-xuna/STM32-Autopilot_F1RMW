#ifndef SENSORS_H
#define SENSORS_H

void sensors(void);

void postSensorEvent(void);

void read_sensors_eventHandler(void);

void do_calibration_step(void);

bool try_get_calibration_values(float* min_ext, float* max_ext);

void save_calib_values(float* min_ext, float* max_ext);

int I2C_scan(void);

bool init_accel_gyro();

bool init_magn();

bool init_baro();

#endif