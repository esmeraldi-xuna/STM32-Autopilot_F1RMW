
void sensInit(void);
void postSensorEvent(void);
void AccMagRead(void);
void calib_irq_handle(void);
void calibration(void);
void readFromSD(void);
void refreshParamFileSD(void);



/*
#ifndef SENSORS_INIT_H
#define SENSORS_INIT_H

void sensInit(void);

void setup_mag_event(void);
void setup_encoder_event(void);

// event handler for reading sensors
void AccMagRead(void);
void EncoderRead(void);

// handler for calibration
void calib_irq_handler(void);
void calibration(void);

// write new param on SD
void refreshParamFileSD(void);

#endif
*/