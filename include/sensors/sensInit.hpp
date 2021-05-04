#ifndef SENSORS_INIT_H
#define SENSORS_INIT_H

void sensInit(void);

// setup event
void postSensorEvent(void);

// event handler for reading sensors
void read_sensors_eventHandler(void);

#endif