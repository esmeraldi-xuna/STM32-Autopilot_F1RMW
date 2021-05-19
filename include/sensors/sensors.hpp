#ifndef SENSORS_H
#define SENSORS_H

void sensors(void);

// setup event
void postSensorEvent(void);

// event handler for reading sensors
void read_sensors_eventHandler(void);

#endif