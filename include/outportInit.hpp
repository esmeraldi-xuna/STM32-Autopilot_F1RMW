#ifndef OUT_INIT_H
#define OUT_INIT_H

// Initialization of the servomotor
void outportInit(void);

// set params and post event
void ServoWriteEventSetup(void);
void MotorWriteEventSetup(void);

// event handlers
void ServoWriteHandler(void);
void MotorWriteHandler(void);

#endif