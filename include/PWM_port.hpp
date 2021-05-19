#ifndef PWM_PORT_H
#define PWM_PORT_H

// Initialization of the servomotor
void PWMport(void);

// set params and post event
void ServoWriteEventSetup(void);
void MotorWriteEventSetup(void);

// event handlers
void ServoWriteHandler(void);
void MotorWriteHandler(void);

#endif