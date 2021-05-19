/*! @file outportInit.cpp

    @author Davide Carminati

    @brief This thread creates the periodic task sending the pulse-width modulated signals to the enabled pins.

    @details The script uses events and a queue to pile periodic tasks. The queue contains both read-from and write-to pins events. 


*/
#include <mbed.h>
#include "Servo.h"
#include "global_vars.hpp"
#include "TankMotor.hpp"
#include "PWM_port.hpp"
#include "commander.hpp"


float pos = 75.0/180;
float delta_pos = 0.01;

// pin enabled for servomotor
// PwmOut servopwm(PTC3);

EventQueue queuePWM;
Event<void(void)> servowriteEvent(&queuePWM,ServoWriteHandler);
Event<void(void)> motorwriteEvent(&queuePWM,MotorWriteHandler);

/** Initialization of the servomotor(calibration)
 *  set freq for PWM
 *  define paramsfor events, post them and dispatch queue.
 */
void PWMport()
{
    print_lock.lock();
    printf("Start PWM thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();

    //servo1.calibrate(0.0005,90); // 0.0005 s from center (1.5ms) to max/min. The Servo::calibrate() method accepts as first input a value IN SECONDS.
    
    //servopwm.pulsewidth_us(1500);
    
    ServoWriteEventSetup();
    MotorWriteEventSetup();

    queuePWM.dispatch_forever(); // Also here the queue has to be started in this thread!!! otherwise doesn't dispatch

    // reach this point only when queue is stopped
    print_lock.lock();
    printf("End PWM thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();

    return;
}

// The period and the initial delay of the PWM write event are set. Then it is posted in the queue.
void ServoWriteEventSetup(void)
{
    servowriteEvent.period(500ms);
    servowriteEvent.delay(500ms);
    servowriteEvent.post();    
}

void MotorWriteEventSetup(void)
{
    motorwriteEvent.delay(4s);
    motorwriteEvent.period(200ms);
    motorwriteEvent.post();
}

/** The event handler is simply a method writing the computed duty cycle to the enabled pin port. The duty cycle is directly dependent on the output of the
 *  controller thread: note that the extern variable feedback_control_Y is present.
 */

void ServoWriteHandler(void)
{
    ExtY_PI_contr_T PI_Y;

    // get data from controller
    PI_Y = global_data->read_cntr();

    // put output on PWM only if armed
    if (main_commander->is_armed()){
        // output enabled
        ThisThread::sleep_for(50ms);
    }
    global_data->write_pwm(/*put data here*/);


    // TODO add semaphore in here!
    /*if (pos <= 75.0/180)
    {
        delta_pos = 0.01;
    }
    else if (pos >= 105.0/180)
    {
        delta_pos = -0.01;
    }
    printf("delta pos = %f  pos = %f\n", delta_pos, pos);
    pos = pos + delta_pos;
    servopwm.pulsewidth_us(pos*2000);
    */
    
    // servo1.write(pos);
    // servo2.write(pos);
    
    
    // pos = feedback_control_Y.u;//*180;
    // servo1.write(pos);
    // printf("\033[1;1H");
    // printf("pos given to pwm port: %f\n",pos);
}

void MotorWriteHandler(void)
{
    ExtY_PI_contr_T PI_Y;

    // get data from controller
    PI_Y = global_data->read_cntr();

    // put output on PWM only if armed
    if (main_commander->is_armed()){
        // output enabled
        ThisThread::sleep_for(50ms);
    }
    global_data->write_pwm(/*put data here*/);


    // printf("\033[2;50Hout1");
    // semContrPWM.acquire();
    // printf("\033[2;50Hout2");
    // leftMotor.Move(feedback_control_Y.pwm_left);
    // rightMotor.Move(feedback_control_Y.pwm_right);
    

    // leftMotor.Move(7000);
    // rightMotor.Move(7000);
    // ThisThread::sleep_for(1s);

    // leftMotor.Move(10000);
    // rightMotor.Move(10000);
    // ThisThread::sleep_for(3s);

    // leftMotor.Move(15000);
    // rightMotor.Move(15000);
    // ThisThread::sleep_for(3s);

    // leftMotor.Move(10000);
    // rightMotor.Move(-10000);
    // ThisThread::sleep_for(1s);

    // leftMotor.Move(-10000);
    // rightMotor.Move(-10000);
    // ThisThread::sleep_for(1s);

    // leftMotor.Move(0);
    // rightMotor.Move(0);
}



