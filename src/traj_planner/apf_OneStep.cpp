#include <stddef.h>
#include <stdio.h>                // This ert_main.c example uses printf/fflush
#include "mbed.h"
#include "rtwtypes.h"
#include "APF_conver.h"                // Model's header file
#include "apf_OneStep.hpp"
#include "TankMotor.hpp"

#include "global_vars.hpp"

Timer apf_timer;
Kernel::Clock::time_point apf_epoch;

void apf(RT_MODEL_APF_conver_T *const APF_conver_M)
{
    apf_timer.start();
    while (1)
    {
        ////////////////////////////////////// for debug///////////////////////////////////////////////
    
        // wait data from EKF
        sem_EKF_NAV_ctrl.acquire();

        // do something
        ThisThread::sleep_for(100ms);

        // new data available, allows controller to do one step
        sem_nav_ctrl.release();
        continue;
        //////////////////////////////////////////////////////////////////////////////////////////////////
    
        apf_epoch = Kernel::Clock::now();
        // static boolean_T OverrunFlag = false;

        // Disable interrupts here

        // Check for overrun
        // if (OverrunFlag) {
        //     rtmSetErrorStatus(APF_conver_M, "Overrun");
        //     return;
        // }

        // OverrunFlag = true;

        // Save FPU context here (if necessary)
        // Re-enable timer or interrupt here
        // Set model inputs here

        #if MAVLINK
        APF_conver_U.psi_est = atan2(2*odom.q[3]*odom.q[2], 1 - 2*pow(odom.q[2],2));
        APF_conver_U.Vx_est = odom.vx*cos(atan2(2*odom.q[3]*odom.q[2], 1 - 2*pow(odom.q[2],2)))+odom.vy*sin(atan2(2*odom.q[3]*odom.q[2], 1 - 2*pow(odom.q[2],2))); // Vx in body frame
        APF_conver_U.X_est = odom.x;
        APF_conver_U.Y_est = odom.y;
        #endif

        // Step the model
        APF_conver_step(APF_conver_M, &APF_conver_U, &APF_conver_Y);

        // leftMotor.Move(APF_conver_Y.PWM_l);
        // rightMotor.Move(APF_conver_Y.PWM_r);

        // Get model outputs here

        // Indicate task complete
        // OverrunFlag = false;

        // Disable interrupts here
        // Restore FPU context here (if necessary)
        // Enable interrupts here
        ThisThread::sleep_until(apf_epoch+100ms);
    }
}