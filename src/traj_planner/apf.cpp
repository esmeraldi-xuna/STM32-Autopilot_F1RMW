#include <stddef.h>
#include <stdio.h>                // This ert_main.c example uses printf/fflush
#include "mbed.h"
#include "rtwtypes.h"
#include "APF_conver.h"                // Model's header file
#include "apf.hpp"
#include "TankMotor.hpp"
#include "global_vars.hpp"

void apf()
{
  print_lock.lock();
  printf("Start APF thread ID: %d\n", (int)ThisThread::get_id());
  print_lock.unlock();

  Kernel::Clock::time_point epoch;
  std::chrono::milliseconds step = 500ms;

  // init phase
  RT_MODEL_APF_conver_T APF_conver_M_;
  RT_MODEL_APF_conver_T *const APF_conver_M = &APF_conver_M_;// Real-time model 
  DW_APF_conver_T APF_conver_DW;  // Observable states

  ExtU_APF_conver_T APF_conver_U; // inputs
  ExtY_APF_conver_T APF_conver_Y; // outputs

  ExtY_Kalman_filter_conv_T EKF_Y;

  // Pack model data into RTM
  APF_conver_M->dwork = &APF_conver_DW;

  // Initialize model
  APF_conver_initialize(APF_conver_M, &APF_conver_U, &APF_conver_Y);

  // working phase
  while (1)
  {
    epoch = Kernel::Clock::now();

    // get data from EKF
    EKF_Y = global_data->read_ekf_Y();

    // setup inputs
    // APF_conver_U = ....

    // do step
    APF_conver_step(APF_conver_M, &APF_conver_U, &APF_conver_Y);
    ThisThread::sleep_for(100ms);

    // write result on data structure
    global_data->write_nav(APF_conver_U , APF_conver_Y);

    ThisThread::sleep_until(epoch+step);
  }
}



/*
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
*/