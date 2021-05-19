#include <stddef.h>
#include <stdio.h>                // This ert_main.c example uses printf/fflush
#include "mbed.h"
#include "rtwtypes.h"
#include "ekf.hpp"
#include "Kalman_filter_conv.h"        // Model's header file
#include "global_vars.hpp"

void ekf()
{
  print_lock.lock();
  printf("Start EKF thread ID: %d\n", (int)ThisThread::get_id());
  print_lock.unlock();
 
  Kernel::Clock::time_point epoch;
  std::chrono::milliseconds step = 200ms;

  // init phase
  RT_MODEL_Kalman_filter_conv_T Kalman_filter_conv_M_;
  RT_MODEL_Kalman_filter_conv_T *const Kalman_filter_conv_M = &Kalman_filter_conv_M_;              // Real-time model
  B_Kalman_filter_conv_T Kalman_filter_conv_B;// Observable signals
  DW_Kalman_filter_conv_T Kalman_filter_conv_DW;// Observable states

  ExtU_Kalman_filter_conv_T Kalman_filter_conv_U; // inputs
  ExtY_Kalman_filter_conv_T Kalman_filter_conv_Y; // outputs

  // Pack model data into RTM
  Kalman_filter_conv_M->blockIO = &Kalman_filter_conv_B;
  Kalman_filter_conv_M->dwork = &Kalman_filter_conv_DW;

  // Initialize model
  Kalman_filter_conv_initialize(Kalman_filter_conv_M, &Kalman_filter_conv_U, &Kalman_filter_conv_Y);

  // working phase
  while (1)
  {
    epoch = Kernel::Clock::now();

    // get data from sensors
    /*get data here*/global_data->read_sensor();

    // setup inputs
    // Kalman_filter_conv_U = .....

    // do step
    // Kalman_filter_conv_step(Kalman_filter_conv_M, &Kalman_filter_conv_U, &Kalman_filter_conv_Y);
    ThisThread::sleep_for(100ms);

    // write result on data structure
    global_data->write_ekf(Kalman_filter_conv_U, Kalman_filter_conv_Y);

    ThisThread::sleep_until(epoch+step);
  }
}


////////////////////////////////////////////////////////////////////////////////////

/*
    ekf_timer.start();
    while (1)
    {
        
    
        // wait data from sensors
        sem_sens_EKF.acquire();

        // do something
        ThisThread::sleep_for(100ms);

        // new data available, allows NAV and controller to do one step
        sem_EKF_NAV_ctrl.release();
        sem_EKF_NAV_ctrl.release();
        continue;
        //////////////////////////////////////////////////////////////////////////////////////////////////
    
        ekf_epoch = Kernel::Clock::now();
        // static boolean_T OverrunFlag = false;

        // Disable interrupts here

        // Check for overrun
        // if (OverrunFlag) {
        //     rtmSetErrorStatus(Kalman_filter_conv_M, "Overrun");
        //     return;
        // }

        // OverrunFlag = true;

        // Save FPU context here (if necessary)
        // Re-enable timer or interrupt here
        // Set model inputs here

        // Inputs defined in sensinit.cpp

        // Step the model
        // Kalman_filter_conv_step(Kalman_filter_conv_M, &Kalman_filter_conv_U, &Kalman_filter_conv_Y);

        // Get model outputs here

        // Indicate task complete
        // OverrunFlag = false;

        // Disable interrupts here
        // Restore FPU context here (if necessary)
        // Enable interrupts here
        ThisThread::sleep_until(ekf_epoch+200ms);
    }
}
*/