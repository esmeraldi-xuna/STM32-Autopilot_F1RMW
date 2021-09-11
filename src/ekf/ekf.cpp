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

  struct_sensors_data sens_data;

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

  main_commander->all_flags.ekf_active = true;

  // working phase
  while (1)
  {
    epoch = Kernel::Clock::now();

    // get data from sensors
    sens_data = global_data->read_sensor();

    // setup inputs
    // Kalman_filter_conv_U = .....

    // do step
    Kalman_filter_conv_step(Kalman_filter_conv_M, &Kalman_filter_conv_U, &Kalman_filter_conv_Y);
    ThisThread::sleep_for(100ms);

    // write result on data structure
    global_data->write_ekf(Kalman_filter_conv_U, Kalman_filter_conv_Y);

    ThisThread::sleep_until(epoch+step);
  }
}


/*

// External inputs (root inport signals with default storage)
typedef struct {
  real_T pos_l;                        // '<Root>/pos_l'
  real_T pos_r;                        // '<Root>/pos_r'
  real_T psi_mag;                      // '<Root>/psi_mag'
  real_T ax;                           // '<Root>/ax'
  real_T ay;                           // '<Root>/ay'
} ExtU_Kalman_filter_conv_T;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T X;                            // '<Root>/X'
  real_T Y;                            // '<Root>/Y'
  real_T psi;                          // '<Root>/psi '
  real_T Vx;                           // '<Root>/Vx '
  real_T Vy;                           // '<Root>/Vy '
} ExtY_Kalman_filter_conv_T;

*/