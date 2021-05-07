#include <stddef.h>
#include <stdio.h>                // This ert_main.c example uses printf/fflush
#include "rtwtypes.h"
#include "Kalman_filter_conv.h"        // Model's header file
#include "ekfInit.hpp"
#include "ekf_OneStep.hpp"

#include "global_vars.hpp"


static RT_MODEL_Kalman_filter_conv_T Kalman_filter_conv_M_;
static RT_MODEL_Kalman_filter_conv_T *const Kalman_filter_conv_MPtr = &Kalman_filter_conv_M_;              // Real-time model
static B_Kalman_filter_conv_T Kalman_filter_conv_B;// Observable signals
static DW_Kalman_filter_conv_T Kalman_filter_conv_DW;// Observable states

const char* ekf_thread_name = "EKF";

Thread EKF(osPriorityNormal,4096,nullptr,ekf_thread_name);

void ekfInit(void)
{
    print_lock.lock();
    printf("Start EKF init thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();

    RT_MODEL_Kalman_filter_conv_T *const Kalman_filter_conv_M = Kalman_filter_conv_MPtr;

    // Pack model data into RTM
    Kalman_filter_conv_M->blockIO = &Kalman_filter_conv_B;
    Kalman_filter_conv_M->dwork = &Kalman_filter_conv_DW;

    // Initialize model
    Kalman_filter_conv_initialize(Kalman_filter_conv_M, &Kalman_filter_conv_U, &Kalman_filter_conv_Y);

    EKF.start(callback(ekf, Kalman_filter_conv_M));

    print_lock.lock();
    printf("End EKF init thread ID: %d\n", (int)ThisThread::get_id());
    print_lock.unlock();
}