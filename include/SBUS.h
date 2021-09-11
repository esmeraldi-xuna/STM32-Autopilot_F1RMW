#ifndef SBUS
#define SBUS

#include "mbed.h"

// source code example: https://github.com/bolderflight/sbus

#define HEADER 0x0F
#define TAIL 0x00
#define FOOTER 0x00
#define FOOTER2 0x04


int sbus_init(PinName TX, PinName RX);

int sbus_get_data(unsigned int*);

int sbus_fill_channels(unsigned int*, unsigned int*);

void sbus_use_channels_data(unsigned int*);

#endif