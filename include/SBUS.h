#ifndef SBUS
#define SBUS

#include "mbed.h"

// source code example: https://github.com/bolderflight/sbus

#define HEADER 0x0F
#define TAIL 0x00
#define FOOTER 0x00
#define FOOTER2 0x04


int sbus_init(PinName TX, PinName RX);

int sbus_get_data(uint*);

int sbus_fill_channels(uint*, uint*);

void sbus_use_channels_data(uint*);

#endif