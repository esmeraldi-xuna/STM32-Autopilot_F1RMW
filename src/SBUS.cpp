#include "mbed.h"
#include "BufferedSerial.h"
#include "SBUS.h"

// source code example: https://github.com/bolderflight/sbus

#define uint unsigned int

static BufferedSerial* sbus_channel;

int sbus_init(PinName TX_pin, PinName RX_pin){

    sbus_channel = new BufferedSerial(TX_pin, RX_pin, 100000);
    if(sbus_channel == NULL){
        return -1;
    }
    sbus_channel->set_format(8, BufferedSerial::Even, 2);

    return 0;
}

int sbus_get_data(uint* buf){

    // DEBUG
    return false;
    ////////////

    // char c;
    int n_read = 0, state = 0;
    uint c = 0;

    // read message
    while (1) {
        
        n_read = sbus_channel->read(&c, 1);
        if (state == 0) {
            if (c == HEADER) {
            buf[state] = c;
            state++;
            } else {
                state = 0;
            }
        } else {
            if (state < 25) {
            buf[state] = c;
            state++;
            } else {
                if ((buf[24] == FOOTER) || ((buf[24] & 0x0F) == FOOTER2)) {
                    return true;
                } else {
                    return false;
                }
            }
        }
    }
    return false;
}


int sbus_fill_channels(uint* raw_data, uint* output){

    output[0]  = (uint)((raw_data[1]      ) | ((raw_data[2]  << 8) & 0x07FF));
    output[1]  = (uint)((raw_data[2]  >> 3) | ((raw_data[3]  << 5) & 0x07FF));
    output[2]  = (uint)((raw_data[3]  >> 6) | ((raw_data[4]  << 2) | ((raw_data[5] << 10) & 0x07FF)));
    output[3]  = (uint)((raw_data[5]  >> 1) | ((raw_data[6]  << 7) & 0x07FF));
    output[4]  = (uint)((raw_data[6]  >> 4) | ((raw_data[7]  << 4) & 0x07FF));
    output[5]  = (uint)((raw_data[7]  >> 7) | ((raw_data[8]  << 1) |  ((raw_data[9] << 9) & 0x07FF)));
    output[6]  = (uint)((raw_data[9]  >> 2) | ((raw_data[10] << 6) & 0x07FF));
    output[7]  = (uint)((raw_data[10] >> 5) | ((raw_data[11] << 3) & 0x07FF));
    output[8]  = (uint)((raw_data[12]     ) | ((raw_data[13] << 8) & 0x07FF));
    output[9]  = (uint)((raw_data[13] >> 3) | ((raw_data[14] << 5) & 0x07FF));
    output[10] = (uint)((raw_data[14] >> 6) | ((raw_data[15] << 2) |  ((raw_data[16] << 10) & 0x07FF)));
    output[11] = (uint)((raw_data[16] >> 1) | ((raw_data[17] << 7) & 0x07FF));
    output[12] = (uint)((raw_data[17] >> 4) | ((raw_data[18] << 4) & 0x07FF));
    output[13] = (uint)((raw_data[18] >> 7) | ((raw_data[19] << 1) |  ((raw_data[20] << 9) & 0x07FF)));
    output[14] = (uint)((raw_data[20] >> 2) | ((raw_data[21] << 6) & 0x07FF));
    output[15] = (uint)((raw_data[21] >> 5) | ((raw_data[22] << 3) & 0x07FF));

    return true;
}

void sbus_use_channels_data(uint* data){
    
    // do something

    return;
}