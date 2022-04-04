#ifndef READ_WRITE_LOCK_H
#define READ_WRITE_LOCK_H

#include "mbed.h"

class Read_Write_Lock {

    public:
        Read_Write_Lock(){};

        void read_lock(){this->lock.lock();};

        void read_unlock(){this->lock.unlock();};
        
        void write_lock(){this->lock.lock();}; 

        void write_unlock(){this->lock.unlock();};

    private:
        rtos::Mutex lock;
};

#endif