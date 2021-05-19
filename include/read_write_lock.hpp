#ifndef READ_WRITE_LOCK_H
#define READ_WRITE_LOCK_H

#include "mbed.h"

class Read_Write_Lock {

    public:
        Read_Write_Lock(){};

        void read_lock(){
            // wait untill lock is free ( using lock() ) and then unlock because multiple readings don't need exclusive access
            this->lock.lock(); 
            this->lock.unlock();
            return; 
        };

        void read_unlock(){}; // do nothing read is not blocking
        
        void write_lock(){this->lock.lock();}; // write is blocking

        void write_unlock(){this->lock.unlock();};

    private:
        rtos::Mutex lock;
};

#endif