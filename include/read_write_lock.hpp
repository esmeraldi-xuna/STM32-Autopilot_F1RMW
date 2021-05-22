#ifndef READ_WRITE_LOCK_H
#define READ_WRITE_LOCK_H

#include "mbed.h"

class Read_Write_Lock {

    public:
        Read_Write_Lock(){};

        void read_lock(){
            // if lock is locked (owner != 0) wait and then unlock (read is not blocking)
            if ((int)this->lock.get_owner() != 0){
                this->lock.lock();
                this->lock.unlock();
            } 
            // else do nothing
        };

        void read_unlock(){}; // do nothing, read is not blocking
        
        void write_lock(){this->lock.lock();}; // write is blocking

        void write_unlock(){this->lock.unlock();};

    private:
        rtos::Mutex lock;
};

#endif