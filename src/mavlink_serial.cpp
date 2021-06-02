#include <mbed.h>
#include "BufferedSerial.h"
#include "global_vars.hpp"
#include "mavlink_serial.hpp"
#include "mavlink/common/mavlink.h"

void mavlink_serial_RX(BufferedSerial* serial_ch){

    Kernel::Clock::time_point epoch;
    std::chrono::milliseconds step = 300ms;

    uint8_t in_data[MAVLINK_MAX_PACKET_LEN];

    // mavlink datastructure
    mavlink_message_t msg;
    mavlink_status_t status;

    mavlink_attitude_t att;
    mavlink_odometry_t odom;
    mavlink_set_position_target_local_ned_t setpointsTrajectoryPlanner;

    int recived = 0;

    while(1){
        epoch = Kernel::Clock::now();

        if((recived = serial_ch->read(in_data, MAVLINK_MAX_PACKET_LEN)) > 0){

            for(int ii = 0; ii < recived; ii++) {
                uint8_t byte = in_data[ii];

                if(mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
                {
                    print_lock.lock();
                    printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
                    print_lock.unlock();
                
                    switch (msg.msgid)
                    {
                    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                        
                        mavlink_msg_set_position_target_local_ned_decode(&msg, &setpointsTrajectoryPlanner);
                        /*
                        printf("\033[10;1H");
                        printf("setpoint: %f, %f, %f, %f, %f\n", setpointsTrajectoryPlanner.x, setpointsTrajectoryPlanner.y, \
                            setpointsTrajectoryPlanner.vx, setpointsTrajectoryPlanner.vy, setpointsTrajectoryPlanner.yaw);
                        */
                        // get data from setpointsTrajectoryPlanner

                        break;

                    case MAVLINK_MSG_ID_ODOMETRY:
                        
                        mavlink_msg_odometry_decode(&msg,&odom);
                        /*
                        printf("\033[11;1H");
                        printf("odometry: %f, %f, %f, %f, %f\n", odom.x, odom.y, odom.vx, odom.vy, atan2(2*odom.q[3]*odom.q[2], 1 - 2*pow(odom.q[2],2))*180/PI);
                        */
                        // get data from odom

                        break;

                    case MAVLINK_MSG_ID_ATTITUDE:
                        
                        mavlink_msg_attitude_decode(&msg,&att);
                        /*
                        printf("\033[11;1H");
                        printf("odometry: %f, %f, %f, %f, %f\n", odom.x, odom.y, odom.vx, odom.vy, atan2(2*odom.q[3]*odom.q[2], 1 - 2*pow(odom.q[2],2))*180/PI);
                        */
                        // get data from att

                        break;
                    
                    default:
                        print_lock.lock();
                        printf("Mavlink message not decoded!\n");
                        print_lock.unlock();
                        break;
                    }
                }
            }
        }
        ThisThread::sleep_until(epoch+step);
    }
}

void mavlink_serial_TX(BufferedSerial* serial_ch){

    Kernel::Clock::time_point epoch;
    std::chrono::milliseconds step = 300ms;

    uint8_t out_data[MAVLINK_MAX_PACKET_LEN];

    int sent = 0, pck_len = 0;

    uint8_t SYS_ID = 1;  // for mavlink encoding
    uint8_t COMP_ID = 1;

    mavlink_message_t msg;
    mavlink_odometry_t odom_data;
    mavlink_attitude_t att;

    while(1)
    {
        epoch = Kernel::Clock::now();

        // preapare data to send
        // odom, attitude data from sensor (or EKF)
        odom_data.x = 5; //  examples
        odom_data.y = 4; //

        att.pitch = 1;   //

        mavlink_msg_odometry_encode(SYS_ID, COMP_ID, &msg, &odom_data);
        pck_len = mavlink_msg_to_send_buffer(out_data, &msg); 

        // sending data...
        if((sent = serial_ch->write(out_data, pck_len)) < 0)
        {
            print_lock.lock();
            printf("Error sending data");
            print_lock.unlock();
        } 

        mavlink_msg_attitude_encode(SYS_ID, COMP_ID, &msg, &att);
        pck_len = mavlink_msg_to_send_buffer(out_data, &msg); 

        // sending data...
        if((sent = serial_ch->write(out_data, pck_len)) < 0)
        {
            print_lock.lock();
            printf("Error sending data");
            print_lock.unlock();
        } 

        ThisThread::sleep_until(epoch+step);
    }
}