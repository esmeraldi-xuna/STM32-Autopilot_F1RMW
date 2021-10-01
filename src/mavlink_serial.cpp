#include <mbed.h>
#include "BufferedSerial.h"
#include "global_vars.hpp"
#include "mavlink_serial.hpp"
#include "mavlink/common/mavlink.h"

// global bool used for check correct communication start 
bool heartbeat_recived = false;

void mavlink_serial_RX(BufferedSerial* serial_ch){

    Kernel::Clock::time_point epoch;
    std::chrono::milliseconds step = 500ms;

    uint8_t in_data[MAVLINK_MAX_PACKET_LEN];

    struct_sensors_data sensor_data;

    // mavlink datastructure
    mavlink_message_t msg;
    mavlink_status_t status;

    mavlink_attitude_t att;
    mavlink_odometry_t odom;
    mavlink_set_position_target_local_ned_t setpointsTrajectoryPlanner;

    int byte_recived = 0;

    // DEBUG
    main_commander->set_flag_comm_mavlink_rx(true);
    //////

    while(1){
        // get actual time
        epoch = Kernel::Clock::now();

        // check if something is arrived
        if((byte_recived = serial_ch->read(in_data, MAVLINK_MAX_PACKET_LEN)) > 0){
            
            // get messages
            for(int ii = 0; ii < byte_recived; ii++) {

                // check if message is correct
                if(mavlink_parse_char(MAVLINK_COMM_0, in_data[ii], &msg, &status))
                {
                    /*
                    print_lock.lock();
                    printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
                    print_lock.unlock();
                    */

                    // switch on message type 
                    switch (msg.msgid)
                    {
                        // heartbeat for communication start/status
                        case MAVLINK_MSG_ID_HEARTBEAT:

                        // notify commander that mavlink_rx is ok
                        main_commander->set_flag_comm_mavlink_rx(true);

                        /*
                        print_lock.lock();
                        printf("recived heartbeat\n");
                        print_lock.unlock();
                        */

                        // set correct receive
                        heartbeat_recived = true;

                        break;
                        
                        // case positions target
                        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                        
                        // decode message
                        mavlink_msg_set_position_target_local_ned_decode(&msg, &setpointsTrajectoryPlanner);

                        /*
                        print_lock.lock();
                        printf("recived set_point: \n");
                        print_lock.unlock();
                        */

                        // do something with message

                        break;

                        // case odometry
                        case MAVLINK_MSG_ID_ODOMETRY:

                        mavlink_msg_odometry_decode(&msg,&odom);
                       
                        // printf("recived odometry: %f, %f, %f, %f, %f, %f\n", odom.x, odom.y, odom.z, odom.vx, odom.vy, odom.vz);

                        // do something with message

                        break;

                        // case attitude
                        case MAVLINK_MSG_ID_ATTITUDE:
                        
                        mavlink_msg_attitude_decode(&msg,&att);
                        
                        // printf("recived attitude: %f, %f, %f\n", att.roll, att.pitch, att.yaw);

                        // do something with message

                        break;
                        
                        default:
                        // error
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
    std::chrono::milliseconds step = 500ms;

    uint8_t out_data[MAVLINK_MAX_PACKET_LEN];

    struct_sensors_data sensor_data;

    int sent = 0, pck_len = 0, cnt_for_heartbeat = 0;

    uint8_t SYS_ID = 8;  // for mavlink encoding
    uint8_t COMP_ID = 1;

    // mavlink datastructure
    mavlink_message_t msg;
    mavlink_odometry_t odom_data;
    mavlink_attitude_t att;
    mavlink_heartbeat_t heart;


    // DEBUG
    main_commander->set_flag_comm_mavlink_tx(true);
    //////

    heartbeat_recived = false;

    // setup heartbeat message
    mavlink_msg_heartbeat_encode(SYS_ID, COMP_ID, &msg, &heart);
    pck_len = mavlink_msg_to_send_buffer(out_data, &msg); 

    print_lock.lock();
    // sending heartbeat to start connection
    if((sent = serial_ch->write(out_data, pck_len)) < 0)
    {
        printf("Error sending data");
    }  
    
    if (sent != pck_len)
        printf("error sending\n");
    else{
        printf("heartbeat sent correctly\n");
        main_commander->set_flag_comm_mavlink_tx(true);
    }

    // printf("Waiting heartbeat...\n");
    print_lock.unlock();

    // when heartbeat recived -> heartbeat_recived=true
    while(heartbeat_recived == false)
        ThisThread::sleep_for(50ms);

    // communication ok, start sending messages
    while(1)
    {
        epoch = Kernel::Clock::now();
        serial_ch->rewind();
        
        // preapare data to send
        sensor_data = global_data->read_sensor();
        odom_data.x = sensor_data.ax;
        odom_data.y = sensor_data.ay;
        odom_data.z = sensor_data.az;
        odom_data.vx = sensor_data.mx;
        odom_data.vy = sensor_data.my;
        odom_data.vz = sensor_data.mz;

        mavlink_msg_odometry_encode(SYS_ID, COMP_ID, &msg, &odom_data);
        pck_len = mavlink_msg_to_send_buffer(out_data, &msg); 

        // sending odom data...
        if((sent = serial_ch->write(out_data, pck_len)) < 0)
        {
            print_lock.lock();
            printf("Error sending data");
            print_lock.unlock();
        }
        if (sent != pck_len)
            printf("error sending\n");

        // preapare data to send
        att.pitch = sensor_data.pitch;
        att.yaw = sensor_data.yaw;
        att.roll = sensor_data.roll;

        mavlink_msg_attitude_encode(SYS_ID, COMP_ID, &msg, &att);
        pck_len = mavlink_msg_to_send_buffer(out_data, &msg); 

        // sending attitude data...
        if((sent = serial_ch->write(out_data, pck_len)) < 0)
        {
            print_lock.lock();
            printf("Error sending data");
            print_lock.unlock();
        }
        if (sent != pck_len)
            printf("error sending\n");
            
        // every 100 cycles send a heartbeat
        if(cnt_for_heartbeat == 100){
            cnt_for_heartbeat = 0;

            mavlink_msg_heartbeat_encode(SYS_ID, COMP_ID, &msg, &heart);
            mavlink_msg_to_send_buffer(out_data, &msg);
            serial_ch->write(out_data, pck_len);
        }

        cnt_for_heartbeat++;
        ThisThread::sleep_until(epoch+step);
    }
}