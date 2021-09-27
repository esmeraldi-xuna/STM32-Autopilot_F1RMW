#include <mbed.h>
#include "BufferedSerial.h"
#include "global_vars.hpp"
#include "mavlink_serial.hpp"
#include "mavlink/common/mavlink.h"

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

    int recived = 0;

    // DEBUG
    main_commander->set_flag_comm_mavlink_rx(true);
    //////

    while(1){
        epoch = Kernel::Clock::now();

        if((recived = serial_ch->read(in_data, MAVLINK_MAX_PACKET_LEN)) > 0){
            
            for(int ii = 0; ii < recived; ii++) {

                if(mavlink_parse_char(MAVLINK_COMM_0, in_data[ii], &msg, &status))
                {
                    /*
                    print_lock.lock();
                    printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
                    print_lock.unlock();
                    */
                    switch (msg.msgid)
                    {
                        case MAVLINK_MSG_ID_HEARTBEAT:
                        main_commander->set_flag_comm_mavlink_rx(true);

                        /*
                        print_lock.lock();
                        printf("recived heartbeat\n");
                        print_lock.unlock();
                        */

                        heartbeat_recived = true;

                        break;
                        
                        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                        
                        mavlink_msg_set_position_target_local_ned_decode(&msg, &setpointsTrajectoryPlanner);

                        print_lock.lock();
                        printf("recived set_point: \n");
                        print_lock.unlock();

                        break;

                        case MAVLINK_MSG_ID_ODOMETRY:
                    
                        mavlink_msg_odometry_decode(&msg,&odom);
                        sensor_data = global_data->read_sensor();
                        sensor_data.ax = odom.x;
                        sensor_data.ay = odom.y;
                        sensor_data.az = odom.z;
                        sensor_data.mx = odom.vx;
                        sensor_data.my = odom.vy;
                        sensor_data.mz = odom.vz;
                        global_data->write_sensor(sensor_data);
                       
                        // printf("recived odometry: %f, %f, %f, %f, %f, %f\n", odom.x, odom.y, odom.z, odom.vx, odom.vy, odom.vz);

                        break;

                        case MAVLINK_MSG_ID_ATTITUDE:
                        
                        mavlink_msg_attitude_decode(&msg,&att);
                        sensor_data = global_data->read_sensor();
                        sensor_data.pitch = att.pitch;
                        sensor_data.yaw = att.yaw;
                        sensor_data.roll = att.roll;
                        global_data->write_sensor(sensor_data);
                        
                        // printf("recived attitude: %f, %f, %f\n", att.roll, att.pitch, att.yaw);

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
    std::chrono::milliseconds step = 500ms;

    uint8_t out_data[MAVLINK_MAX_PACKET_LEN];

    struct_sensors_data sensor_data;

    int sent = 0, pck_len = 0, cnt_for_heartbeat = 0;

    uint8_t SYS_ID = 8;  // for mavlink encoding
    uint8_t COMP_ID = 1;

    mavlink_message_t msg;
    mavlink_odometry_t odom_data;
    mavlink_attitude_t att;
    mavlink_heartbeat_t heart;


    // DEBUG
    main_commander->set_flag_comm_mavlink_tx(true);
    //////

    heartbeat_recived = false;

    mavlink_msg_heartbeat_encode(SYS_ID, COMP_ID, &msg, &heart);
    pck_len = mavlink_msg_to_send_buffer(out_data, &msg); 

    // sending heartbeat to start connection
    if((sent = serial_ch->write(out_data, pck_len)) < 0)
    {
        print_lock.lock();
        printf("Error sending data");
        print_lock.unlock();
    }  
    print_lock.lock();
    if (sent != pck_len)
        printf("error sending\n");
    else{
        printf("heartbeat sent correctly\n");
        main_commander->set_flag_comm_mavlink_tx(true);
    }

    printf("Waiting heartbeat...\n");
    print_lock.unlock();

    // when heartbeat recived -> heartbeat_recived=true
    while(heartbeat_recived == true)
        ThisThread::sleep_for(50ms);

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
        }/*
        if (sent != pck_len)
            printf("error sending\n");
        else
            printf("sent odometry: %f, %f, %f, %f, %f, %f\n", odom_data.x, odom_data.y, odom_data.z, odom_data.vx, odom_data.vy, odom_data.vz);
*/

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
        }/*
        if (sent != pck_len)
            printf("error sending\n");
        else
            printf("sent attitude: %f, %f, %f\n", att.roll, att.pitch, att.yaw);
*/
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