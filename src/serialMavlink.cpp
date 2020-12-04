#include "mbed.h"

#include "global_vars.hpp"
#include "common/mavlink.h"
#include "global_msgs.hpp"
#include "serialMavlink.hpp"

mavlink_message_t msgIn;
mavlink_rc_channels_t rc;
mavlink_status_t status;

uint8_t SYS_ID = 1;
uint8_t COMP_ID = 1;

uint8_t in_data[MAVLINK_MAX_PACKET_LEN];

void SerialMavlink(Serial *serial)
{
    // Read from serial
    Stream *stream = serial;
    while(1)
    {
        if (stream->readable())
        {
            printf("Sto leggendo da seriale...\n");
            stream->read(&in_data, MAVLINK_MAX_PACKET_LEN);

            for(int ii = 0; ii < MAVLINK_MAX_PACKET_LEN; ii++) 
                {
                    uint8_t byte = in_data[ii];
                    if(mavlink_parse_char(MAVLINK_COMM_0, byte, &msgIn, &status))
                    {
                        switch (msgIn.msgid)
                        {
                        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                            mavlink_msg_set_position_target_local_ned_decode(&msgIn, &setpointsTrajectoryPlanner);
                            printf("\033[10;1H");
                            printf("setpoint: %f, %f, %f, %f, %f\n", setpointsTrajectoryPlanner.x, setpointsTrajectoryPlanner.y, \
                                setpointsTrajectoryPlanner.vx, setpointsTrajectoryPlanner.vy, setpointsTrajectoryPlanner.yaw);
                            break;

                        case MAVLINK_MSG_ID_ODOMETRY:
                            mavlink_msg_odometry_decode(&msgIn,&odom);
                            printf("\033[11;1H");
                            printf("odometry: %f, %f, %f, %f\n", odom.x, odom.y, odom.vx, odom.vy);
                            break;

                        case MAVLINK_MSG_ID_RC_CHANNELS:
                            mavlink_msg_rc_channels_decode(&msgIn, &rc);
                            printf("\033[13;1H");
                            printf("ch1: %d, ch2: %d, ch3: %d, ch4: %d \n", rc.chan1_raw, rc.chan2_raw, \
                                rc.chan3_raw, rc.chan4_raw);
                        
                        default:
                            printf("\033[4;1H");
                            printf("Mavlink message not decoded!\n");
                            break;
                        }
                        
                        // printf("\033[4;1H");
                        // printf("%f, %f, %f, %f, %f", odom.x, odom.q[0], odom.q[1], odom.q[2], odom.q[3]);
                        // semUDPNav.release();
                        // flagMavlink = true;
                    }
                }
        }
        else
        {
            printf("No mavlink message received...\n");
        }
    }
    

}
