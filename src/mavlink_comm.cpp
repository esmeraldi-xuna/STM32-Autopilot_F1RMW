#include "mavlink_comm.h"
#include "mbed.h"
#include <EthernetInterface.h>
#include "global_vars.hpp"
#include "mavlink/common/mavlink.h"

mavlink_scaled_imu_t imu_k64;
mavlink_scaled_imu2_t imu_ext;
mavlink_wheel_distance_t encoders;

uint8_t SYS_ID = 1;
uint8_t COMP_ID = 17;
EthernetInterface eth;
SocketAddress sockAddr_out(ltpndIP, 8151);
SocketAddress sockAddr_in(ltpndIP, 14550);
UDPSocket socket;
uint8_t in_data[MAVLINK_MAX_PACKET_LEN], out_buf[MAVLINK_MAX_PACKET_LEN];

mavlink_message_t msgIn, ekf_data_fusedOut, imu_k64_Out, imu_ext_Out, encodersOut, hb;
mavlink_status_t status;
mavlink_rc_channels_t rc;

void mavlink_RX(void)
{   
    socket.set_timeout(0); 
    while (1)
    {
        if (socket.recvfrom(&sockAddr_in, &in_data, MAVLINK_MAX_PACKET_LEN) != NSAPI_ERROR_WOULD_BLOCK) // now act as a server and receive from simulink data back
        {
            for (int ii = 0; ii < MAVLINK_MAX_PACKET_LEN; ii++)
            {
                uint8_t byte = in_data[ii];
                if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msgIn, &status))
                {
                    switch (msgIn.msgid)
                    {
                    case MAVLINK_MSG_ID_RC_CHANNELS:
                        mavlink_msg_rc_channels_decode(&msgIn, &rc);
                        if (main_commander->get_main_FMS_state() == SYS_RUN_MANUAL)
                        {
                            int signs = 1, signt = 1;

                            if (rc.chan3_raw == 1)
                            {
                                signs = -1;
                            }

                            if (rc.chan4_raw == 1)
                            {
                                signt = -1;
                            }
                            struct_pwm_data pwmm;
                            pwmm.motorL = signs * rc.chan1_raw + signt * rc.chan2_raw;
                            pwmm.motorR = signs * rc.chan1_raw - signt * rc.chan2_raw;
                            global_data->write_pwm(pwmm);
                            //printf("Running joystick, got new pwm\n");
                        }
                        break;
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        printf(" Received hb\n");
                        break;
                        /* case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                            mavlink_msg_set_position_target_local_ned_decode(&msgIn, &setpointsTrajectoryPlanner);
                            printf("\033[10;1H");
                            printf("setpoint: %f, %f, %f, %f, %f\n", setpointsTrajectoryPlanner.x, setpointsTrajectoryPlanner.y, \
                                setpointsTrajectoryPlanner.vx, setpointsTrajectoryPlanner.vy, setpointsTrajectoryPlanner.yaw);
                            break;

                        case MAVLINK_MSG_ID_ODOMETRY:
                            mavlink_msg_odometry_decode(&msgIn,&odom);
                            // printf("\033[11;1H");

                            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", (float)epochUDP, APF_conver_Y.PWM_l, APF_conver_Y.PWM_r, odom.x, odom.y, odom.vx, odom.vy, atan2(2*odom.q[3]*odom.q[0], 1 - 2*pow(odom.q[3],2))*180/PI, encoders.distance[0], encoders.distance[1], debug_psi_ref, debug_vel_ref);
                            //printf("psi filter %f, psi mag %f \n", atan2(2*odom.q[3]*odom.q[0], 1 - 2*pow(odom.q[3],2))*180/PI, Kalman_filter_conv_U.psi_mag*180/PI);
                            break; */

                        // case MAVLINK_MSG_ID_IMU:
                        //     // printf("\033[11;1H");
                        //     // printf("odometry: %f, %f, %f, %f, %f\n", odom.x, odom.y, odom.vx, odom.vy, atan2(2*odom.q[3]*odom.q[2], 1 - 2*pow(odom.q[2],2))*180/PI);

                        //     break;

                    default:
                        printf("\033[4;1H");
                        printf("Mavlink message not decoded!\n");
                        break;
                    }
                }
            }
        }
        else
        {
           
        }
        ThisThread::sleep_for(2000ms);
    }
}
void mavlink_TX(void)
{ // printf("qui thread\n");
    // socket.set_timeout(2000);
    while (1)
    {
        struct_sensors_data ss = global_data->read_sensor();
        mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &hb, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        mavlink_msg_to_send_buffer((uint8_t *)&out_buf, &hb);
        
        if (socket.sendto(sockAddr_out, (const void *)out_buf, MAVLINK_MAX_PACKET_LEN) != NSAPI_ERROR_WOULD_BLOCK) // sending data...
        {
            // continue;
            // printf("ekf data sent!\n");
        }
        else
        {
            printf("Heartbeat not sent!\n");
        }
        imu_k64.time_boot_ms = 0;
        imu_k64.xacc = (int16_t)(ss.a.x * 1000); // occhio segni, son mg
        imu_k64.yacc = (int16_t)(ss.a.y * 1000);
        imu_k64.zacc = (int16_t)(ss.a.z * 1000);
        imu_k64.xmag = (int16_t)ss.m.x;
        imu_k64.ymag = (int16_t)ss.m.y;
        // quat_w = sin(Kalman_filter_conv_U.psi_mag/2)*10000;
        // imu_k64.zmag = (int16_t)quat_w;
        imu_k64.zmag = (int16_t)ss.m.z;
        // NB: Codice fw alpha usa mag e gyro di questo primo pacchetto per covarianze, solo zmag come quat
        mavlink_msg_scaled_imu_encode(SYS_ID, COMP_ID, &imu_k64_Out, &imu_k64);
        mavlink_msg_to_send_buffer((uint8_t *)&out_buf, &imu_k64_Out);
      
        if (socket.sendto(sockAddr_out, (const void *)out_buf, MAVLINK_MAX_PACKET_LEN) != NSAPI_ERROR_WOULD_BLOCK) // sending data...
        {
            // continue;
            // printf("ekf data sent!\n");
        }
        else
        {
            printf("Int. imu not sent!\n");
        }
       
        // external imu
        imu_ext.time_boot_ms = 0;
        imu_ext.xacc = (int16_t)(ss.a_ext.x * 1000);
        imu_ext.yacc = (int16_t)(ss.a_ext.y * 1000);
        imu_ext.zacc = (int16_t)(ss.a_ext.z * 1000);
        imu_ext.zmag = 0; // TODO: change variable name when ekf is not used
        // printf("Quaternion: %d  Angle: %f quat_w: %f \n", imu_k64.zmag, Kalman_filter_conv_U.psi_mag*180/PI, quat_w);
        //  Following variables in SCALED_IMU MAVLINK message are used to sent COVARIANCE values
        imu_ext.xmag = 0;                             // Covariance for ax
        imu_ext.ymag = 0;                             // Covariance for ay
        imu_ext.xgyro = (int16_t)(ss.g_ext.x * 1000); //  in [deg/s]
        imu_ext.ygyro = (int16_t)(ss.g_ext.y * 1000); //
        imu_ext.zgyro = (int16_t)(ss.g_ext.z * 1000); //
        // printf("gyr_x: %i, gyr_y: %d , gyr_z: %i acc_x: %i, acc_y: %i , acc_z: %i %i %i %i\n", imu_ext.xgyro, imu_ext.ygyro, imu_ext.zgyro, imu_ext.xacc, imu_ext.yacc, imu_ext.zacc, imu_k64.xacc, imu_k64.yacc, imu_k64.zacc);

        mavlink_msg_scaled_imu2_encode(SYS_ID, COMP_ID, &imu_ext_Out, &imu_ext);
        mavlink_msg_to_send_buffer((uint8_t *)&out_buf, &imu_ext_Out);
     
        if (socket.sendto(sockAddr_out, (const void *)out_buf, MAVLINK_MAX_PACKET_LEN) != NSAPI_ERROR_WOULD_BLOCK) // sending data...
        {
            // continue;
            // printf("ekf data sent!\n");
        }
        else
        {
            printf("Ext. imu data not sent!\n");
        }
     
        encoders.time_usec = 0;
        encoders.distance[0] = ss.posL * 0.02 * 3.14 / 180; // Kalman_filter_conv_U.pos_l*0.02*PI/180;
        encoders.distance[1] = ss.posR * 0.02 * 3.14 / 180;

        mavlink_msg_wheel_distance_encode(SYS_ID, COMP_ID, &encodersOut, &encoders);
        mavlink_msg_to_send_buffer((uint8_t *)&out_buf, &encodersOut);
     
        if (socket.sendto(sockAddr_out, (const void *)out_buf, MAVLINK_MAX_PACKET_LEN) != NSAPI_ERROR_WOULD_BLOCK) // sending data...
        {
            // continue;
            // printf("ekf data sent!\n");
        }
        else
        {
            printf("Encoder data not sent!\n");
        }
       
        ThisThread::sleep_for(5000ms);
    }
}