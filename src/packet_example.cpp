/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*          C Language Dynamic Spatial SDK, Version 2.3         */
/*   Copyright 2013, Xavier Orr, Advanced Navigation Pty Ltd    */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2013 Advanced Navigation Pty Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#define RADIANS_TO_DEGREES (180.0/M_PI)
#define pai 3.141592653

int com_port_number;

int an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);
	return SendBuf(com_port_number, an_packet_pointer(an_packet), an_packet_size(an_packet));
}

/*
 * This is an example of sending a configuration packet to Spatial.
 *
 * 1. First declare the structure for the packet, in this case sensor_ranges_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */
void set_sensor_ranges()
{
	an_packet_t *an_packet;
	sensor_ranges_packet_t sensor_ranges_packet;

	sensor_ranges_packet.permanent = TRUE;
	sensor_ranges_packet.accelerometers_range = accelerometer_range_4g;
	sensor_ranges_packet.gyroscopes_range = gyroscope_range_500dps;
	sensor_ranges_packet.magnetometers_range = magnetometer_range_2g;

	an_packet = encode_sensor_ranges_packet(&sensor_ranges_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}

void get_data(uint8_t requested_packet_id)
{
	/*an_packet_t *an_packet;
	sensor_ranges_packet_t sensor_ranges_packet;

	sensor_ranges_packet.permanent = TRUE;
	sensor_ranges_packet.accelerometers_range = accelerometer_range_4g;
	sensor_ranges_packet.gyroscopes_range = gyroscope_range_500dps;
	sensor_ranges_packet.magnetometers_range = magnetometer_range_2g;

	an_packet = encode_sensor_ranges_packet(&sensor_ranges_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);*/

	an_packet_t *an_packet;
	an_packet = encode_request_packet(requested_packet_id);
	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}
int main(int argc, char *argv[])
{
        ros::init(argc, argv, "navigation");
        ros::NodeHandle n;
        tf::TransformBroadcaster odom_broadcaster;
        ros::Time current_time,last_time;
        current_time=ros::Time::now();
        last_time=ros::Time::now();
        ros::Rate r(100);
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	
	system_state_packet_t system_state_packet;
	raw_sensors_packet_t raw_sensors_packet;
	euler_orientation_packet_t euler_orientation_packet;
	body_velocity_packet_t body_velocity_packet;
	ecef_position_packet_t ecef_position_packet;
	quaternion_orientation_packet_t quaternion_orientation_packet;
	external_position_velocity_packet_t external_position_velocity_packet;
	running_time_packet_t running_time_packet;
	
	int bytes_received;

	if (argc != 3)
	{
		printf("Usage - program com_port_number baud_rate\nExample - packet_example.exe 0 115200\n");
		exit(EXIT_FAILURE);
	}

	com_port_number = atoi(argv[1]);
	
	/* open the com port */
	if (OpenComport(com_port_number, atoi(argv[2])))
	{
		exit(EXIT_FAILURE);
	}
	
	an_decoder_initialise(&an_decoder);
	/*set_sensor_ranges();*/
	
	get_data(packet_id_system_state);
	while (n.ok())
	{
		/*get_data(packet_id_system_state);*/
		/*get_data(packet_id_ecef_position);*/
		get_data(packet_id_body_velocity);
		if ((bytes_received = PollComport(com_port_number, an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			/* increment the decode buffer length by the number of bytes received */
			an_decoder_increment(&an_decoder, bytes_received);
			
			/* decode all the packets in the buffer */
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				if (an_packet->id == packet_id_system_state) /* system state packet */
				{
					/* copy all the binary data into the typedef struct for the packet */
					/* this allows easy access to all the different values             */
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						printf("System State Packet:\n");
						printf("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
						printf("\tRoll = %f, Pitch = %f, Yaw = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
                                                double roll=(system_state_packet.orientation[0] * RADIANS_TO_DEGREES*pai)/360;
double pitch=(system_state_packet.orientation[1] * RADIANS_TO_DEGREES*pai)/360;
double yaw=(system_state_packet.orientation[2] * RADIANS_TO_DEGREES*pai)/360;

double w=cos(roll)*cos(pitch)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
double x=sin(roll)*cos(pitch)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw);
double y=cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*cos(pitch)*sin(yaw);
double z=cos(roll)*cos(pitch)*sin(yaw)-sin(roll)*sin(pitch)*cos(yaw);

current_time=ros::Time::now();
                                        geometry_msgs::Quaternion odom_quat;
                                        geometry_msgs::TransformStamped odom_trans;
                                        odom_trans.header.stamp=current_time;
                                        odom_trans.header.frame_id="odom";
                                        odom_trans.child_frame_id="base_link";
                                        odom_trans.transform.translation.x=0;
                                        odom_trans.transform.translation.y=0;
                                        odom_trans.transform.translation.z=0;
                                        odom_trans.transform.rotation.x=x;
                                        odom_trans.transform.rotation.y=y;
                                        odom_trans.transform.rotation.z=z;
                                        odom_trans.transform.rotation.w=w;
                                        odom_broadcaster.sendTransform(odom_trans);

printf("\t11111  Q0 = %f, Q1 = %f, Q2 = %f, Q3 = %f\n",x,y,z,w);



					}
				}
				else if (an_packet->id == packet_id_raw_sensors) /* raw sensors packet */
				{
					/* copy all the binary data into the typedef struct for the packet */
					/* this allows easy access to all the different values             */
					if(decode_raw_sensors_packet(&raw_sensors_packet, an_packet) == 0)
					{
						printf("Raw Sensors Packet:\n");
						printf("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
						printf("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
					}
				}/*no use*/
				/*else
				{
					printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
				}*/
				else if (an_packet->id == packet_id_euler_orientation) /* euler_orientation packet */
				{
					if(decode_euler_orientation_packet(&euler_orientation_packet, an_packet) == 0)					
					printf("euler_orientation_packet:\n");
					printf("\tRoll = %f, Pitch = %f, Yaw = %f\n", euler_orientation_packet.orientation[0] * RADIANS_TO_DEGREES, euler_orientation_packet.orientation[1] * RADIANS_TO_DEGREES, euler_orientation_packet.orientation[2] * RADIANS_TO_DEGREES);

					/*printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);*/
				}
				else if (an_packet->id == packet_id_body_velocity)
				{
					if(decode_body_velocity_packet(&body_velocity_packet,an_packet) == 0)					
					printf("body_velocity_packet_t:\n");
					printf("\tVX = %f, VY = %f, VZ = %f\n", body_velocity_packet.velocity[0], body_velocity_packet.velocity[1], body_velocity_packet.velocity[2]);

					/*printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);*/
				}
				else if (an_packet->id == packet_id_ecef_position)
				{
					if(decode_ecef_position_packet(&ecef_position_packet,an_packet) == 0)					
					printf("ecef_position_packet:\n");
					printf("\tX = %f, Y = %f, Z = %f\n", ecef_position_packet.position[0], ecef_position_packet.position[1], ecef_position_packet.position[2]);

				}
				else if (an_packet->id == packet_id_quaternion_orientation)
				{
					if(decode_quaternion_orientation_packet(&quaternion_orientation_packet,an_packet) == 0)					
					printf("quaternion_orientation_packet:\n");
					printf("\tQ0 = %f, Q1 = %f, Q2 = %f, Q3 = %f\n", quaternion_orientation_packet.orientation[0], quaternion_orientation_packet.orientation[1], quaternion_orientation_packet.orientation[2],quaternion_orientation_packet.orientation[3]);
                                        
				}
				else if (an_packet->id == packet_id_external_position_velocity)
				{
					if(decode_external_position_velocity_packet(&external_position_velocity_packet,an_packet) == 0)					
					printf("external_position_velocity_packet:\n");
					printf("\tVEL_NORTH = %f, VEL_EAST = %f, VEL_DOWN = %f\n", external_position_velocity_packet.velocity[0], external_position_velocity_packet.velocity[1], external_position_velocity_packet.velocity[2]);
				}
				else if (an_packet->id == packet_id_running_time)
				{
					if(decode_running_time_packet(&running_time_packet,an_packet) == 0)					
					printf("running_time_packet:\n");
					printf("\tseconds = %d, microseconds = %d", running_time_packet.seconds, running_time_packet.microseconds);
				}
				/* Ensure that you free the an_packet when your done with it or you will leak memory */
				an_packet_free(&an_packet);
			}
		}
#ifdef _WIN32
    Sleep(10);
#else
    usleep(10000);
#endif
	}
}
