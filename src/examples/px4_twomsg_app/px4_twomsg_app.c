/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>      //timestamp lib

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/a_tmn.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/servo_info.h>
#include <uORB/topics/servo_report.h>



__EXPORT int px4_twomsg_app_main(int argc, char *argv[]);
/*get vehicle_local_position and send to debug_vect with loop*/
int px4_twomsg_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello 2 Msg!");

	/* subscribe to vehicle_local_position topic */
	int vehicle_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	/* limit the update rate to 5 Hz */
	orb_set_interval(vehicle_sub_fd, 200);

	/* subscribe to sensor_gps topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_gps));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* subscribe to servo_report topic */
	int servo_report_sub_fd = orb_subscribe(ORB_ID(servo_report));
	/* limit the update rate to 5 Hz */
	orb_set_interval(servo_report_sub_fd, 200);


	/* advertise debug_vect topic */
	struct debug_vect_s dbg_vect;
	memset(&dbg_vect, 0, sizeof(dbg_vect));
	orb_advert_t dbg_vect_pub = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

	/* advertise debug_vect topic */
	// struct servo_info_s servo_info;
	// memset(&servo_info, 0, sizeof(servo_info));
	// orb_advert_t servo_info_pub = orb_advertise(ORB_ID(servo_info), &servo_info);


	/* advertise a_tmn topic */
	// struct a_tmn_s tmn;
	// memset(&tmn, 0, sizeof(tmn));
	// orb_advert_t tmn_pub = orb_advertise(ORB_ID(a_tmn), &tmn);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = vehicle_sub_fd,   .events = POLLIN },
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = servo_report_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;
	int value_counter = 0;


	while (value_counter < 100){
	//for (int i = 0; i < 100; i++) {
		uint64_t timestamp_us = hrt_absolute_time();
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_local_position_s pose;
				struct sensor_gps_s gps;

				struct servo_report_s report;

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_local_position), vehicle_sub_fd, &pose);
				orb_copy(ORB_ID(sensor_gps), sensor_sub_fd, &gps);

				orb_copy(ORB_ID(servo_report), servo_report_sub_fd, &report);

				/*[%8f]Vehicle Position:X:%8.4f Y:%8.4f Z:%8.4f\n */

				/* set dbg_vect and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/


				// servo_info.timestamp = timestamp_us;
				// servo_info.counter = 568; // test only int 0-3
				// servo_info.count = 1;
				// servo_info.connectiontype = SERVO_INFO_ESC_CONNECTION_TYPE_DSHOT;
				// servo_info.temperature[0]= 4000;
				// servo_info.error_count[0]= 1000;


				dbg_vect.x = report.servo_id;
				dbg_vect.y = report.servo_position;
				dbg_vect.z = report.servo_speed;
				dbg_vect.timestamp = timestamp_us;//report.timestamp;

				strncpy(dbg_vect.name, "svReport", 10);
				/*tmn.x = gps.lat;
				tmn.y = gps.lon;
				tmn.z = gps.alt;
				strncpy(tmn.name, "GPS", 10);*/
				//PX4_INFO("svReport id:%9.4f pos:%9.4f spd:%9.4f tst:%9.4f\n ",
					// (double)value_counter+1,
					// (double)pose.x,
					// (double)pose.y,
					// (double)pose.z,
					//  (double)dbg_vect.x,
					//  (double)dbg_vect.y,
					//  (double)dbg_vect.z
					//  (double)dbg_vect.timestamp);

				//orb_publish(ORB_ID(a_tmn), tmn_pub, &tmn);
				orb_publish(ORB_ID(debug_vect), dbg_vect_pub, &dbg_vect);
				//orb_publish(ORB_ID(servo_info), servo_info_pub, &servo_info);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
		value_counter++;
		px4_usleep(500000);
	}

	PX4_INFO("exiting");

	return 0;
}
