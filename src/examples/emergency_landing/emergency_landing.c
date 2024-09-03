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
 * @file emergency_landing.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

__EXPORT int emergency_landing_main(int argc, char *argv[]);

int emergency_landing_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to vehicle_local_position topic */
	int local_pos_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	/* limit the update rate to 5 Hz */
	orb_set_interval(local_pos_sub_fd, 200);

	/* advertise attitude setpoint topic */
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = local_pos_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	while (true) {
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
				struct vehicle_local_position_s local_pos;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_local_position), local_pos_sub_fd, &local_pos);
				PX4_INFO("Local Position:\t%8.4f\t%8.4f\t%8.4f",
					 (double)local_pos.x,
					 (double)local_pos.y,
					 (double)local_pos.z);

                PX4_INFO("Local Velocity:\t%8.4f\t%8.4f\t%8.4f",
                    (double)local_pos.vx,
                    (double)local_pos.vy,
                    (double)local_pos.vz);                

				/* set att_sp and publish this information for other apps */
				att_sp.roll_body = local_pos.x; // Example: setting roll to x position (not meaningful, just an example)
				att_sp.pitch_body = local_pos.y; // Example: setting pitch to y position
				att_sp.yaw_body = local_pos.z; // Example: setting yaw to z position
				att_sp.thrust_body[0] = 0.0f;
				att_sp.thrust_body[1] = 0.0f;
				att_sp.thrust_body[2] = -9.81f; // Example: set thrust to counteract gravity

				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}