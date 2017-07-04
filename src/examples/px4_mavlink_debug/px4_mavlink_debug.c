/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_mavlink_debug.c
 * Debug application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/vtol_vehicle_status.h>

__EXPORT int px4_mavlink_debug_main(int argc, char *argv[]);

int px4_mavlink_debug_main(int argc, char *argv[])
{
	printf("Hello Debug!\n");
    int trans_state_sub_fd=orb_subscribe(ORB_ID(vtol_vehicle_status));
    orb_set_interval(trans_state_sub_fd, 500);
    px4_pollfd_struct_t fds[] = {
            { .fd = trans_state_sub_fd,   .events = POLLIN },
          };

    /* advertise debug value */
    struct debug_key_value_s dbg = { .key = "in_rw_mode", .value = 0.0f };
	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

    int value_counter = 0;
    int error_counter = 0;
    while (error_counter < 500) {
		/* send one value */
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
                struct vtol_vehicle_status_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vtol_vehicle_status), trans_state_sub_fd, &raw);
                //PX4_INFO("Transition Status:\t%1f",
                 //    (double)raw.vtol_in_rw_mode);

                dbg.value = raw.vtol_in_rw_mode;
                orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
            }
        }
        //warnx("sent one more value..");

        value_counter++;
		usleep(500000);
	}
    PX4_INFO("exiting");
	return 0;
}
