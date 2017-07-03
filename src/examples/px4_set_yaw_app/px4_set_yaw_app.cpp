/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_set_yaw_app.c
 * Previous yaw set point test for further wind vane control mode
 *
 * @author Jingxuan Sun <jingxuan.j.sun@connect.polyu.hk>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_attitude.h>

extern "C" __EXPORT int px4_set_yaw_app_main(int argc, char *argv[]);

int px4_set_yaw_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    int control_state_sub_fd=orb_subscribe(ORB_ID(control_state));
    orb_set_interval(sensor_sub_fd, 200);
    orb_set_interval(control_state_sub_fd, 200);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
            { .fd = sensor_sub_fd,   .events = POLLIN },
            { .fd = control_state_sub_fd,   .events = POLLIN }
            /* there could be more file descriptors here, in the form like:
             * { .fd = other_sub_fd,   .events = POLLIN },
             */
        };

    int error_counter = 0;

        for (int i = 0; i < 5; i++) {
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
                    struct sensor_combined_s raw;
                    struct control_state_s	 _ctrl_state;
                    /* copy sensors raw data into local buffer */
                    orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
                    orb_copy(ORB_ID(control_state), control_state_sub_fd, &_ctrl_state);
                    math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
                    math::Vector<3> att_euler=q_att.to_euler()*57.3;

                    PX4_INFO("Attitude:\t%8.4f\t%8.4f\t%8.4f",
                         (double)att_euler(0),
                         (double)att_euler(1),
                         (double)att_euler(2));
                }

                /* there could be more file descriptors here, in the form like:
                 * if (fds[1..n].revents & POLLIN) {}
                 */
            }
        }

        PX4_INFO("exiting");

        return 0;
}
