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
#include <uORB/topics/manual_control_setpoint.h>

extern "C" __EXPORT int px4_wd_debug_main(int argc, char *argv[]);

class WindDisturbanceControl
{
public:
    /**
         * Constructor
         */
        WindDisturbanceControl();

        /**
         * Destructor, also kills the main task
         */
        ~WindDisturbanceControl();

        /**
         * Start the control task.
         *
         * @return		OK on success.
         */
        int		start();
        /**
             * Task status
             *
             * @return  true if the mainloop is running
             */
         bool task_running() { return _task_running; }
private:

        bool        _task_should_exit;
        bool        _task_running;          /**< if true, task is running in its mainloop */
        int         _wd_task;

        int _manual_sub_fd;

        orb_advert_t _manual_pub;

        struct manual_control_setpoint_s _manual;
        struct manual_control_setpoint_s _manual_rev;

        void task_main();
        static void task_main_test(int argc, char *argv[]);

};
namespace wd_control
{

    WindDisturbanceControl	*g_wd = nullptr;
}

WindDisturbanceControl::WindDisturbanceControl():

    _task_should_exit(false),
    _task_running(false),
    _wd_task(-1),

    _manual_sub_fd(-1),

    _manual_pub(nullptr)
    //_manual{}
    //_manual_rev{}
{
    PX4_INFO("Class constructor");
}

WindDisturbanceControl::~WindDisturbanceControl()
{
    if (_wd_task != -1) {

            /* task wakes up every 100ms or so at the longest */
            _task_should_exit = true;

            /* wait for a second for the task to quit at our request */
            unsigned i = 0;

            do {
                /* wait 20ms */
                usleep(20000);

                /* if we have given up, kill it */
                if (++i > 50) {
                    px4_task_delete(_wd_task);
                    break;
                }
            } while (_wd_task != -1);
        }

        wd_control::g_wd = nullptr;
}

void WindDisturbanceControl::task_main_test(int argc, char *argv[])
{
    wd_control::g_wd->task_main();
}

void WindDisturbanceControl::task_main()
{
    PX4_INFO("Wind Disturbance debug...");
    _manual_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
    _manual_pub = orb_advertise(ORB_ID(manual_control_setpoint), &_manual_rev);
    _task_running = true;
    for (int i = 0; i < 10; i++)
    {
            bool updated;

            /* get pilots inputs */
            orb_check(_manual_sub_fd, &updated);

            if (updated) {

                    /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(manual_control_setpoint), _manual_sub_fd, &_manual);

                PX4_INFO("Flaps:\t%8.4f\t aux 5:%8.4f\t",
                         (double)_manual.flaps,
                         (double)_manual.aux5);

                }
                else
                {
                    PX4_INFO("No RC Data...");
                }
            if((double)_manual.aux5 > 0.9)
            {
                PX4_INFO("Enter Yaw Target Code...");
            }
            if(i>15)
            {
                _manual_rev.aux5=1;
                orb_publish(ORB_ID(manual_control_setpoint),_manual_pub,&_manual_rev);
            }
            usleep(500000);
       }
}

int WindDisturbanceControl::start()
{
    ASSERT(_wd_task == -1);

        /* start the task */
        _wd_task = px4_task_spawn_cmd("px4_wd_debug",
                             SCHED_DEFAULT,
                             SCHED_PRIORITY_MAX - 20,
                             4600,
                             (px4_main_t)&WindDisturbanceControl::task_main_test,
                             nullptr);

        if (_wd_task < 0) {
            warn("task start failed");
            return -errno;
        }

        return OK;
}

int px4_wd_debug_main(int argc, char *argv[])
{

    if (argc < 2) {
        PX4_ERR("usage: px4_wd_debug {start|stop}");
        return 1;
    }
    if (!strcmp(argv[1], "start")) {

            if (wd_control::g_wd != nullptr) {
                PX4_ERR("already running");
                return 1;
            }

            wd_control::g_wd = new WindDisturbanceControl();

            if (wd_control::g_wd == nullptr) {
                PX4_ERR("alloc failed");
                return 1;
            }

            if (OK != wd_control::g_wd->start()) {
                delete wd_control::g_wd;
                wd_control::g_wd = nullptr;
                PX4_ERR("start failed");
                return 1;
            }

            /* avoid memory fragmentation by not exiting start handler until the task has fully started */
            while (wd_control::g_wd == nullptr || !wd_control::g_wd->task_running()) {
                usleep(50000);
                PX4_INFO(".");
            }

            return 0;
        }
    if (!strcmp(argv[1], "stop")) {

            delete wd_control::g_wd;
            wd_control::g_wd = nullptr;
            return 0;
        }
    PX4_ERR("unrecognized command");
        return 1;

}
