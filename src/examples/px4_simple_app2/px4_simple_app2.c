#include <px4_platform_common/log.h>
#include <poll.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/uORB.h>

__EXPORT int px4_simple_app2_main(int argc, char *argv[]);

int px4_simple_app2_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    orb_set_interval(sensor_sub_fd, 200);

    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,   .events = POLLIN },
    };

    int error_counter = 0;

    for (int i = 0; i < 100; i++) {
        int poll_ret = px4_poll(fds, 1, 1000);

        if (poll_ret == 0){
            PX4_ERR("No data acquired within 1 second");
        } else if (poll_ret < 0) {
            if (error_counter < 10 || error_counter % 50 == 0) {
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;
        }
        else {


            if (fds[0].revents & POLLIN) {
                struct sensor_combined_s raw;

                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
                PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                    (double)raw.accelerometer_m_s2[0],
                    (double)raw.accelerometer_m_s2[1],
                    (double)raw.accelerometer_m_s2[2]);

                att.q[0] = raw.accelerometer_m_s2[0];
                att.q[1] = raw.accelerometer_m_s2[1];
                att.q[2] = raw.accelerometer_m_s2[2];

                orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
            }

        }

    }

    PX4_INFO("closing script...");

    return 0;
}



