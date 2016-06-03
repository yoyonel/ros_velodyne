//
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "velodyne_settings/VelodyneSettingsConfig.h"


// * @brief callback
// * @param config
// * @param level
// */
void callback(velodyne_settings::VelodyneSettingsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %s", config.net_addr.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_settings_server");
    ros::NodeHandle n("~");

    dynamic_reconfigure::Server<velodyne_settings::VelodyneSettingsConfig> server;
    dynamic_reconfigure::Server<velodyne_settings::VelodyneSettingsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Ready to set velodyne settings.");
    ros::spin();

    return 0;
}

