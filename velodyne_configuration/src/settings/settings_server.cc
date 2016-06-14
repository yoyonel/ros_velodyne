//
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
//
#include "velodyne_configuration/VLP16_settingsConfig.h"
#include "velodyne_configuration/VLP16_SettingsService.h"
//
#include <velodyne_tools.h>


velodyne_tools::VLP16_WebServer webserver;


/**
 * @brief get_settings
 * @param req
 * @param res
 * @return
 */
bool get_settings(velodyne_configuration::VLP16_SettingsServiceRequest  &req,
                  velodyne_configuration::VLP16_SettingsServiceResponse &res)
{
    const std::string res_request = webserver.request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::settings);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );

    return webserver.parse_JSON_for_settings(res_request, res);
}

// * @brief callback
// * @param config
// * @param level
// */
void callback(velodyne_configuration::VLP16_settingsConfig &config, uint32_t level) {
    const int return_set_configs = webserver.send_settings_to_webserver(config);
    ROS_INFO("Reconfigure Request: %d", return_set_configs);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_settings_server");

    ros::NodeHandle ros_node("~");

    webserver.get_ip(ros_node);


    //----------------------------------------------------
    // Dynamic Parameter server
    //----------------------------------------------------
    dynamic_reconfigure::Server<velodyne_configuration::VLP16_settingsConfig> server;
    dynamic_reconfigure::Server<velodyne_configuration::VLP16_settingsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    //
    ROS_INFO("Ready to set velodyne settings.");
    //----------------------------------------------------


    //----------------------------------------------------
    // Services
    //----------------------------------------------------
    ros::ServiceServer service = ros_node.advertiseService("get_settings", get_settings);
    //
    ROS_INFO("Ready to get velodyne settings.");
    //----------------------------------------------------

    ros::spin();

    return 0;
}

