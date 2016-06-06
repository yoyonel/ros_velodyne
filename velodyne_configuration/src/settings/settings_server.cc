//
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
//
#include "velodyne_configuration/VLP16_settingsConfig.h"
#include "velodyne_configuration/VLP16_SettingsService.h"
//
#include <velodyne_tools.h>


namespace pt = boost::property_tree;


/**
 * @brief get_settings
 * @param req
 * @param res
 * @return
 */
bool get_settings(velodyne_configuration::VLP16_SettingsServiceRequest  &req,
               velodyne_configuration::VLP16_SettingsServiceResponse &res)
{
   const std::string str_exec_res = velodyne_tools::request_webserver(
               velodyne_tools::str_IP_ADRESS_WEBSERVER_LIDAR,
               velodyne_tools::WebServerCommands::settings,
               velodyne_tools::max_delay_for_cmd
               );
   //
   ROS_INFO_STREAM("response from VLP webserver: " << str_exec_res);

   // ------------------------------------
   // Manual JSON file parsing
   // ------------------------------------
   // urls:
   // - http://zenol.fr/blog/boost-property-tree/en.html
   // - https://gist.github.com/mloskot/1509935
   try
   {
       JSON_INIT(root, str_exec_res);

       JSON_READ_BOOL   (root, laser,                res, laser_state);
       JSON_READ_STRING (root, returns,              res, returns);
       JSON_READ_UINT16 (root, rpm,                  res, rpm);
       JSON_READ_UINT16 (root, fov.start,            res, fov_start);
       JSON_READ_UINT16 (root, fov.end,              res, fov_end);
       JSON_READ_STATE  (root, phaselock.enabled,    res, phaselock_enabled);
       JSON_READ_UINT16 (root, phaselock.offset,     res, phaselock_offset);
       JSON_READ_STRING (root, host.addr,            res, host_addr);
       JSON_READ_UINT16 (root, host.dport,           res, host_dport);
       JSON_READ_UINT16 (root, host.tport,           res, host_tport);
       JSON_READ_STRING (root, net.addr,             res, net_addr);
       JSON_READ_STRING (root, net.mask,             res, net_mask);
       JSON_READ_STRING (root, net.gateway,          res, net_gateway);
       JSON_READ_BOOL   (root, net.dhcp,             res, net_dhcp);

   }
   catch (std::exception const& e)
   {
       ROS_ERROR_STREAM(e.what());
   }
   // ------------------------------------

   return true;
}

// * @brief callback
// * @param config
// * @param level
// */
void callback(velodyne_configuration::VLP16_settingsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %s", config.net_addr.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_settings_server");
    ros::NodeHandle n("~");

    dynamic_reconfigure::Server<velodyne_configuration::VLP16_settingsConfig> server;
    dynamic_reconfigure::Server<velodyne_configuration::VLP16_settingsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    //
    ROS_INFO("Ready to set velodyne settings.");

    ros::ServiceServer service = n.advertiseService("get_settings", get_settings);
    //
    ROS_INFO("Ready to get velodyne settings.");

    ros::spin();

    return 0;
}

