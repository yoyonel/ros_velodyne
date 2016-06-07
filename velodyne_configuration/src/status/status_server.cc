//
#include <ros/ros.h>
//
#include "velodyne_configuration/VLP16_StatusService.h"
#include <velodyne_tools.h>
//
#include <string>


velodyne_tools::VLP16_WebServer webserver;


/**
 * @brief get_status
 * @param req
 * @param res
 * @return
 */
bool get_status(velodyne_configuration::VLP16_StatusServiceRequest  &req,
                velodyne_configuration::VLP16_StatusServiceResponse &res)
{
    const std::string res_request = webserver.request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::status);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );

    return webserver.parse_JSON_for_status(res_request, res);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_status_server");

    ros::NodeHandle ros_node("~");

    webserver.get_ip(ros_node);

    ros::ServiceServer service = ros_node.advertiseService("get_status", get_status);

    ROS_INFO("Ready to get velodyne status.");

    ros::spin();

    return 0;
}

