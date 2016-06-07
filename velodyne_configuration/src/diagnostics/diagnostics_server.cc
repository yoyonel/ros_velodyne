//
#include <ros/ros.h>
//
#include "velodyne_configuration/VLP16_DiagnosticsService.h"
#include "velodyne_configuration/VLP16_DiagnosticsRawService.h"
#include <velodyne_tools.h>
//
#include <string>
//
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


velodyne_tools::VLP16_WebServer webserver;


/**
 * @brief get_diagnostics_raw
 * @param res
 * @return
 */
bool get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse &res)
{
    const std::string res_request = webserver.request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::diag);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
    return webserver.parse_JSON_for_diagnostics_raw(res_request, res);
}

/**
 * @brief get_diagnostics
 * @param req
 * @param res
 * @return
 */
bool get_diagnostics(velodyne_configuration::VLP16_DiagnosticsServiceRequest &req,
                     velodyne_configuration::VLP16_DiagnosticsServiceResponse &res)
{
    velodyne_configuration::VLP16_DiagnosticsRawServiceResponse res_raw;
    get_diagnostics_raw(res_raw);
    return webserver.scale_volt_temp(res_raw.msg, res.msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_diagnostics_server");
    ros::NodeHandle ros_node("~");

    webserver.get_ip(ros_node);

    ros::ServiceServer service = ros_node.advertiseService("get_diagnostics", get_diagnostics);

    ROS_INFO("Ready to get velodyne diagnostics.");

    ros::spin();

    return 0;
}

