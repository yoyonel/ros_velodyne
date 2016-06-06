//
#include <ros/ros.h>
//
#include "velodyne_configuration/VLP16_StatusService.h"
#include <velodyne_tools.h>
//
#include <string>
//
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


namespace pt = boost::property_tree;


/**
 * @brief get_status
 * @param req
 * @param res
 * @return
 */
bool get_status(velodyne_configuration::VLP16_StatusServiceRequest  &req,
                velodyne_configuration::VLP16_StatusServiceResponse &res)
{
    const std::string str_exec_res = velodyne_tools::request_webserver(
                velodyne_tools::str_IP_ADRESS_WEBSERVER_LIDAR,
                velodyne_tools::WebServerCommands::status,
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

        /**
         JSON: { ... "gps":{"pps_state":"Locked","position":"49 00.00N 200 .00W"} ... }
         - gps
         -- pps_state
         -- position
        /**/
        JSON_READ_STRING(root, gps.pps_state,  res, gps_state);
        JSON_READ_STRING(root, gps.position,   res, gps_position);
        /**
         JSON: { ... "motor":{"state":"On","rpm":0,"lock":"Off","phase":30755} ... }
         - motor
         -- state
         -- rpm
         -- lock
         -- phase
        /**/
        JSON_READ_BOOL(root, motor.state, res, motor_state);
        JSON_READ_UINT16(root, motor.rpm, res, motor_rpm);
        JSON_READ_BOOL(root, motor.lock,  res, motor_lock);
        JSON_READ_UINT16(root, motor.phase, res, motor_phase);
        /**
         JSON: { ... "laser":{"state":"Disabled"} ... }
         - laser
         -- state
        /**/
        JSON_READ_STATE(root, laser.state, res, laser_state);
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
    // ------------------------------------

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_status_server");
    ros::NodeHandle n("~");

    ros::ServiceServer service = n.advertiseService("get_status", get_status);
    ROS_INFO("Ready to get velodyne status.");
    ros::spin();

    return 0;
}

