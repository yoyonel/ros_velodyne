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

const std::string str_IP_ADRESS_WEBSERVER_LIDAR = "192.168.1.201";
const float max_delay_for_cmd = 0.03f;

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
                str_IP_ADRESS_WEBSERVER_LIDAR,
                velodyne_tools::WebServerCommands::status,
                max_delay_for_cmd
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
        std::stringstream ss;
        ss << str_exec_res;
        pt::ptree root;
        boost::property_tree::read_json(ss, root);

        /**
         JSON: { ... "gps":{"pps_state":"Locked","position":"49 00.00N 200 .00W"} ... }
         - gps
         -- pps_state
         -- position
        /**/
        res.msg.gps_state = root.get<std::string>("gps.pps_state");
        res.msg.gps_position = root.get<std::string>("gps.position");
        /**
         JSON: { ... "motor":{"state":"On","rpm":0,"lock":"Off","phase":30755} ... }
         - motor
         -- state
         -- rpm
         -- lock
         -- phase
        /**/
        res.msg.motor_state = root.get<std::string>("motor.state") == "On";
        // url: http://www.cplusplus.com/forum/general/13135/
        //        res.msg.motor_rpm = boost::lexical_cast<uint16_t>(root.get<std::string>("motor.rpm"));
        res.msg.motor_rpm = root.get<uint16_t>("motor.rpm");
        res.msg.motor_lock = root.get<std::string>("motor.lock") != "Off";
        res.msg.motor_phase = root.get<uint16_t>("motor.phase");
        /**
         JSON: { ... "laser":{"state":"Disabled"} ... }
         - laser
         -- state
        /**/
        res.msg.laser_state = root.get<std::string>("laser.state") != "Disabled";
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

