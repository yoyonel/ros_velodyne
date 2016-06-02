//
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "velodyne_status_msgs/ServiceVelodyneStatus.h"
#include "velodyne_status_msgs/VelodyneStatusConfig.h"
//
#include <string>
//
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

const std::string str_IP_ADRESS_WEBSERVER_LIDAR = "192.168.1.201";
const float max_delay_for_cmd = 0.03f;

/**
 * @brief exec
 * @param cmd
 * @return
 */
std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

/**
 * @brief get_status
 * @param req
 * @param res
 * @return
 */
bool get_status(velodyne_status_msgs::ServiceVelodyneStatusRequest &req,
                velodyne_status_msgs::ServiceVelodyneStatusResponse &res)
{
    // url: http://fr.cppreference.com/w/cpp/string/basic_string/to_string
    std::string cmd_control_timing = "timeout " + std::to_string(max_delay_for_cmd) + "s ";
    std::string cmd_curl = "curl -s http://" + str_IP_ADRESS_WEBSERVER_LIDAR + "/cgi/status.json";
    std::string cmd = cmd_control_timing + cmd_curl;
    std::string str_exec_res = exec(cmd.c_str());

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
        //      ROS_ERROR(e.what());
        std::cerr << e.what() << std::endl;
    }

    return true;
}

/**
 * @brief callback
 * @param config
 * @param level
 */
void callback(velodyne_status_server::VelodyneStatusConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %s", config.str_net_addr.c_str());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_status_server");
    ros::NodeHandle n("~");

    dynamic_reconfigure::Server<velodyne_status_server::VelodyneStatusConfig> server;
    dynamic_reconfigure::Server<velodyne_status_server::VelodyneStatusConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::ServiceServer service = n.advertiseService("velodyne_status", get_status);
    ROS_INFO("Ready to get velodyne status.");
    ros::spin();

    return 0;
}

