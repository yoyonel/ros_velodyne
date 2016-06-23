#include "velodyne_status.h"
//
#include "velodyne_configuration/VLP16_StatusMessage.h"
//
#include <ros/ros.h>

CVelodyneStatus::CVelodyneStatus() :
    nh_("~")
{
    /**webserver_.**/get_ip(nh_);

    //----------------------------------------------------
    // Services
    //----------------------------------------------------
    service_ = nh_.advertiseService("get_status", &CVelodyneStatus::get_status, this);
    //
    ROS_INFO("Ready to get velodyne status.\t[SERVICE]");
    //----------------------------------------------------

    //----------------------------------------------------
    // Publisher
    //----------------------------------------------------
    velodyne_status_pub_ = nh_.advertise<velodyne_configuration::VLP16_StatusMessage>("velodyne_status", 10);
    //
    ROS_INFO("Publish velodyne status messages.\t[PUBLISHER]");
    //----------------------------------------------------
}

void CVelodyneStatus::run() {
    //----------------------------------------
    // Boucle d'exécution ROS
    //----------------------------------------
    ros::Rate loop_rate(2);   // 1 Hz
    while (ros::ok())
    {
        // someone listening?
        if (velodyne_status_pub_.getNumSubscribers() != 0) {
            if( get_status(laser_status_) )
                velodyne_status_pub_.publish(laser_status_.msg);
        }
        ros::spinOnce();

        loop_rate.sleep();
    }
    //----------------------------------------
}

bool CVelodyneStatus::get_status(velodyne_configuration::VLP16_StatusServiceResponse &res)
{
    const std::string res_request = /**webserver_.**/request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::status);
    ROS_INFO_STREAM("response from VLP webserver: " << res_request );

    return /**webserver_.**/parse_JSON_for_status(res_request, res);
}
