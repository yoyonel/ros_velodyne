#include <ros/ros.h>
//
//#include <velodyne_diagnostics.h>
#include <velodyne_tools.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_diagnostics_server");

    velodyne_tools::Velodyne_WebServer_Diagnostics v_ws_d;
//    Velodyne_WebServer_Diagnostics v_wb_d;

    v_ws_d.run();

    return 0;
}

