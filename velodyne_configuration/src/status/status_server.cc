#include <ros/ros.h>
//
//#include <velodyne_status.h>
#include <velodyne_tools.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_status_server");

//    CVelodyneStatus vs;   // instanciation de l'object (à faire) APRES ros::init !

    velodyne_tools::Velodyne_WebServer_Status v_ws_s;
    v_ws_s.run();

    return 0;
}

