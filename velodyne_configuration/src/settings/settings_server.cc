//
#include <ros/ros.h>
//
#include <velodyne_settings.h>
//#include <velodyne_tools.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_settings_server");

    CVelodyneSettings vs;   // instanciation de l'object (à faire) APRES ros::init !
    vs.run();

    //    velodyne_tools::Velodyne_WebServer_Settings v_ws_s;
//    v_ws_s.run();

    return 0;
}

