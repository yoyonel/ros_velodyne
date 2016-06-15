//
#include <ros/ros.h>
//
#include <velodyne_settings.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_settings_server");

    CVelodyneSettings vs;   // instanciation de l'object (à faire) APRES ros::init !
    vs.run();

    return 0;
}

