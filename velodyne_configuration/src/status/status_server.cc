#include <ros/ros.h>
//
#include <velodyne_status.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_status_server");

    CVelodyneStatus vs;   // instanciation de l'object (à faire) APRES ros::init !
    vs.run();

    return 0;
}

