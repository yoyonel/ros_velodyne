#include <ros/ros.h>
//
#include <velodyne_diagnostics.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_diagnostics_server");

    CVelodyneDiagnostics vs;
    vs.run();

    return 0;
}

