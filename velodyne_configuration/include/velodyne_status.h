#ifndef VELODYNE_STATUS_H
#define VELODYNE_STATUS_H

//
#include "velodyne_configuration/VLP16_StatusService.h"
//
#include <velodyne_tools.h>

class CVelodyneStatus
{
public:
    CVelodyneStatus();

    void run();

protected:
    bool get_status(velodyne_configuration::VLP16_StatusServiceResponse &res);
    inline bool get_status(
            velodyne_configuration::VLP16_StatusServiceRequest &req,
            velodyne_configuration::VLP16_StatusServiceResponse &res
            )
    {
        get_status(res);
    }

private:
    //! Private NodeHandle
    ros::NodeHandle nh_;

    velodyne_tools::VLP16_WebServer webserver_;

    ros::ServiceServer service_;

    ros::Publisher velodyne_status_pub_;
    velodyne_configuration::VLP16_StatusServiceResponse laser_status_;
};

#endif // VELODYNE_STATUS_H
