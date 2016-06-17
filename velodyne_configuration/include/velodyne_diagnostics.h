#ifndef VELODYNE_DIAGNOSTICS_H
#define VELODYNE_DIAGNOSTICS_H

#include "velodyne_configuration/VLP16_DiagnosticsService.h"
#include "velodyne_configuration/VLP16_DiagnosticsRawService.h"
#include <velodyne_tools.h>

//class CVelodyneDiagnostics
//{
//public:
//    CVelodyneDiagnostics();

//    void run();

//protected:

//    bool get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse &res);
//    bool get_diagnostics(velodyne_configuration::VLP16_DiagnosticsServiceResponse &res);
//    inline bool get_diagnostics(velodyne_configuration::VLP16_DiagnosticsServiceRequest &req,
//                                velodyne_configuration::VLP16_DiagnosticsServiceResponse &res) {
//        return get_diagnostics(res);
//    }


//private:
//    //! Private NodeHandle
//    ros::NodeHandle nh_;

//    velodyne_tools::VLP16_WebServer webserver_;

//    ros::ServiceServer service_;

//    ros::Publisher velodyne_diagnostics_pub_;

//    double loop_rate_value_;

//    velodyne_configuration::VLP16_DiagnosticsServiceResponse laser_diagnostics_;
//};

//using namespace velodyne_configuration;
//using namespace velodyne_tools;

//class Velodyne_WebServer_Diagnostics : public velodyne_tools::Velodyne_WebServer_Services<velodyne_configuration::VLP16_DiagnosticsService, velodyne_configuration::VLP16_DiagnosticsMessage>
//{
//public:
//    Velodyne_WebServer_Diagnostics(const std::string& _name="diagnostics");

//    void run();

//protected:
//    bool get_response(velodyne_configuration::VLP16_DiagnosticsServiceResponse& _res);

//private:
//    bool get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse& _res);
//};

#endif // VELODYNE_DIAGNOSTICS_H
