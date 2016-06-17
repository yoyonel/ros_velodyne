#include "velodyne_diagnostics.h"

////
//#include "velodyne_configuration/VLP16_DiagnosticsMessage.h"

//CVelodyneDiagnostics::CVelodyneDiagnostics() :
//    nh_("~"), loop_rate_value_(1)
//{
//    webserver_.get_ip(nh_);

//    //----------------------------------------------------
//    // Services
//    //----------------------------------------------------
//    service_ = nh_.advertiseService("get_diagnostics", &CVelodyneDiagnostics::get_diagnostics, this);
//    //
//    ROS_INFO("Ready to get velodyne diagnostics.\t[SERVICE]");
//    //----------------------------------------------------

//    //----------------------------------------------------
//    // Publisher
//    //----------------------------------------------------
//    velodyne_diagnostics_pub_ = nh_.advertise<velodyne_configuration::VLP16_DiagnosticsMessage>("velodyne_diagnostics", 10);
//    //
//    ROS_INFO("Publish velodyne diagnostics messages.\t[PUBLISHER]");
//    //----------------------------------------------------
//}

//void CVelodyneDiagnostics::run()
//{
//    //----------------------------------------
//    // Boucle d'exécution ROS
//    //----------------------------------------
//    ros::Rate loop_rate(loop_rate_value_);   // 1 Hz
//    while (ros::ok())
//    {
//        if( velodyne_diagnostics_pub_.getNumSubscribers() != 0 ) {
//            if( get_diagnostics(laser_diagnostics_) ) {
//                velodyne_diagnostics_pub_.publish(laser_diagnostics_.msg);
//            }
//        }

//        ros::spinOnce();

//        loop_rate.sleep();
//    }
//    //----------------------------------------
//}

//bool CVelodyneDiagnostics::get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse &res)
//{
//    const std::string res_request = webserver_.request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::diag);
//    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
//    return webserver_.parse_JSON_for_diagnostics_raw(res_request, res);
//}

//bool CVelodyneDiagnostics::get_diagnostics(velodyne_configuration::VLP16_DiagnosticsServiceResponse &res)
//{
//    velodyne_configuration::VLP16_DiagnosticsRawServiceResponse res_raw;
//    get_diagnostics_raw(res_raw);
//    return webserver_.scale_volt_temp(res_raw.msg, res.msg);
//}

////---------------------------------------------------------------------------
//Velodyne_WebServer_Diagnostics::Velodyne_WebServer_Diagnostics(const std::string& _name) : Velodyne_WebServer_Services(_name )
//{
//}

//void Velodyne_WebServer_Diagnostics::run()
//{
//    Velodyne_WebServer_Services<velodyne_configuration::VLP16_DiagnosticsService, velodyne_configuration::VLP16_DiagnosticsMessage>::run_with_test_sub();
//}

//bool Velodyne_WebServer_Diagnostics::get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse& _res)
//{
//    const std::string res_request = webserver_.request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::diag);
//    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
//    return webserver_.parse_JSON_for_diagnostics_raw(res_request, _res);
//}

//bool Velodyne_WebServer_Diagnostics::get_response(velodyne_configuration::VLP16_DiagnosticsServiceResponse& _res)
//{
//    velodyne_configuration::VLP16_DiagnosticsRawServiceResponse res_raw;
//    get_diagnostics_raw(res_raw);
//    return webserver_.scale_volt_temp(res_raw.msg, _res.msg);
//}
////---------------------------------------------------------------------------
