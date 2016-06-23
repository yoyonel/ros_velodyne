#include <velodyne_webserver.h>


namespace velodyne_webserver {

///**
// * @brief VLP16_WebServer::request_webserver
// * @param _str_ip_laser
// * @param _cmd
// * @param _max_delay_for_cmd
// * @return
// */
//std::string VLP16_WebServer::request_webserver(const WebServerCommands& _cmd) const
//{
//    //    return request_webserver_curl(_cmd);
//    //    return request_webserver_asio_synch(_cmd);
//    return request_webserver_asio_asynch(_cmd);
//}

//std::string VLP16_WebServer::request_webserver_curl(const WebServerCommands& _cmd) const
//{
//    // url: http://stackoverflow.com/questions/23936246/error-invalid-operands-of-types-const-char-35-and-const-char-2-to-binar
//    const std::string cmd_curl =                    \
//            "curl -s http://" +                     \
//            network_sensor_ip_ +                   \
//            "/cgi/" + _cmd._to_string() + ".json";

//    const std::string cmd_bash =
//            (max_delay_for_cmd_ != 0.0f ?                                  \
//                "timeout " + std::to_string(max_delay_for_cmd_) + "s " :   \
//                "")                                                        \
//            + cmd_curl;
//    //
//    ROS_INFO_STREAM("Commande bash: " << cmd_bash);

//    return velodyne_tools::exec_cmd(cmd_bash.c_str());
//}

//std::string VLP16_WebServer::request_webserver_asio_asynch(const WebServerCommands& _cmd) const
//{
//    std::string response_json = "";
//    try {
//        boost::asio::io_service io_service;
//        client_asynch c(
//                    io_service,
//                    network_sensor_ip_,
//                    std::string("/cgi/") + _cmd._to_string() + ".json");
//        io_service.run();

//        response_json = c.get_response();
//    }
//    catch (std::exception& e)
//    {
//        std::cout << "Exception: " << e.what() << "\n";
//    }
//    return response_json;
//}

//std::string VLP16_WebServer::request_webserver_asio_synch(const WebServerCommands& _cmd) const
//{
//    // url: http://www.boost.org/doc/libs/1_49_0/doc/html/boost_asio/example/http/client/sync_client.cpp
//    using boost::asio::ip::tcp;

//    std::string response_json = "";

//    try
//    {
//        boost::asio::io_service io_service;

//        // Get a list of endpoints corresponding to the server name.
//        tcp::resolver resolver(io_service);
//        tcp::resolver::query query(network_sensor_ip_, "http");
//        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

//        // Try each endpoint until we successfully establish a connection.
//        tcp::socket socket(io_service);
//        boost::asio::connect(socket, endpoint_iterator);

//        // Form the request. We specify the "Connection: close" header so that the
//        // server will close the socket after transmitting the response. This will
//        // allow us to treat all data up until the EOF as the content.
//        boost::asio::streambuf request;
//        std::ostream request_stream(&request);
//        request_stream << "GET " << "/cgi/" << _cmd._to_string() << ".json" << " HTTP/1.0\r\n";
//        request_stream << "Host: " << network_sensor_ip_ << "\r\n";
//        request_stream << "Accept: */*\r\n";
//        request_stream << "Connection: close\r\n\r\n";
//        //
//        //        ROS_INFO_STREAM("Request: " << request_stream);

//        // Send the request.
//        boost::asio::write(socket, request);

//        // Read the response status line. The response streambuf will automatically
//        // grow to accommodate the entire line. The growth may be limited by passing
//        // a maximum size to the streambuf constructor.
//        boost::asio::streambuf response;
//        boost::asio::read_until(socket, response, "\r\n");

//        // Check that response is OK.
//        std::istream response_stream(&response);
//        std::string http_version;
//        response_stream >> http_version;
//        unsigned int status_code;
//        response_stream >> status_code;
//        std::string status_message;
//        std::getline(response_stream, status_message);
//        if (!response_stream || http_version.substr(0, 5) != "HTTP/")
//        {
//            ROS_WARN_STREAM("Invalid response\n");
//            return response_json;
//        }
//        if (status_code != 200)
//        {
//            ROS_WARN_STREAM("Response returned with status code " << status_code << "\n");
//            return response_json;
//        }

//        // Read the response headers, which are terminated by a blank line.
//        boost::asio::read_until(socket, response, "\r\n\r\n");

//        // Process the response headers.
//        std::string header;
//        while (std::getline(response_stream, header) && header != "\r")
//            //            ROS_INFO_STREAM("header: " << header << "\n");
//            ;
//        //        ROS_INFO_STREAM("\n");

//        // Write whatever content we already have to output.
//        if (response.size() > 0)
//        {
//            //            ROS_INFO_STREAM("response: " << &response);   // ps: ca consumme les donnees !
//            // urls:
//            // - http://stackoverflow.com/questions/1899750/how-do-i-convert-a-boostasiostreambuf-into-a-stdstring
//            // - http://stackoverflow.com/a/2546953
//            std::istream(&response) >> response_json;
//            //            ROS_INFO_STREAM("response_json: " << response_json);
//        }

//        // Read until EOF, writing data to output as we go.
//        boost::system::error_code error;
//        while (boost::asio::read(socket, response,
//                                 boost::asio::transfer_at_least(1), error))
//            //ROS_INFO_STREAM(&response);
//            ;

//        if (error != boost::asio::error::eof)
//            throw boost::system::system_error(error);
//    }
//    catch (std::exception& e)
//    {
//        ROS_WARN_STREAM("Exception: " << e.what() << "\n");
//    }
//    return response_json;
//}

///**
// * @brief VLP16_WebServer::convert_config_to_x
// * @param _config
// * @return
// */
//std::string VLP16_WebServer::convert_config_to_xwwwformcoded(const velodyne_configuration::VLP16_settingsConfig& _config) const
//{
//    const std::string result = \
//            "laser=" + std::string(_config.laser_state?"on":"off") + "&" \
//            "returns=" + LaserReturns::_from_integral(_config.return_type)._to_string() + "&" \
//            "rpm=" + std::to_string(_config.rpm);

//    return result;
//}

///**
// * @brief VLP16_WebServer::send_settings_to_webserver
// * @param _config
// * @return
// */
//int VLP16_WebServer::send_settings_to_webserver(const velodyne_configuration::VLP16_settingsConfig& _config) const
//{
//    const std::string curl_settings = convert_config_to_xwwwformcoded(_config);

//    //     From: man curl
//    //    -d, --data <data>
//    //            (HTTP)  Sends  the  specified data in a POST request to the HTTP server, in the same way that a browser does when a user has filled in an HTML
//    //            form and presses the submit button. This will cause curl to pass the data to the server using the  content-type  application/x-www-form-urlen?
//    //                coded.  Compare to -F, --form.
//    //
//    //            -d,  --data  is the same as --data-ascii. To post data purely binary, you should instead use the --data-binary option. To URL-encode the value
//    //            of a form field you may use --data-urlencode.
//    //
//    //            If any of these options is used more than once on the same command line, the data pieces specified will be merged together with  a  separating
//    //            &-symbol. Thus, using '-d name=daniel -d skill=lousy' would generate a post chunk that looks like 'name=daniel&skill=lousy'.
//    //
//    //            If  you  start  the  data with the letter @, the rest should be a file name to read the data from, or - if you want curl to read the data from
//    //            stdin. Multiple files can also be specified. Posting data from a file named 'foobar' would thus be done with --data @foobar.  When  --data  is
//    //            told to read from a file like that, carriage returns and newlines will be stripped out.

//    const std::string cmd_curl = \
//            "curl -X POST http://" + \
//            network_sensor_ip_ + \
//            "/cgi/setting --data '" + \
//            curl_settings + \
//            "'"
//            ;

//    const std::string cmd =
//            (max_delay_for_cmd_ != 0 ?                                     \
//                "timeout " + std::to_string(max_delay_for_cmd_) + "s " :   \
//                "")                                                         \
//            + cmd_curl;
//    //
//    ROS_INFO_STREAM("Commande bash: " << cmd);

//    const std::string ret_cmd = velodyne_tools::exec_cmd(cmd.c_str());

//    return 1;
//}



//bool VLP16_WebServer::get_ip(
//        const ros::NodeHandle &_n,
//        const std::string &_param_name
//        )
//{
//    bool return_get_param = _n.getParam(_param_name, network_sensor_ip_);
//    ROS_INFO_STREAM("getParam -> VLP16_NETWORK_SENSOR_IP: " << network_sensor_ip_);
//    return return_get_param;
//}

//bool VLP16_WebServer::parse_JSON_for_settings(
//        const std::string &res_request,
//        velodyne_configuration::VLP16_SettingsServiceResponse &res
//        )
//{
//    // ------------------------------------
//    // Manual JSON file parsing
//    // ------------------------------------
//    // urls:
//    // - http://zenol.fr/blog/boost-property-tree/en.html
//    // - https://gist.github.com/mloskot/1509935
//    try
//    {
//        JSON_INIT(root, res_request);

//        JSON_READ_BOOL   (root, laser,                res, laser_state);
//        JSON_READ_STRING (root, returns,              res, returns);
//        JSON_READ_UINT16 (root, rpm,                  res, rpm);
//        JSON_READ_UINT16 (root, fov.start,            res, fov_start);
//        JSON_READ_UINT16 (root, fov.end,              res, fov_end);
//        JSON_READ_STATE  (root, phaselock.enabled,    res, phaselock_enabled);
//        JSON_READ_UINT16 (root, phaselock.offset,     res, phaselock_offset);
//        JSON_READ_STRING (root, host.addr,            res, host_addr);
//        JSON_READ_UINT16 (root, host.dport,           res, host_dport);
//        JSON_READ_UINT16 (root, host.tport,           res, host_tport);
//        JSON_READ_STRING (root, net.addr,             res, net_addr);
//        JSON_READ_STRING (root, net.mask,             res, net_mask);
//        JSON_READ_STRING (root, net.gateway,          res, net_gateway);
//        JSON_READ_BOOL   (root, net.dhcp,             res, net_dhcp);

//    }
//    catch (std::exception const& e)
//    {
//        ROS_ERROR_STREAM(e.what());
//        return false;
//    }
//    // ------------------------------------

//    return true;
//}

//bool VLP16_WebServer::parse_JSON_for_status(
//        const std::string &res_request,
//        velodyne_configuration::VLP16_StatusServiceResponse &res
//        )
//{
//    // ------------------------------------
//    // Manual JSON file parsing
//    // ------------------------------------
//    // urls:
//    // - http://zenol.fr/blog/boost-property-tree/en.html
//    // - https://gist.github.com/mloskot/1509935
//    try
//    {
//        JSON_INIT(root, res_request );

//        /**
//         JSON: { ... "gps":{"pps_state":"Locked","position":"49 00.00N 200 .00W"} ... }
//         - gps
//         -- pps_state
//         -- position
//        /**/
//        JSON_READ_STRING(root, gps.pps_state,  res, gps_state);
//        JSON_READ_STRING(root, gps.position,   res, gps_position);
//        /**
//         JSON: { ... "motor":{"state":"On","rpm":0,"lock":"Off","phase":30755} ... }
//         - motor
//         -- state
//         -- rpm
//         -- lock
//         -- phase
//        /**/
//        JSON_READ_BOOL(root, motor.state, res, motor_state);
//        JSON_READ_UINT16(root, motor.rpm, res, motor_rpm);
//        JSON_READ_BOOL(root, motor.lock,  res, motor_lock);
//        JSON_READ_UINT16(root, motor.phase, res, motor_phase);
//        /**
//         JSON: { ... "laser":{"state":"Disabled"} ... }
//         - laser
//         -- state
//        /**/
//        JSON_READ_BOOL(root, laser.state, res, laser_state);
//    }
//    catch (std::exception const& e)
//    {
//        ROS_ERROR_STREAM(e.what());
//        return false;
//    }
//    // ------------------------------------
//    return true;
//}

//bool VLP16_WebServer::parse_JSON_for_diagnostics_raw(
//        const std::string &res_request,
//        velodyne_configuration::VLP16_DiagnosticsRawServiceResponse &res
//        )
//{
//    // ------------------------------------
//    // Manual JSON file parsing
//    // ------------------------------------
//    // urls:
//    // - http://zenol.fr/blog/boost-property-tree/en.html
//    // - https://gist.github.com/mloskot/1509935
//    try
//    {
//        JSON_INIT(root, res_request);

//        JSON_READ_UINT16(root, volt_temp.bot.i_out, res, bot_i_out);
//        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_2v, res, bot_pwr_1_2v);
//        JSON_READ_UINT16(root, volt_temp.bot.lm20_temp, res, bot_lm20_temp);
//        JSON_READ_UINT16(root, volt_temp.bot.pwr_5v, res, bot_pwr_5v);
//        JSON_READ_UINT16(root, volt_temp.bot.pwr_2_5v, res, bot_pwr_2_5v);
//        JSON_READ_UINT16(root, volt_temp.bot.pwr_3_3v, res, bot_pwr_3_3v);
//        JSON_READ_UINT16(root, volt_temp.bot.pwr_v_in, res, bot_pwr_v_in);
//        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_25v, res, bot_pwr_1_25v);

//        JSON_READ_UINT16(root, volt_temp.top.lm20_temp, res, top_lm20_temp);
//        JSON_READ_UINT16(root, volt_temp.top.hv, res, top_hv);
//        JSON_READ_UINT16(root, volt_temp.top.ad_temp, res, top_ad_temp);
//        JSON_READ_UINT16(root, volt_temp.top.pwr_5v, res, top_pwr_5v);
//        JSON_READ_UINT16(root, volt_temp.top.pwr_2_5v, res, top_pwr_2_5v);
//        JSON_READ_UINT16(root, volt_temp.top.pwr_3_3v, res, top_pwr_3_3v);
//        JSON_READ_UINT16(root, volt_temp.top.pwr_vccint, res, top_pwr_vccint);

//        JSON_READ_UINT16(root, vhv, res, vhv);
//        JSON_READ_UINT16(root, adc_nf, res, adc_nf);
//        JSON_READ_UINT16(root, ixe, res, ixe);
//    }
//    catch (std::exception const& e)
//    {
//        ROS_ERROR_STREAM(e.what());
//        return false;
//    }
//    // ------------------------------------

//    return true;
//}

///**
// * @brief scale_volt_temp
// *  inspired by 'js/diag.js' dump from the VLP-16 webserver
// * @param msg_raw
// * @param msg
// */
//bool VLP16_WebServer::scale_volt_temp(
//        velodyne_configuration::VLP16_DiagnosticsRawMessage &_msg_raw,
//        velodyne_configuration::VLP16_DiagnosticsMessage    &_msg
//        )
//{
//    const float scale_2x_vref = 5.0/4096;

//    try
//    {
//        //        volt_temp.top.hv = hdltop_volts_to_hv(volt_temp.top.hv);
//        //        volt_temp.top.lm20_temp = lm20_volts_to_degCel(volt_temp.top.lm20_temp);
//        //        volt_temp.top.pwr_5v *= 2.0;
//        //        volt_temp.top.pwr_5v_raw *= 2.0;
//        _msg.top_hv = hdltop_volts_to_hv(_msg_raw.top_hv * scale_2x_vref);
//        _msg.top_lm20_temp = lm20_volts_to_degCel(_msg_raw.top_lm20_temp * scale_2x_vref);
//        _msg.top_pwr_5v = _msg_raw.top_pwr_5v * 2.0 * scale_2x_vref;
//        _msg.top_pwr_5v_raw = _msg_raw.top_pwr_5v_raw * 2.0 * scale_2x_vref;
//        _msg.top_ad_temp = _msg_raw.top_ad_temp * scale_2x_vref;
//        _msg.top_pwr_2_5v = _msg_raw.top_pwr_2_5v * scale_2x_vref;
//        _msg.top_pwr_3_3v = _msg_raw.top_pwr_3_3v * scale_2x_vref;
//        _msg.top_pwr_vccint = _msg_raw.top_pwr_vccint * scale_2x_vref;

//        //        volt_temp.bot.i_out = acs17_volts_to_amps(volt_temp.bot.i_out);
//        //        volt_temp.bot.lm20_temp = lm20_volts_to_degCel(volt_temp.bot.lm20_temp);
//        //        volt_temp.bot.pwr_5v *= 2.0;
//        //        volt_temp.bot.pwr_v_in*= 11.0;
//        _msg.bot_i_out = acs17_volts_to_amps(_msg_raw.bot_i_out * scale_2x_vref);
//        _msg.bot_lm20_temp = lm20_volts_to_degCel(_msg_raw.bot_lm20_temp * scale_2x_vref);
//        _msg.bot_pwr_5v = _msg_raw.bot_pwr_5v * 2.0 * scale_2x_vref;
//        _msg.bot_pwr_v_in = _msg_raw.bot_pwr_v_in * 11.0 * scale_2x_vref;
//        _msg.bot_pwr_2_5v = _msg_raw.bot_pwr_2_5v * scale_2x_vref;
//        _msg.bot_pwr_3_3v = _msg_raw.bot_pwr_3_3v * scale_2x_vref;
//        _msg.bot_pwr_1_25v = _msg_raw.bot_pwr_1_25v * scale_2x_vref;
//        _msg.bot_pwr_1_2v = _msg_raw.bot_pwr_1_2v * scale_2x_vref;

//        _msg.vhv = _msg_raw.vhv;
//        _msg.ixe = _msg_raw.ixe;
//        _msg.adc_nf = _msg_raw.adc_nf;
//    }
//    catch (std::exception const& e)
//    {
//        ROS_ERROR_STREAM(e.what());
//    }
//}

//template<class Service, class Message>
//Velodyne_WebServer_Services<Service, Message>::Velodyne_WebServer_Services(
//        const std::string& _name,
//        const std::string& _nh,
//        const double& _loop_rate_value
//        ) :
//    nh_(_nh), loop_rate_value_(_loop_rate_value)
//{
//    //    /**webserver_.**/get_ip(nh_);
//    get_ip(nh_);

//    //----------------------------------------------------
//    // Services
//    //----------------------------------------------------
//    node_name_srv_ = "get_" + _name;
//    velodyne_service_srv_ = nh_.advertiseService(node_name_srv_, &Velodyne_WebServer_Services::get_response, this);
//    //
//    ROS_INFO("Ready to get velodyne %s.\t[SERVICE]", _name.c_str());
//    //----------------------------------------------------

//    //----------------------------------------------------
//    // Publisher
//    //----------------------------------------------------
//    topic_name_pub_ = _name + "_pub";
//    velodyne_service_pub_ = nh_.advertise<Message>(topic_name_pub_, 1000);
//    //
//    ROS_INFO("Publish %s messages.\t[PUBLISHER]", _name.c_str());
//    //----------------------------------------------------
//}

//template<class Service, class Message>
//void Velodyne_WebServer_Services<Service, Message>::run(
//        boost::function<bool()> _prePublish,
//        boost::function<void()> _postPublish
//        )
//{
//    //----------------------------------------
//    // Boucle d'exécution ROS
//    //----------------------------------------
//    ros::Rate loop_rate(loop_rate_value_);   // Hz
//    while (ros::ok())
//    {
//        if( _prePublish() ) {
//            if( get_response(laser_data_) ) {
//                velodyne_service_pub_.publish(laser_data_.msg);
//            }
//            _postPublish();
//        }

//        ros::spinOnce();

//        loop_rate.sleep();
//    }
//    //----------------------------------------
//}

//template<class Service, class Message>
//void Velodyne_WebServer_Services<Service, Message>::run_with_test_sub(
//        boost::function<void()> _postPublish
//        )
//{
//    run( boost::function<bool()>( [this](){ return velodyne_service_pub_.getNumSubscribers() != 0; }, _postPublish ) );
//}


////----------------------------------------------------------------------------------------------------------------------------------

////---------------------------------------------------------------------------
//Velodyne_WebServer_Status::Velodyne_WebServer_Status(
//        const std::string& _name,
//        const std::string& _nh,
//        const double& _loop_rate_value
//        ) :
//    Velodyne_WebServer_Services(_name, _nh, _loop_rate_value)
//{
//}

//bool Velodyne_WebServer_Status::get_response(velodyne_configuration::VLP16_StatusServiceResponse &_res)
//{
//    //    const std::string res_request = /**webserver_.**/request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::status);
////    const std::string res_request = request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::status);
//    const std::string res_request = request_webserver(velodyne_webserver::Velodyne_WebServer::WebServerCommands::status);
//    ROS_INFO_STREAM("response from VLP webserver: " << res_request );

//    return /**webserver_.**/parse_JSON_for_status(res_request, _res);
//}

//void Velodyne_WebServer_Status::run()
//{
//    // url: http://fr.cppreference.com/w/cpp/language/lambda
//    Velodyne_WebServer_Services<velodyne_configuration::VLP16_StatusService, velodyne_configuration::VLP16_StatusMessage>::run(
//                boost::function<bool()>( [this](){ return velodyne_service_pub_.getNumSubscribers() != 0; } )
//                );
//}
////---------------------------------------------------------------------------

////---------------------------------------------------------------------------
//Velodyne_WebServer_Settings::Velodyne_WebServer_Settings(const std::string& _name) : Velodyne_WebServer_Services(_name )
//{
//}

//bool Velodyne_WebServer_Settings::get_response(velodyne_configuration::VLP16_SettingsServiceResponse &_res)
//{
////    const std::string res_request = /**webserver_.**/request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::settings);
//    const std::string res_request = request_webserver(velodyne_webserver::Velodyne_WebServer::WebServerCommands::settings);
//    ROS_INFO_STREAM("response from VLP webserver: " << res_request );

//    return /**webserver_.**/parse_JSON_for_settings(res_request, _res);
//}
////---------------------------------------------------------------------------

////---------------------------------------------------------------------------
//Velodyne_WebServer_Diagnostics::Velodyne_WebServer_Diagnostics(const std::string& _name) : Velodyne_WebServer_Services(_name )
//{
//}

//void Velodyne_WebServer_Diagnostics::run()
//{
////    Velodyne_WebServer_Services<VLP16_DiagnosticsService, VLP16_DiagnosticsMessage>::run_with_test_sub();
//    Velodyne_WebServer_Services<VLP16_DiagnosticsService, VLP16_DiagnosticsMessage>::run_with_test_sub();
//}

//bool Velodyne_WebServer_Diagnostics::get_diagnostics_raw(velodyne_configuration::VLP16_DiagnosticsRawServiceResponse& _res)
//{
////    const std::string res_request = /**webserver_.**/request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::diag);
//    const std::string res_request = /**webserver_.**/request_webserver(velodyne_webserver::Velodyne_WebServer::WebServerCommands::diag);
//    ROS_INFO_STREAM("response from VLP webserver: " << res_request );
//    return /**webserver_.**/parse_JSON_for_diagnostics_raw(res_request, _res);
//}

//bool Velodyne_WebServer_Diagnostics::get_response(velodyne_configuration::VLP16_DiagnosticsServiceResponse& _res)
//{
//    velodyne_configuration::VLP16_DiagnosticsRawServiceResponse res_raw;
//    get_diagnostics_raw(res_raw);
//    return /**webserver_.**/scale_volt_temp(res_raw.msg, _res.msg);
//}
////---------------------------------------------------------------------------

}
