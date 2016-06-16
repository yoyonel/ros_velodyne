#include <ros/ros.h>
#include <velodyne_tools.h>
#include <string>


namespace velodyne_tools
{

/**
 * @brief VLP16_WebServer::exec_cmd
 * @param cmd
 * @return
 */
std::string Velodyne_WebServer::exec_cmd(const char* cmd) const
{
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}


/**
 * @brief VLP16_WebServer::request_webserver
 * @param _str_ip_laser
 * @param _cmd
 * @param _max_delay_for_cmd
 * @return
 */
std::string VLP16_WebServer::request_webserver(const WebServerCommands& _cmd) const
{
    // url: http://stackoverflow.com/questions/23936246/error-invalid-operands-of-types-const-char-35-and-const-char-2-to-binar
    const std::string cmd_curl =                    \
            "curl -s http://" +                     \
            m_network_sensor_ip +                   \
            "/cgi/" + _cmd._to_string() + ".json";

    const std::string cmd =
            (m_max_delay_for_cmd != 0 ?                                     \
                "timeout " + std::to_string(m_max_delay_for_cmd) + "s " :   \
                "")                                                         \
            + cmd_curl;
    //
    ROS_INFO_STREAM("Commande bash: " << cmd);

    return exec_cmd(cmd.c_str());
}

/**
 * @brief VLP16_WebServer::convert_config_to_x
 * @param _config
 * @return
 */
std::string VLP16_WebServer::convert_config_to_xwwwformcoded(const velodyne_configuration::VLP16_settingsConfig& _config) const
{
    const std::string result = \
            "laser=" + std::string(_config.laser_state?"on":"off") + "&" \
            "returns=" + LaserReturns::_from_integral(_config.return_type)._to_string() + "&" \
            "rpm=" + std::to_string(_config.rpm);

    return result;
}

/**
 * @brief VLP16_WebServer::send_settings_to_webserver
 * @param _config
 * @return
 */
int VLP16_WebServer::send_settings_to_webserver(const velodyne_configuration::VLP16_settingsConfig& _config) const
{
    const std::string curl_settings = convert_config_to_xwwwformcoded(_config);

//     From: man curl
//    -d, --data <data>
//            (HTTP)  Sends  the  specified data in a POST request to the HTTP server, in the same way that a browser does when a user has filled in an HTML
//            form and presses the submit button. This will cause curl to pass the data to the server using the  content-type  application/x-www-form-urlen?
//                coded.  Compare to -F, --form.
//
//            -d,  --data  is the same as --data-ascii. To post data purely binary, you should instead use the --data-binary option. To URL-encode the value
//            of a form field you may use --data-urlencode.
//
//            If any of these options is used more than once on the same command line, the data pieces specified will be merged together with  a  separating
//            &-symbol. Thus, using '-d name=daniel -d skill=lousy' would generate a post chunk that looks like 'name=daniel&skill=lousy'.
//
//            If  you  start  the  data with the letter @, the rest should be a file name to read the data from, or - if you want curl to read the data from
//            stdin. Multiple files can also be specified. Posting data from a file named 'foobar' would thus be done with --data @foobar.  When  --data  is
//            told to read from a file like that, carriage returns and newlines will be stripped out.

    const std::string cmd_curl = \
            "curl -X POST http://" + \
            m_network_sensor_ip + \
            "/cgi/setting --data '" + \
            curl_settings + \
            "'"
            ;

    const std::string cmd =
            (m_max_delay_for_cmd != 0 ?                                     \
                "timeout " + std::to_string(m_max_delay_for_cmd) + "s " :   \
                "")                                                         \
            + cmd_curl;
    //
    ROS_INFO_STREAM("Commande bash: " << cmd);

    const std::string ret_cmd = exec_cmd(cmd.c_str());

    return 1;
}



bool VLP16_WebServer::get_ip(
        const ros::NodeHandle &_n,
        const std::string &_param_name
        )
{
    bool return_get_param = _n.getParam(_param_name, m_network_sensor_ip);
    ROS_INFO_STREAM("getParam -> VLP16_NETWORK_SENSOR_IP: " << m_network_sensor_ip);
    return return_get_param;
}

bool VLP16_WebServer::parse_JSON_for_settings(
        const std::string &res_request,
        velodyne_configuration::VLP16_SettingsServiceResponse &res
        )
{
    // ------------------------------------
    // Manual JSON file parsing
    // ------------------------------------
    // urls:
    // - http://zenol.fr/blog/boost-property-tree/en.html
    // - https://gist.github.com/mloskot/1509935
    try
    {
        JSON_INIT(root, res_request);

        JSON_READ_BOOL   (root, laser,                res, laser_state);
        JSON_READ_STRING (root, returns,              res, returns);
        JSON_READ_UINT16 (root, rpm,                  res, rpm);
        JSON_READ_UINT16 (root, fov.start,            res, fov_start);
        JSON_READ_UINT16 (root, fov.end,              res, fov_end);
        JSON_READ_STATE  (root, phaselock.enabled,    res, phaselock_enabled);
        JSON_READ_UINT16 (root, phaselock.offset,     res, phaselock_offset);
        JSON_READ_STRING (root, host.addr,            res, host_addr);
        JSON_READ_UINT16 (root, host.dport,           res, host_dport);
        JSON_READ_UINT16 (root, host.tport,           res, host_tport);
        JSON_READ_STRING (root, net.addr,             res, net_addr);
        JSON_READ_STRING (root, net.mask,             res, net_mask);
        JSON_READ_STRING (root, net.gateway,          res, net_gateway);
        JSON_READ_BOOL   (root, net.dhcp,             res, net_dhcp);

    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    // ------------------------------------

    return true;
}

bool VLP16_WebServer::parse_JSON_for_status(
        const std::string &res_request,
        velodyne_configuration::VLP16_StatusServiceResponse &res
        )
{
    // ------------------------------------
    // Manual JSON file parsing
    // ------------------------------------
    // urls:
    // - http://zenol.fr/blog/boost-property-tree/en.html
    // - https://gist.github.com/mloskot/1509935
    try
    {
        JSON_INIT(root, res_request );

        /**
         JSON: { ... "gps":{"pps_state":"Locked","position":"49 00.00N 200 .00W"} ... }
         - gps
         -- pps_state
         -- position
        /**/
        JSON_READ_STRING(root, gps.pps_state,  res, gps_state);
        JSON_READ_STRING(root, gps.position,   res, gps_position);
        /**
         JSON: { ... "motor":{"state":"On","rpm":0,"lock":"Off","phase":30755} ... }
         - motor
         -- state
         -- rpm
         -- lock
         -- phase
        /**/
        JSON_READ_BOOL(root, motor.state, res, motor_state);
        JSON_READ_UINT16(root, motor.rpm, res, motor_rpm);
        JSON_READ_BOOL(root, motor.lock,  res, motor_lock);
        JSON_READ_UINT16(root, motor.phase, res, motor_phase);
        /**
         JSON: { ... "laser":{"state":"Disabled"} ... }
         - laser
         -- state
        /**/
        JSON_READ_BOOL(root, laser.state, res, laser_state);
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    // ------------------------------------
    return true;
}

bool VLP16_WebServer::parse_JSON_for_diagnostics_raw(
        const std::string &res_request,
        velodyne_configuration::VLP16_DiagnosticsRawServiceResponse &res
        )
{
    // ------------------------------------
    // Manual JSON file parsing
    // ------------------------------------
    // urls:
    // - http://zenol.fr/blog/boost-property-tree/en.html
    // - https://gist.github.com/mloskot/1509935
    try
    {
        JSON_INIT(root, res_request);

        JSON_READ_UINT16(root, volt_temp.bot.i_out, res, bot_i_out);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_2v, res, bot_pwr_1_2v);
        JSON_READ_UINT16(root, volt_temp.bot.lm20_temp, res, bot_lm20_temp);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_5v, res, bot_pwr_5v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_2_5v, res, bot_pwr_2_5v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_3_3v, res, bot_pwr_3_3v);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_v_in, res, bot_pwr_v_in);
        JSON_READ_UINT16(root, volt_temp.bot.pwr_1_25v, res, bot_pwr_1_25v);

        JSON_READ_UINT16(root, volt_temp.top.lm20_temp, res, top_lm20_temp);
        JSON_READ_UINT16(root, volt_temp.top.hv, res, top_hv);
        JSON_READ_UINT16(root, volt_temp.top.ad_temp, res, top_ad_temp);
        JSON_READ_UINT16(root, volt_temp.top.pwr_5v, res, top_pwr_5v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_2_5v, res, top_pwr_2_5v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_3_3v, res, top_pwr_3_3v);
        JSON_READ_UINT16(root, volt_temp.top.pwr_vccint, res, top_pwr_vccint);

        JSON_READ_UINT16(root, vhv, res, vhv);
        JSON_READ_UINT16(root, adc_nf, res, adc_nf);
        JSON_READ_UINT16(root, ixe, res, ixe);
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    // ------------------------------------

    return true;
}

/**
 * @brief scale_volt_temp
 *  inspired by 'js/diag.js' dump from the VLP-16 webserver
 * @param msg_raw
 * @param msg
 */
bool VLP16_WebServer::scale_volt_temp(
        velodyne_configuration::VLP16_DiagnosticsRawMessage &_msg_raw,
        velodyne_configuration::VLP16_DiagnosticsMessage    &_msg
        )
{
    const float scale_2x_vref = 5.0/4096;

    try
    {
        //        volt_temp.top.hv = hdltop_volts_to_hv(volt_temp.top.hv);
        //        volt_temp.top.lm20_temp = lm20_volts_to_degCel(volt_temp.top.lm20_temp);
        //        volt_temp.top.pwr_5v *= 2.0;
        //        volt_temp.top.pwr_5v_raw *= 2.0;
        _msg.top_hv = hdltop_volts_to_hv(_msg_raw.top_hv * scale_2x_vref);
        _msg.top_lm20_temp = lm20_volts_to_degCel(_msg_raw.top_lm20_temp * scale_2x_vref);
        _msg.top_pwr_5v = _msg_raw.top_pwr_5v * 2.0 * scale_2x_vref;
        _msg.top_pwr_5v_raw = _msg_raw.top_pwr_5v_raw * 2.0 * scale_2x_vref;
        _msg.top_ad_temp = _msg_raw.top_ad_temp * scale_2x_vref;
        _msg.top_pwr_2_5v = _msg_raw.top_pwr_2_5v * scale_2x_vref;
        _msg.top_pwr_3_3v = _msg_raw.top_pwr_3_3v * scale_2x_vref;
        _msg.top_pwr_vccint = _msg_raw.top_pwr_vccint * scale_2x_vref;

        //        volt_temp.bot.i_out = acs17_volts_to_amps(volt_temp.bot.i_out);
        //        volt_temp.bot.lm20_temp = lm20_volts_to_degCel(volt_temp.bot.lm20_temp);
        //        volt_temp.bot.pwr_5v *= 2.0;
        //        volt_temp.bot.pwr_v_in*= 11.0;
        _msg.bot_i_out = acs17_volts_to_amps(_msg_raw.bot_i_out * scale_2x_vref);
        _msg.bot_lm20_temp = lm20_volts_to_degCel(_msg_raw.bot_lm20_temp * scale_2x_vref);
        _msg.bot_pwr_5v = _msg_raw.bot_pwr_5v * 2.0 * scale_2x_vref;
        _msg.bot_pwr_v_in = _msg_raw.bot_pwr_v_in * 11.0 * scale_2x_vref;
        _msg.bot_pwr_2_5v = _msg_raw.bot_pwr_2_5v * scale_2x_vref;
        _msg.bot_pwr_3_3v = _msg_raw.bot_pwr_3_3v * scale_2x_vref;
        _msg.bot_pwr_1_25v = _msg_raw.bot_pwr_1_25v * scale_2x_vref;
        _msg.bot_pwr_1_2v = _msg_raw.bot_pwr_1_2v * scale_2x_vref;

        _msg.vhv = _msg_raw.vhv;
        _msg.ixe = _msg_raw.ixe;
        _msg.adc_nf = _msg_raw.adc_nf;
    }
    catch (std::exception const& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

} // namespace velodyne_settings
