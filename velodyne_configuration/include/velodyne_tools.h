#ifndef __VLP_WEBCLIENT_H
#define __VLP_WEBCLIENT_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "enum.h"

#include <ros/ros.h>

#include "velodyne_configuration/VLP16_SettingsService.h"
#include "velodyne_configuration/VLP16_StatusService.h"
#include "velodyne_configuration/VLP16_DiagnosticsService.h"
#include "velodyne_configuration/VLP16_DiagnosticsRawService.h"
#include "velodyne_configuration/VLP16_settingsConfig.h"

namespace pt = boost::property_tree;


namespace velodyne_tools
{

// -------------------------------------------
// url: http://stackoverflow.com/a/13188585
// -------------------------------------------
#define stringify( name ) # name

BETTER_ENUM( _WebServerCommands, char,
             settings=1,
             status,
             info,
             diag
             )

BETTER_ENUM( _LaserReturns, char,
             Strongest=0,
             Last,
             Dual
        )

#define hdltop_volts_to_hv(volts)   \
    101.0 * (volts - 5.0)

#define lm20_volts_to_degCel(volts) \
    -1481.96 + sqrt(2.1962E6 + (1.8639 - volts)/3.88E-6)

#define acs17_volts_to_amps(volts)  \
    10.0 * (volts - 2.5);


class Velodyne_WebServer
{
    public:
    //    enum WebServerCommands {
    //        settings,
    //        status,
    //        info,
    //        diag
    //    };
    // -------------------------------------------

    // url: http://aantron.github.io/better-enums/
    // usage: https://raw.githubusercontent.com/aantron/better-enums/master/doc/image/sample.gif
    typedef _WebServerCommands WebServerCommands;
    typedef _LaserReturns LaserReturns;


    public:
    inline void set_network_sensor_ip(const std::string &_addr) { m_network_sensor_ip = _addr; }

    virtual std::string request_webserver(const WebServerCommands &_cmd) const = 0;

    protected:
    std::string exec_cmd(const char* cmd) const;


    protected:
    std::string m_network_sensor_ip;
    float m_max_delay_for_cmd;
};

class VLP16_WebServer: Velodyne_WebServer
{
public:
    // url: http://stackoverflow.com/questions/2290733/initialize-parents-protected-members-with-initialization-list-c
    VLP16_WebServer()
    {
        m_network_sensor_ip = "192.168.1.201";
        m_max_delay_for_cmd = 0.03f;
    }

    std::string request_webserver(const WebServerCommands &_cmd) const;
    int send_settings_to_webserver(const velodyne_configuration::VLP16_settingsConfig& _config) const;
    std::string convert_config_to_xwwwformcoded(const velodyne_configuration::VLP16_settingsConfig& _config) const;

    bool get_ip(const ros::NodeHandle &_n, const std::string &_param_name="VLP16_NETWORK_SENSOR_IP");

    //
    bool parse_JSON_for_settings(const std::string &res_request, velodyne_configuration::VLP16_SettingsServiceResponse &res);
    bool parse_JSON_for_status(const std::string &res_request, velodyne_configuration::VLP16_StatusServiceResponse &res);
    bool parse_JSON_for_diagnostics_raw(const std::string &res_request, velodyne_configuration::VLP16_DiagnosticsRawServiceResponse &res);

    bool scale_volt_temp(velodyne_configuration::VLP16_DiagnosticsRawMessage &_msg_raw,velodyne_configuration::VLP16_DiagnosticsMessage &_msg);
};

// url: http://www.alexonlinux.com/gcc-macro-language-extensions
#define JSON_INIT(_root, _input)                                \
    std::stringstream _input ## _stream;                        \
    _input ## _stream << _input;                                \
    pt::ptree _root;                                            \
    boost::property_tree::read_json(_input ## _stream, _root);

#define JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, _type)   \
    _ros_res.msg._ros_msg_child = _json_root.get < _type > ( #_json_child )

#define JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, std::string)

#define JSON_READ_UINT16(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, uint16_t)

#define JSON_READ_UINT8(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, uint8_t)

#define JSON_READ_BOOL(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child) == "On"

#define JSON_READ_STATE(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child) == "Enabled"

#define JSON_READ_STRING(_json_root, _json_child, _ros_res, _ros_msg_child)   \
    JSON_READ(_json_root, _json_child, _ros_res, _ros_msg_child, std::string)

} // namespace velodyne_tools

#endif
