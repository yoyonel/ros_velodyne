#ifndef __VLP_WEBCLIENT_H
#define __VLP_WEBCLIENT_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "enum.h"

namespace velodyne_tools
{

namespace pt = boost::property_tree;

const std::string str_IP_ADRESS_WEBSERVER_LIDAR = "192.168.1.201";
const float max_delay_for_cmd = 0.03f;

// -------------------------------------------
// url: http://stackoverflow.com/a/13188585
// -------------------------------------------
//#define stringify( name ) # name

//enum WebServerCommands {
//    settings,
//    status,
//    info,
//    diag
//};
// -------------------------------------------

// url: http://aantron.github.io/better-enums/
// usage: https://raw.githubusercontent.com/aantron/better-enums/master/doc/image/sample.gif
BETTER_ENUM( WebServerCommands, char,
             settings=1,
             status,
             info,
             diag
             )

std::string exec_cmd(const char* cmd);

std::string request_webserver(
        std::string _ip_laser,
        WebServerCommands _cmd,
        const float& _max_delay_for_cmd = 0
        );

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


} // namespace velodyne_settings

#endif
