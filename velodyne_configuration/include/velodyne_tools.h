#ifndef __VLP_WEBCLIENT_H
#define __VLP_WEBCLIENT_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "enum.h"

namespace velodyne_tools
{

namespace pt = boost::property_tree;

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

} // namespace velodyne_settings

#endif
