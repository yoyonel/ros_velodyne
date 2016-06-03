#ifndef __VLP_WEBCLIENT_H
#define __VLP_WEBCLIENT_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace velodyne_settings
{
    namespace pt = boost::property_tree;

    std::string exec(const char* cmd);
} // namespace velodyne_settings

#endif
