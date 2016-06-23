#ifndef VELODYNE_WEBSERVER_VLP16_H
#define VELODYNE_WEBSERVER_VLP16_H

#include <velodyne_webserver.h>
#include <boost/assign.hpp>

namespace vlp16_webserver {

using namespace velodyne_configuration;
using namespace velodyne_webserver;

#define DEFAULT_WEBSERVER_CONNECTION_TYPE   WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS


class VLP16_WebServer: public Velodyne_WebServer
{
public:
    // url: http://stackoverflow.com/questions/2290733/initialize-parents-protected-members-with-initialization-list-c
    VLP16_WebServer();

    std::string request_webserver(
            const WebServerCommands &_cmd,
            WebServerConnectionType _typeConnection = DEFAULT_WEBSERVER_CONNECTION_TYPE
            ) const override;

    int send_settings_to_webserver(const VLP16_settingsConfig& _config) const;
    std::string convert_config_to_xwwwformcoded(const VLP16_settingsConfig& _config) const;

    bool get_ip(const ros::NodeHandle &_n, const std::string &_param_name="VLP16_NETWORK_SENSOR_IP");

    //
    bool parse_JSON_for_settings(const std::string &res_request, VLP16_SettingsServiceResponse &res);
    bool parse_JSON_for_status(const std::string &res_request, VLP16_StatusServiceResponse &res);
    bool parse_JSON_for_diagnostics_raw(const std::string &res_request, VLP16_DiagnosticsRawServiceResponse &res);

    /**
     * @brief scale_volt_temp
     *  inspired by 'js/diag.js' dump from the VLP-16 webserver
     * @param msg_raw
     * @param msg
     */
    bool scale_volt_temp(VLP16_DiagnosticsRawMessage &_msg_raw, VLP16_DiagnosticsMessage &_msg);

protected:
    std::string request_webserver_curl(const WebServerCommands& _cmd) const;
    std::string request_webserver_asio_synch(const WebServerCommands& _cmd) const;
    std::string request_webserver_asio_asynch(const WebServerCommands& _cmd) const;

private:
    typedef boost::function<std::string(const WebServerCommands&)> fun_t;
    const std::map<WebServerConnectionType, fun_t> map_WSCT_FuncRequest_ = boost::assign::map_list_of
            ( WebServerConnectionType::BOOST_ASIO_ASYNCHRONOUS, boost::bind(&VLP16_WebServer::request_webserver_asio_asynch,    this, _1))
            ( WebServerConnectionType::BOOST_ASIO_SYNCHRONOUS,  boost::bind(&VLP16_WebServer::request_webserver_asio_synch,     this, _1))
            ( WebServerConnectionType::CURL,                    boost::bind(&VLP16_WebServer::request_webserver_curl,           this, _1))
            ;
};

}

#endif // VELODYNE_WEBSERVER_VLP16_H
