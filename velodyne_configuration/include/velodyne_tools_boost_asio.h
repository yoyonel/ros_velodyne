#ifndef VELODYNE_TOOLS_BOOST_ASIO_H
#define VELODYNE_TOOLS_BOOST_ASIO_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

namespace velodyne_tools {
namespace boost_asio {

using boost::asio::ip::tcp;

class client_synch
{
public:
    /**
     * @brief client_synch
     * @param io_service
     * @param server
     * @param path
     */
    client_synch(boost::asio::io_service& io_service, const std::string& server, const std::string& path);

    /**
     * @brief get_response
     * @return
     */
    inline const std::string& get_response() const { return str_response_; }

protected:
    /**
     * @brief handle_request
     * @param server
     * @param path
     * @return
     */
    int handle_request(const std::string& server, const std::string& path);

private:
    std::string str_response_;
    tcp::resolver resolver_;
    tcp::socket socket_;
};

// url: http://www.boost.org/doc/libs/1_49_0/doc/html/boost_asio/example/http/client/async_client.cpp
class client_asynch
{
public:
    /**
     * @brief client_asynch
     * @param io_service
     * @param server
     * @param path
     * @param http_resquet
     */
    client_asynch(
            boost::asio::io_service& io_service,
            const std::string& server,
            const std::string& path
            );

    /**
     * @brief client_asynch
     * @param io_service
     * @param server
     * @param path
     * @param xwwwformcoded
     *
     * url: http://stackoverflow.com/questions/36141746/boost-asio-http-client-post
     */
    client_asynch(
            boost::asio::io_service& io_service,
            const std::string& server,
            const std::string& path,
            const std::string& xwwwformcoded
            );

    /**
     * @brief get_response
     * @return
     */
    const std::string& get_response() const { return str_response_; }

protected:
    /**
     * @brief handle_resolve
     * @param err
     * @param endpoint_iterator
     */
    void handle_resolve(const boost::system::error_code& err, tcp::resolver::iterator endpoint_iterator);

    /**
     * @brief handle_connect
     * @param err
     */
    void handle_connect(const boost::system::error_code& err);

    /**
     * @brief handle_write_request
     * @param err
     */
    void handle_write_request(const boost::system::error_code& err);

    /**
     * @brief handle_read_status_line
     * @param err
     */
    void handle_read_status_line(const boost::system::error_code& err);

    /**
     * @brief handle_read_headers
     * @param err
     */
    void handle_read_headers(const boost::system::error_code& err);

    /**
     * @brief handle_read_content
     * @param err
     */
    void handle_read_content(const boost::system::error_code& err);

private:
    tcp::resolver resolver_;
    tcp::socket socket_;
    boost::asio::streambuf request_;
    boost::asio::streambuf response_;
    std::string str_response_;
};

} // namespace boost_asio
} // namespace velodyne_tools

#endif // VELODYNE_TOOLS_BOOST_ASIO_H
