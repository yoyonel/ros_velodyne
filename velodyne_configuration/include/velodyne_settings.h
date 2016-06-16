#ifndef VELODYNE_SETTINGS_H
#define VELODYNE_SETTINGS_H

#include <dynamic_reconfigure/server.h>
//
#include "velodyne_configuration/VLP16_settingsConfig.h"
#include "velodyne_configuration/VLP16_SettingsService.h"
//
#include <velodyne_tools.h>
//
#include <boost/thread.hpp>


/**
 * @brief The CVelodyneSettings class
 */
class CVelodyneSettings
{
public:
    CVelodyneSettings();
    void run();

protected:
    bool get_settings(velodyne_configuration::VLP16_SettingsServiceRequest  &req,
                      velodyne_configuration::VLP16_SettingsServiceResponse &res);
    bool get_settings(velodyne_configuration::VLP16_SettingsServiceResponse &res);
    void callback(velodyne_configuration::VLP16_settingsConfig &config, uint32_t level);
    void velodyne_settingsCallback(velodyne_configuration::VLP16_SettingsMessage _msg);
    bool synch_Laser_ROS();
    bool synch_Laser_ROS(const velodyne_configuration::VLP16_SettingsMessage& _msg);

    inline void update_config(const velodyne_configuration::VLP16_SettingsMessage& _msg);
    inline bool state_has_changed(const velodyne_configuration::VLP16_SettingsMessage& _msg) const;

private:
    //! Private NodeHandle
    ros::NodeHandle nh_;

    double loop_rate_value_;

    velodyne_tools::VLP16_WebServer webserver_;

    bool bAuthSyncROStoLaser_;

    ros::ServiceServer service_;

    // url: http://answers.ros.org/question/10709/assertion-pthread_mutex_lockm-failed-runtime-error-while-working-with-custom-message-and-kinect/
    // => l'ordre de déclaration entre le mutex et le dynamic_reconfigure_server est important !
    // url: http://www.boost.org/doc/libs/1_58_0/doc/html/thread/synchronization.html#thread.synchronization.mutex_types.recursive_mutex
    boost::recursive_mutex dynamic_reconfigure_mutex_;
    dynamic_reconfigure::Server<velodyne_configuration::VLP16_settingsConfig> server_;

    velodyne_configuration::VLP16_settingsConfig config_;

    ros::Publisher velodyne_settings_pub_;

    velodyne_configuration::VLP16_SettingsServiceResponse laser_settings_;

    ros::Subscriber velodyne_settings_sub_;
};

#endif // VELODYNE_SETTINGS_H
