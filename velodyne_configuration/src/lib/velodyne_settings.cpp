#include <velodyne_settings.h>


CVelodyneSettings::CVelodyneSettings() :
    nh_("~"), loop_rate_value_(1), server_(dynamic_reconfigure_mutex_) {

    webserver_.get_ip(nh_);

    //
    synch_Laser_ROS();
    //
//    bAuthSyncROStoLaser_ = true;
    //

    //----------------------------------------------------
    // Dynamic Parameter server
    //----------------------------------------------------
    server_.setCallback(
                boost::bind(&CVelodyneSettings::callback, this, _1, _2)
                );
    server_.getConfigDefault(config_);
    //
    ROS_INFO("Ready to set velodyne settings.\t[DYNAMIC_RECONF]");
    //----------------------------------------------------

    //----------------------------------------------------
    // Services
    //----------------------------------------------------
    service_ = nh_.advertiseService("get_settings", &CVelodyneSettings::get_settings, this);
    //
    ROS_INFO("Ready to get velodyne settings.\t[SERVICE]");
    //----------------------------------------------------

    //----------------------------------------------------
    // Publisher
    //----------------------------------------------------
    velodyne_settings_pub_ = nh_.advertise<velodyne_configuration::VLP16_SettingsMessage>("velodyne_settings", 10);
    //
    ROS_INFO("Publish velodyne settings messages.\t[PUBLISHER]");
    //----------------------------------------------------

    //----------------------------------------------------
    // Subscriber
    //----------------------------------------------------
    velodyne_settings_sub_ = nh_.subscribe("velodyne_settings", 10,
                                           &CVelodyneSettings::velodyne_settingsCallback,
                                           (CVelodyneSettings *) this
                                           );
    ROS_INFO("Subscribe to velodyne settings messages.\t[SUBSCRIBER]");
    //----------------------------------------------------
}

void CVelodyneSettings::run() {
    //----------------------------------------
    // Boucle d'exécution ROS
    //----------------------------------------
    ros::Rate loop_rate(loop_rate_value_);   // 1 Hz
    while (ros::ok())
    {
        if( get_settings(laser_settings_) ) {
            velodyne_settings_pub_.publish(laser_settings_.msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }
    //----------------------------------------
}

bool CVelodyneSettings::get_settings(velodyne_configuration::VLP16_SettingsServiceRequest  &req,
                                     velodyne_configuration::VLP16_SettingsServiceResponse &res)
{
    return get_settings(res);
}

bool CVelodyneSettings::get_settings(velodyne_configuration::VLP16_SettingsServiceResponse &res)
{
    const std::string res_request = webserver_.request_webserver(velodyne_tools::Velodyne_WebServer::WebServerCommands::settings);
    ROS_INFO_STREAM("Response from VLP webserver: " << res_request );

    return webserver_.parse_JSON_for_settings(res_request, res);
}

void CVelodyneSettings::callback(velodyne_configuration::VLP16_settingsConfig &config, uint32_t level) {
    const int return_set_configs = webserver_.send_settings_to_webserver(config);
    ROS_INFO("Reconfigure Request: %d", return_set_configs);
}

void CVelodyneSettings::velodyne_settingsCallback(velodyne_configuration::VLP16_SettingsMessage _msg)
{
    ROS_INFO("velodyne_settingsCallback");
    synch_Laser_ROS(_msg);
}

bool CVelodyneSettings::state_has_changed(const velodyne_configuration::VLP16_SettingsMessage& _msg) const
{
    return
            config_.laser_state != _msg.laser_state ||
            config_.rpm != _msg.rpm ||
            config_.return_type != atoi(_msg.returns.c_str());
}

void CVelodyneSettings::update_config(const velodyne_configuration::VLP16_SettingsMessage& _msg)
{
    config_.laser_state = _msg.laser_state;
    config_.rpm = _msg.rpm;
    config_.return_type = atoi(_msg.returns.c_str());
}

bool CVelodyneSettings::synch_Laser_ROS(const velodyne_configuration::VLP16_SettingsMessage& _msg)
{
    bool ret = true;

    // On verifie qu'il y a eu un changement entre l'état du laser reporté dans le message
    // et l'état du laser stocké localement dans la config.
    if( state_has_changed(_msg) ) {

        //------------------------------------------
        // MAJ de la config (local) du laser
        //------------------------------------------
        update_config(_msg);

        //------------------------------------------
        // MAJ du Dynamic Parameter Server
        //------------------------------------------
        // urls:
        // - http://answers.ros.org/question/57498/notify-changes-to-dynamic_reconfigure/
        // - http://www.boost.org/doc/libs/1_61_0/libs/thread/example/recursive_mutex.cpp
        {
            boost::recursive_mutex::scoped_lock dyn_reconf_lock(dynamic_reconfigure_mutex_);
            server_.updateConfig(config_);
        }
        //------------------------------------------
    }

    return ret;
}

bool CVelodyneSettings::synch_Laser_ROS()
{
    velodyne_configuration::VLP16_SettingsServiceResponse laser_settings;
    // On effectue une requete au webserver pour récupérer l'état du laser
    if( get_settings(laser_settings) ) {
        // Si on a récupéré l'état,
        // on met à jour (si besoin) l'enregistrement local (ROS) de l'état du laser
        return synch_Laser_ROS(laser_settings.msg);
    }
    else
    {
        ROS_ERROR_STREAM("Can't get settings from VLP-16 webserver !");
        return false;
    }
}
