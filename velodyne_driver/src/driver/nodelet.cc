/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver.h"

//#include "velodyne_configuration/VLP16_StatusMessage.h" // Utile pour: velodyne_configuration::VLP16_StatusMessage
#include <velodyne_configuration/VLP16_StatusMessage.h>

namespace velodyne_driver
{

class DriverNodelet: public nodelet::Nodelet
{
public:

    DriverNodelet():
        running_(false)
    {}

    ~DriverNodelet()
    {
        if (running_)
        {
            NODELET_INFO("shutting down driver thread");
            running_ = false;
            deviceThread_->join();
            NODELET_INFO("driver thread stopped");
        }
    }

private:

    virtual void onInit(void);
    virtual void devicePoll(void);

    void cbVelodyneStatus(velodyne_configuration::VLP16_StatusMessage _msg);

    volatile bool running_;               ///< device thread is running
    boost::shared_ptr<boost::thread> deviceThread_;

    boost::shared_ptr<VelodyneDriver> dvr_; ///< driver implementation class

    ros::Subscriber velodyne_status_sub_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Lock pour protéger la mise à jour du paramètre RPM du Laser
    // Ce lock est utilisé dans le poll avec le driver (utilisation de la valeur)
    // et dans la fonction callback rattachée au call back du server status du velodyne (écriture de la valeur)
    boost::mutex lock_;
};

void DriverNodelet::onInit()
{
    nh_ = getNodeHandle();
    private_nh_ = getPrivateNodeHandle();

    // start the driver
    dvr_.reset(new VelodyneDriver(nh_, private_nh_));

    // spawn device poll thread
    running_ = true;
    deviceThread_ = boost::shared_ptr< boost::thread >
            (new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));

    //----------------------------------------------------
    // Subscriber
    //----------------------------------------------------
    // Abonnement au publisher "/velodyne_status_server/status_pub" qui renvoit le statut du laser dont le RPM.
    // On rattache cet abonnement à un callback 'DriverNodelet::cbVelodyneStatus' de mise à jour de ce setting RPM
    velodyne_status_sub_ = nh_.subscribe(
                "/velodyne_status_server/status_pub", 10,
                &DriverNodelet::cbVelodyneStatus,
                (DriverNodelet *) this
                );
    ROS_INFO("Subscribe to velodyne settings messages.\t[SUBSCRIBER]");
    //----------------------------------------------------

}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
    while(ros::ok())
    {
        // poll device until end of file
        {
            // On lock car on utilise 'config_' dans le driver
            boost::mutex::scoped_lock lock(lock_);
            running_ = dvr_->poll();
        }
        // fin du lock

        if (!running_)
            break;
    }
    running_ = false;
}

void DriverNodelet::cbVelodyneStatus(velodyne_configuration::VLP16_StatusMessage _msg)
{
    //
    double rpm = _msg.motor_rpm;
    // On lock car on met à jour 'config_' du driver
    boost::mutex::scoped_lock lock(lock_);
    dvr_->update_rpm(rpm);
} // fin du lock


} // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_driver, DriverNodelet,
                        velodyne_driver::DriverNodelet, nodelet::Nodelet);
