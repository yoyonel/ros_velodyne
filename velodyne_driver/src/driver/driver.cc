/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
//
#include <velodyne_msgs/VelodyneScan.h>
//#include <velodyne_msgs/VelodyneStatus.h>
//
#include "driver.h"

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
    :
      private_nh_(private_nh)
{
    // use private node handle to get parameters
    private_nh_.param("frame_id", config_.frame_id, std::string("velodyne"));
    std::string tf_prefix = tf::getPrefixParam(private_nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

    // get model name, validate string, determine packet rate
    private_nh_.param("model", config_.model, std::string("64E"));
    //  double packet_rate_;                   // packet frequency (Hz)
    std::string model_full_name;
    //
    config_.rpm_max = -1;  // default value (=> no value)
    config_.rpm_min = -1;  // default value (=> no value)
    //
    if ((config_.model == "64E_S2") ||
            (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
        packet_rate_ = 3472.17;            // 1333312 / 384
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "64E")
    {
        packet_rate_ = 2600.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32E")
    {
        packet_rate_ = 1808.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "VLP16")
    {
        packet_rate_ = 781.25;             // 300000 / 384
        model_full_name = "VLP-16";
        //
        config_.rpm_max = 600;  // max RPM for VLP-16
        config_.rpm_min = 300;  // min RPM for VLP-16
    }
    else
    {
        ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
        packet_rate_ = 2600.0;
    }
    //
    config_.rpm_range = config_.rpm_max - config_.rpm_min;
    //  std::string deviceName_(std::string("Velodyne ") + model_full_name);
    deviceName_ = std::string("Velodyne ") + model_full_name;

    private_nh_.param("rpm", config_.rpm, 600.0);
    //
    config_.rpm_max = config_.rpm_max == -1 ? config_.rpm : config_.rpm_max;    // by take current RPM for max (if no setting for rpm_max)
    config_.rpm_min = config_.rpm_min == -1 ? config_.rpm : config_.rpm_min;    // by take current RPM for min (if no setting for rpm_min)
    //
    ROS_INFO_STREAM(deviceName_ << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0);     // expected Hz rate
    //
    const double frequency_min = (config_.rpm_min / 60.0);
    const double frequency_max = (config_.rpm_max / 60.0);

    // default number of packets for each scan is a single revolution
    // (fractions rounded up)
    config_.npackets = (int) ceil(packet_rate_ / frequency);
    //
    const int config_npackets_min = (int) ceil(packet_rate_ / frequency_min);
    const int config_npackets_max = (int) ceil(packet_rate_ / frequency_max);
    //
    private_nh_.getParam("npackets", config_.npackets);
    ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

    std::string dump_file;
    private_nh_.param("pcap", dump_file, std::string(""));

    int udp_port;
    private_nh_.param("port", udp_port, (int)UDP_PORT_NUMBER);

    // initialize diagnostics
    diagnostics_.setHardwareID(deviceName_);
    const double diag_freq = packet_rate_/config_.npackets;
    //
    double diag_min_freq_ = packet_rate_/config_npackets_max;
    double diag_max_freq_ = packet_rate_/config_npackets_min;
    //
//    diag_max_freq_ = diag_freq;
//    diag_min_freq_ = diag_freq;
    //
    ROS_INFO("expected frequency: %.3f (Hz) (min/max: %.3f, %.3f (Hz))",
             diag_freq, diag_min_freq_, diag_max_freq_);

    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                               &diag_max_freq_,
                                                               0.1, 10),
                                          TimeStampStatusParam()));

    // open Velodyne input device or file
    using_input_socket_ = dump_file == "";
    if (!using_input_socket_)
    {
        input_.reset(new velodyne_driver::InputPCAP(private_nh_,
                                                    packet_rate_,
                                                    dump_file));
    }
    else
    {
        input_.reset(new velodyne_driver::InputSocket(private_nh_, udp_port));
    }

    std::string devip;
    private_nh_.param("device_ip", devip, std::string(""));
    if(!devip.empty())
        ROS_INFO_STREAM("Set device ip to " << devip << ", only accepting packets from this address." );
    input_->setDeviceIP(devip);

    // raw data output topic
    output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
//    boost::mutex::scoped_lock lock(lock_);

    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);

    scan->packets.resize(config_.npackets);

    // Since the velodyne delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    for (int i = 0; i < config_.npackets; ++i)
    {
        while (true)
        {
            // keep reading until full packet received
            int rc = input_->getPacket(&scan->packets[i]);
            if (rc == 0) break;       // got a full packet?
            if (rc < 0) return false; // end of file reached?
        }
    }

    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full Velodyne scan.");
    scan->header.stamp = ros::Time(scan->packets[config_.npackets - 1].stamp);
    scan->header.frame_id = config_.frame_id;
    output_.publish(scan);

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic_->tick(scan->header.stamp);
    diagnostics_.update();

    return true;
}

bool VelodyneDriver::update_rpm(const double& _rpm)
{
    //    boost::mutex::scoped_lock lock(lock_);

    bool b_not_update_rpm = false;
    if( config_.rpm_range )
    {
        const double diff_rpm = std::abs(config_.rpm - _rpm) / config_.rpm_range;
        b_not_update_rpm = diff_rpm < 0.01;  // diff < 1%
        ROS_INFO("update_rpm -> diff_rpm: %.3f %%", diff_rpm*100.0);
    }
    else {
        b_not_update_rpm = config_.rpm == _rpm;
    }
    if( b_not_update_rpm )
        return true;

    ROS_INFO_STREAM("Update RPM setting for Velodyne ...");

    ROS_INFO_STREAM(deviceName_ << " rotated at " << config_.rpm << " RPM");
    config_.rpm = _rpm;
    ROS_INFO_STREAM(deviceName_ << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0);     // expected Hz rate

    // default number of packets for each scan is a single revolution
    // (fractions rounded up)
    config_.npackets = (int) ceil(packet_rate_ / frequency);
    private_nh_.getParam("npackets", config_.npackets);
    ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

    return true;
}

} // namespace velodyne_driver
