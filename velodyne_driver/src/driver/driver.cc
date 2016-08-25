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
    }
    else
    {
        ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
        packet_rate_ = 2600.0;
    }
    //  std::string deviceName_(std::string("Velodyne ") + model_full_name);
    deviceName_ = std::string("Velodyne ") + model_full_name;

    private_nh_.param("rpm", config_.rpm, 600.0);
    ROS_INFO_STREAM(deviceName_ << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0);     // expected Hz rate

    // default number of packets for each scan is a single revolution
    // (fractions rounded up)
    config_.npackets = (int) ceil(packet_rate_ / frequency);
    private_nh_.getParam("npackets", config_.npackets);
    ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

    std::string dump_file;
    private_nh_.param("pcap", dump_file, std::string(""));

    int udp_port;
    private_nh_.param("port", udp_port, (int)UDP_PORT_NUMBER);

    // initialize diagnostics
    diagnostics_.setHardwareID(deviceName_);
    const double diag_freq = packet_rate_/config_.npackets;
    diag_max_freq_ = diag_freq;
    diag_min_freq_ = diag_freq;
    ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

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
    //
    //  output_status_ = node.advertise<velodyne_msgs::VelodyneStatus>("velodyne_status", 10);
}

//std::string exec(const char* cmd) {
//  FILE* pipe = popen(cmd, "r");
//  if (!pipe) return "ERROR";
//  char buffer[128];
//  std::string result = "";
//  while(!feof(pipe)) {
//      if(fgets(buffer, 128, pipe) != NULL)
//        result += buffer;
//    }
//  pclose(pipe);
//  return result;
//}

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

    //  //
    //  // if (using_input_socket_)
    //  if (true)
    //    {
    //      //
    //      velodyne_msgs::VelodyneStatusPtr status(new velodyne_msgs::VelodyneStatus);
    //      //
    //      status->gps_position = "";
    //      status->motor_state = true;
    //      status->motor_rpm = 600;
    //      status->laser_state = true;
    //      // std::string cmd = "timeout 0.03s curl -s http://192.168.1.201/cgi/status.json";
    //      // std::string res = exec(cmd.c_str());
    //      // status->gps_position = res;
    //      // publish message
    //      ROS_DEBUG("Publishing Velodyne status.");
    //      output_status_.publish(status);
    //    }

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic_->tick(scan->header.stamp);
    diagnostics_.update();

    return true;
}

bool VelodyneDriver::update_rpm(const double& _rpm)
{
    //    boost::mutex::scoped_lock lock(lock_);

    if( config_.rpm == _rpm )
        return true;

    ROS_INFO_STREAM("Update RPM setting for Velodyne ...");

    config_.rpm = _rpm;

    ROS_INFO_STREAM(deviceName_ << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0);     // expected Hz rate

    // default number of packets for each scan is a single revolution
    // (fractions rounded up)
    config_.npackets = (int) ceil(packet_rate_ / frequency);
    private_nh_.getParam("npackets", config_.npackets);
    ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

    // initialize diagnostics
    //    diagnostics_.setHardwareID(deviceName);
    const double diag_freq = packet_rate_/config_.npackets;
    diag_max_freq_ = diag_freq;
    diag_min_freq_ = diag_freq;
    ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

    //    using namespace diagnostic_updater;
    //    diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
    //                                          FrequencyStatusParam(&diag_min_freq_,
    //                                                               &diag_max_freq_,
    //                                                               0.1, 10),
    //                                          TimeStampStatusParam()));

    return true;
}

} // namespace velodyne_driver