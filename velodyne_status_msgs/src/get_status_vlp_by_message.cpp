#include <ros/ros.h>
#include <string>
#include "velodyne_status_msgs/ServiceVelodyneStatus.h"

std::string exec(const char* cmd) {
  FILE* pipe = popen(cmd, "r");
  if (!pipe) return "ERROR";
  char buffer[128];
  std::string result = "";
  while(!feof(pipe)) {
      if(fgets(buffer, 128, pipe) != NULL)
        result += buffer;
    }
  pclose(pipe);
  return result;
}

bool get_status(velodyne_status_msgs::ServiceVelodyneStatusRequest &req,
                velodyne_status_msgs::ServiceVelodyneStatusResponse &res)
{
  //
//  res.msg.gps_position = "";
  std::string cmd = "timeout 0.03s curl -s http://192.168.1.201/cgi/status.json";
  std::string str_exec_res = exec(cmd.c_str());
  res.msg.gps_position = str_exec_res;
  //
  res.msg.motor_state = true;
  res.msg.motor_rpm = 600;
  res.msg.laser_state = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_status_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("velodyne_status", get_status);
  ROS_INFO("Ready to get velodyne status.");
  ros::spin();

  return 0;
}

