// save_all_server.cpp
// 210910
// Service to save all the octomap

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <time.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

std::vector<std::string> robotList;
std::map<std::string, ros::ServiceClient> srv_client_list;

////////////////////////////////////////////////////////////////////////////////
bool saveallCallback(
  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  bool f_saving = request.data;
  if (f_saving)
  {
    // call all map savers
    for (auto robot: robotList)
    {
      std_srvs::SetBool srv;
      srv.request.data = true;
      auto res = srv_client_list[robot].call(srv);
    }
  }
  response.success = f_saving;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_all_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  nh.param("robots", robotList, robotList);

  std::string saveall_topic;
  pnh.getParam("saveall_topic", saveall_topic);
  ros::ServiceServer srv = nh.advertiseService(saveall_topic, saveallCallback);

  std::string savemap_topic;
  pnh.getParam("savemap_topic", savemap_topic);
  for (auto robot: robotList)
  {
    srv_client_list[robot]
      = nh.serviceClient<std_srvs::SetBool>(
        "/" + robot + "/" + savemap_topic);
  }

  ros::spin();

  return(0);
}
