// save_map.cpp
// 200316
// Service to save an octomap

#include <algorithm>
#include <cmath>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <time.h>

#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

octomap_msgs::Octomap octomap_buff;

std::string filename_base, filename_ext;

////////////////////////////////////////////////////////////////////////////////
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  octomap_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
bool savemapCallback(
  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  bool f_saving = request.data;
  if (f_saving)
  {
    octomap::OcTree *map
      = dynamic_cast<octomap::OcTree*>(
          octomap_msgs::fullMsgToMap(octomap_buff));

    time_t rawtime;
    char buffer [80];
    struct tm * timeinfo;
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,80,"%y%m%d_%H%M%S",timeinfo);
    std::string filename = filename_base + std::string(buffer) + filename_ext;
    ROS_INFO_STREAM("Saving map into " << filename);
    map->writeBinary(filename);
  }
  response.success = f_saving;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string filename;
  pnh.getParam("filename", filename);
  std::size_t found = filename.rfind(".");
  if (found!=std::string::npos)
  {
    filename_base = filename.substr(0, found);
    filename_ext = filename.substr(found);
  }
  else
  {
    filename_base = filename;
    filename_ext = ".bt";
  }

  std::string octomap_topic;
  pnh.getParam("octomap_topic", octomap_topic);
  ros::Subscriber octomap_sub = nh.subscribe(octomap_topic, 10, octomapCallback);

  std::string savemap_topic;
  pnh.getParam("savemap_topic", savemap_topic);
  ros::ServiceServer srv = nh.advertiseService(savemap_topic, savemapCallback);

  ros::spin();

  return(0);
}
