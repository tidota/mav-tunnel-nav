// save_map.cpp
// 200316
// Service to save an octomap

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

#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <mav_tunnel_nav/OctomapWithSegId.h>
#include <mav_tunnel_nav/SubmapAck.h>

std::map<std::string, mav_tunnel_nav::OctomapWithSegId> map_list;
//std::mutex map_mutex;

std::deque<mav_tunnel_nav::SubmapAck> ack_list;
//std::mutex ack_mutex;

std::string filename_base, filename_ext;

ros::Subscriber submap_ack_sub;

////////////////////////////////////////////////////////////////////////////////
void octomapCallback(const mav_tunnel_nav::OctomapWithSegId::ConstPtr& msg)
{
  //std::lock_guard<std::mutex> lk(map_mutex);
  map_list[msg->segid] = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void submapAckCallback(const mav_tunnel_nav::SubmapAck::ConstPtr& msg)
{
  //std::lock_guard<std::mutex> lk(ack_mutex);
  ack_list.push_back(*msg);
}

////////////////////////////////////////////////////////////////////////////////
bool savemapCallback(
  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  bool f_saving = request.data;
  if (f_saving)
  {
    //std::lock_guard<std::mutex> lk(map_mutex);
    for (auto p: map_list)
    {
      octomap::OcTree *map
        = dynamic_cast<octomap::OcTree*>(
            octomap_msgs::fullMsgToMap(p.second.octomap));

      time_t rawtime;
      char buffer [80];
      struct tm * timeinfo;
      time (&rawtime);
      timeinfo = localtime (&rawtime);
      strftime (buffer,80,"%y%m%d_%H%M%S",timeinfo);
      std::string filename
        = filename_base
        + "-" + p.first + "-"
        + std::string(buffer) + filename_ext;
      ROS_INFO_STREAM("Saving map into " << filename);
      map->writeBinary(filename);
    }
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

  // get this robot's name
  std::string robot_name;
  if (!pnh.getParam("robot_name", robot_name))
    ROS_ERROR_STREAM("no ros parameter: robot_name");

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
  ros::Subscriber octomap_sub
    = nh.subscribe(octomap_topic, 10, octomapCallback);

  std::string ack_topic;
  pnh.getParam("ack_topic", ack_topic);
  submap_ack_sub = nh.subscribe(ack_topic, 1000, submapAckCallback);

  std::string savemap_topic;
  pnh.getParam("savemap_topic", savemap_topic);
  ros::ServiceServer srv = nh.advertiseService(savemap_topic, savemapCallback);

  ros::Time checkpoint = ros::Time::now();
  const ros::Duration duration(0.01);
  while(ros::ok())
  {
    if (ros::Time::now() - checkpoint >= duration)
    {
      // check the ack and delete the corresponding maps
      while (ack_list.size() > 0)
      {
        auto ack = ack_list.front();
        ack_list.pop_front();

        std::stringstream ss;
        ss << robot_name << "-"
           << std::setw(3) << std::setfill('0')
           << std::stoi(ack.submap_id);
        if (map_list.count(ss.str()) > 0)
        {
          map_list.erase(ss.str());
        }
        else
        {
          ROS_ERROR_STREAM("no such map to delete: " << ss.str());
        }
      }
    }

    ros::spinOnce();
  }

  return(0);
}
