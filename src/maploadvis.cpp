// maploadvis.cpp

#include <algorithm>
#include <cmath>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "maploadvis");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::map<std::string, ros::Publisher> map2pubs;
  for (int i = 1; i <= 10; ++i)
  {
    std::stringstream ss_robot;
    ss_robot << "robot" << i;
    std::stringstream ss_pub;
    ss_pub << "/robot" << i << "/map_marker_occupied";
    map2pubs[ss_robot.str()]
      = nh.advertise<visualization_msgs::MarkerArray>(ss_pub.str(), 1);
  }

  std::vector<std::string> maplist;
  std::map<std::string, std::shared_ptr<octomap::OcTree> > m2map;

  std::string dirname = "localmaps";
  pnh.param<std::string>("dir_name", dirname, "");
  std::string strbuff;
  pnh.param<std::string>("file_list", strbuff, "");
  std::stringstream ss(strbuff);
  std::string filename;
  while (ss >> filename)
  {
    std::stringstream ss_filename(filename);
    std::string prefix, robotname, ID;
    std::getline(ss_filename, prefix, '-');
    std::getline(ss_filename, robotname, '-');
    std::getline(ss_filename, ID, '-');

    std::string mapname = robotname + "-" + ID;
    maplist.push_back(mapname);
    auto map = std::make_shared<octomap::OcTree>(dirname + "/" + filename);
    m2map[mapname] = map;
  }

  auto update_phase = ros::Duration(3.0);
  auto last_update = ros::Time::now();
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      last_update = now;

      for (auto mapname: maplist)
      {
        auto m = m2map[mapname];
        visualization_msgs::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m->getTreeDepth()+1);
        for (
          octomap::OcTree::iterator it = m->begin(m->getTreeDepth()),
          end = m->end(); it != end; ++it)
        {
          if (m->isNodeAtThreshold(*it))
          {
            double x = it.getX();
            double z = it.getZ();
            double y = it.getY();

            unsigned idx = it.getDepth();

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;
            std_msgs::ColorRGBA clr;
            clr.r = 0.5;
            clr.g = 0.5;
            clr.b = 0.5;
            clr.a = 1.0;

            if (m->isNodeOccupied(*it))
            {
              occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
              occupiedNodesVis.markers[idx].colors.push_back(clr);
            }
          }
        }
        for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
        {
          double size = m->getNodeSize(i);

          occupiedNodesVis.markers[i].header.frame_id = "world";
          occupiedNodesVis.markers[i].header.stamp = now;
          occupiedNodesVis.markers[i].ns = mapname;
          occupiedNodesVis.markers[i].id = i;
          occupiedNodesVis.markers[i].type
            = visualization_msgs::Marker::CUBE_LIST;
          occupiedNodesVis.markers[i].scale.x = size;
          occupiedNodesVis.markers[i].scale.y = size;
          occupiedNodesVis.markers[i].scale.z = size;

          // without this line, rviz complains orientation is uninitialized.
          occupiedNodesVis.markers[i].pose.orientation.w = 1;

          // set lifetime
          occupiedNodesVis.markers[i].lifetime = update_phase;

          //occupiedNodesVis.markers[i].color = m_color_occupied;

          if (occupiedNodesVis.markers[i].points.size() > 0)
            occupiedNodesVis.markers[i].action
              = visualization_msgs::Marker::ADD;
          else
            occupiedNodesVis.markers[i].action
              = visualization_msgs::Marker::DELETE;
        }
        std::stringstream ss_mapname(mapname);
        std::string robotname, ID;
        std::getline(ss_mapname, robotname, '-');
        std::getline(ss_mapname, ID, '-');
        map2pubs[robotname].publish(occupiedNodesVis);
      }
    }
    ros::spinOnce();
  }

  return(0);
}
