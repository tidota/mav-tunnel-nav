// colorchg.cpp

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

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

std::vector<ros::Subscriber> subs;
std::map<std::string, ros::Publisher> map2pubs;
std::map<std::string, std::vector<double> > color_list;

////////////////////////////////////////////////////////////////////////////////
void mkCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  std::stringstream ss(msg->markers[0].ns);
  std::string robot;
  std::getline(ss, robot, '-');
  if (color_list.count(robot) > 0)
  {
    visualization_msgs::MarkerArray occupiedNodesVis;
    occupiedNodesVis.markers.resize(msg->markers.size());
    for (unsigned i= 0; i < msg->markers.size(); ++i)
    {
      for (auto& point: msg->markers[i].points)
      {
        occupiedNodesVis.markers[i].points.push_back(point);
        double brightness = -point.z/20.0;
        while (brightness < 0)
          brightness += 1.0;
        while (brightness > 1.0)
          brightness -= 1.0;
        if (brightness >= 0.5)
          brightness = (brightness - 0.5)*2.0;
        else
          brightness = -(brightness - 0.5)*2.0;
        brightness = brightness * 0.8 + 0.2;
        std_msgs::ColorRGBA clr = msg->markers[i].colors[0];
        clr.r = color_list[robot][0] * brightness;
        clr.g = color_list[robot][1] * brightness;
        clr.b = color_list[robot][2] * brightness;
        occupiedNodesVis.markers[i].colors.push_back(clr);
      }

      occupiedNodesVis.markers[i].header.frame_id
        = msg->markers[i].header.frame_id;
      occupiedNodesVis.markers[i].header.stamp
        = msg->markers[i].header.stamp;
      occupiedNodesVis.markers[i].ns
        = msg->markers[i].ns;
      occupiedNodesVis.markers[i].id = msg->markers[i].id;
      occupiedNodesVis.markers[i].type = msg->markers[i].type;
      occupiedNodesVis.markers[i].scale.x
        = msg->markers[i].scale.x;
      occupiedNodesVis.markers[i].scale.y
        = msg->markers[i].scale.y;
      occupiedNodesVis.markers[i].scale.z
        = msg->markers[i].scale.z;

      // without this line, rviz complains orientation is uninitialized.
      occupiedNodesVis.markers[i].pose.orientation.w
        = msg->markers[i].pose.orientation.w;

      // set lifetime
      occupiedNodesVis.markers[i].lifetime
        = msg->markers[i].lifetime;

      //occupiedNodesVis.markers[i].color = m_color_occupied;

      occupiedNodesVis.markers[i].action = msg->markers[i].action;
    }
    if (map2pubs.count(robot) > 0)
    {
      map2pubs[robot].publish(occupiedNodesVis);
    }
    else
    {
      ROS_ERROR_STREAM("no such key for the publishers: " << robot);
    }
  }
  else
  {
    ROS_ERROR_STREAM("no such name space: " << robot);
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "colorchg");
  ros::NodeHandle nh;

  const double palette[10][3]
    = {
        {1.0, 0.2, 0.2}, // red
        {0.2, 1.0, 0.2}, // green
        {0.2, 0.2, 1.0}, // blue
        {1.0, 1.0, 0.0}, // yellow
        {0.0, 1.0, 1.0}, // cyan
        {1.0, 0.0, 1.0}, // magenta
        {1.0, 0.5, 0.0}, // orange
        {0.5, 1.0, 0.0}, // green yellow
        {0.5, 0.0, 1.0}, // volet
        {0.0, 0.5, 1.0}  // progress cyan
      };

  for (int i = 1; i <= 10; ++i)
  {
    std::stringstream ss_robot;
    ss_robot << "robot" << i;
    int indx = i - 1;
    color_list[ss_robot.str()]
      = {palette[indx][0], palette[indx][1], palette[indx][2]};
    std::stringstream ss_pub;
    ss_pub << "/robot" << i << "/map_marker_mapped_color";
    map2pubs[ss_robot.str()]
      = nh.advertise<visualization_msgs::MarkerArray>(ss_pub.str(), 1);
    std::stringstream ss_sub;
    ss_sub << "/robot" << i << "/map_marker_occupied";
    subs.push_back(nh.subscribe(ss_sub.str(), 1000, mkCallback));
  }

  ros::spin();

  //t.join();

  return(0);
}
