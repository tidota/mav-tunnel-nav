// beacon
// 10/18/2020
// broadcast a beacon signal
// reply to a beacon from another robot

#include <string>
#include <map>
#include <memory>
#include <random>
#include <vector>

#include <mav_tunnel_nav/Beacon.h>

#include <ros/ros.h>

ros::Publisher beacon_pub;
std::string robot_name;

////////////////////////////////////////////////////////////////////////////////
void beaconCallback(const mav_tunnel_nav::Beacon::ConstPtr& msg)
{
  // if it is a broad cast from another robot, reply to it
  if (msg->destination.size() == 0 && msg->source != robot_name)
  {
    mav_tunnel_nav::Beacon msg2send = *msg;
    msg2send.source = robot_name;
    msg2send.destination = msg->source;
    beacon_pub.publish(msg2send);
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_control");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get this robot's name
  if (!pnh.getParam("robot_name", robot_name))
    ROS_ERROR_STREAM("no ros parameter: robot_name");

  // subscriber for beacon
  std::string beacon_down_topic;
  ros::Subscriber beacon_sub;
  if (nh.getParam("/beacon_down_topic", beacon_down_topic)) // global param
  {
    beacon_sub = nh.subscribe(beacon_down_topic, 1000, beaconCallback);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: beacon_down_topic");
  }

  // publisher for beacon
  std::string beacon_up_topic;
  if (nh.getParam("/beacon_up_topic", beacon_up_topic)) // global param
  {
    beacon_pub = nh.advertise<mav_tunnel_nav::Beacon>(beacon_up_topic, 1);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: beacon_up_topic");
  }

  ros::Time last_update = ros::Time::now();
  const ros::Duration update_phase(0.1);

  mav_tunnel_nav::Beacon msg;
  msg.source = robot_name;
  msg.destination = "";
  while (ros::ok())
  {
    ros::Time current = ros::Time::now();
    if (current - last_update > update_phase)
    {
      beacon_pub.publish(msg);
      last_update = current;
    }
    ros::spinOnce();
  }

  return(0);
}
