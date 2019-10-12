

#include <signal.h>

#include <cmath>

#include <map>

#include <geometry_msgs/PoseStamped.h>
#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher map_pub;
octomap::OcTree *m_octree = NULL;

int marker_counter = 0;
visualization_msgs::MarkerArray occupiedNodesVis;
std_msgs::ColorRGBA m_color_occupied;
visualization_msgs::MarkerArray freeNodesVis;
std_msgs::ColorRGBA m_color_free;
ros::Publisher marker_occupied_pub;
ros::Publisher marker_free_pub;

ros::Subscriber r_pose_sub;
geometry_msgs::PoseStamped r_pose;

////////////////////////////////////////////////////////////////////////////////
void foo()
{

  // octomath::Vector3 point_sensor(rng_u[i].range, 0, 0);
  //
  // octomath::Vector3 sensor_global = pose_robot.transform(pose_u[i].trans());
  // octomath::Vector3 point_global
  //   = pose_robot.transform(pose_u[i].transform(point_sensor));
  //
  // m_octree->insertRay(sensor_global, point_global, 9.0);

  auto rostime = ros::Time::now();

  octomath::Vector3 r_position(
    r_pose.pose.position.x, r_pose.pose.position.y, r_pose.pose.position.z);
  octomath::Quaternion r_rotation(
    r_pose.pose.orientation.w, r_pose.pose.orientation.x,
    r_pose.pose.orientation.y, r_pose.pose.orientation.z);
  octomath::Pose6D pose_robot(r_position, r_rotation);


  octomap_msgs::Octomap map;
  map.header.frame_id = "world";
  map.header.stamp = rostime;
  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    map_pub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");

  if (marker_counter >= 5)
  {
    m_octree->toMaxLikelihood();
    m_octree->prune();

    for (
      octomap::OcTree::iterator it = m_octree->begin(m_octree->getTreeDepth()),
      end = m_octree->end(); it != end; ++it)
    {
      if (m_octree->isNodeAtThreshold(*it))
      {
        double x = it.getX();
        double z = it.getZ();
        double y = it.getY();

        unsigned idx = it.getDepth();
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        if (m_octree->isNodeOccupied(*it))
        {
          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

          double cosR = std::cos(PI*z/10.0)*0.8+0.2;
          double cosG = std::cos(PI*(2.0/3.0+z/10.0))*0.8+0.2;
          double cosB = std::cos(PI*(4.0/3.0+z/10.0))*0.8+0.2;
          std_msgs::ColorRGBA clr;
          clr.r = (cosR > 0)? cosR: 0;
          clr.g = (cosG > 0)? cosG: 0;
          clr.b = (cosB > 0)? cosB: 0;
          clr.a = 0.5;
          occupiedNodesVis.markers[idx].colors.push_back(clr);
        }
        else
        {
          freeNodesVis.markers[idx].points.push_back(cubeCenter);
        }
      }
    }

    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
    {
      double size = m_octree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = "world";
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "robot";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;

      //occupiedNodesVis.markers[i].color = m_color_occupied;

      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_occupied_pub.publish(occupiedNodesVis);

    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i)
    {
      double size = m_octree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = "world";
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "robot";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;

      freeNodesVis.markers[i].color = m_color_free;

      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    marker_free_pub.publish(freeNodesVis);

    marker_counter = 0;
  }
  else
  {
    marker_counter++;
  }

}

void updateRobotPose(const geometry_msgs::PoseStamped::ConstPtr& new_pose)
{
  ROS_INFO("This message is from updateRobotPose");
  r_pose = *new_pose;
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_octomap");

  // enable the motors
  ros::NodeHandle n;

  map_pub = n.advertise<octomap_msgs::Octomap>("octomap", 1);

  m_octree = new octomap::OcTree(0.25);
  m_octree->setProbHit(0.7);
  m_octree->setProbMiss(0.4);
  m_octree->setClampingThresMin(0.12);
  m_octree->setClampingThresMax(0.97);

  occupiedNodesVis.markers.resize(m_octree->getTreeDepth()+1);
  m_color_occupied.r = 1;
  m_color_occupied.g = 1;
  m_color_occupied.b = 0.3;
  m_color_occupied.a = 0.5;
  freeNodesVis.markers.resize(m_octree->getTreeDepth()+1);
  m_color_free.r = 1;
  m_color_free.g = 1;
  m_color_free.b = 1;
  m_color_free.a = 0.05;

  r_pose_sub = n.subscribe("pose", 1, updateRobotPose);
  marker_occupied_pub
    = n.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  marker_free_pub
    = n.advertise<visualization_msgs::MarkerArray>("map_marker_free", 1);

  marker_counter = 0;

  while(ros::ok())
  {

    ros::spinOnce();
  }

  delete m_octree;

  return(0);
}
