// rbpf.cpp
// 191028
// Rao-Blackwellized Particle Filter

#include <cmath>

#include <string>
#include <memory>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265

#include "rbpf.h"

ros::Publisher map_pub;
// octomap::OcTree *m_octree = NULL;

// int marker_counter = 0;
// visualization_msgs::MarkerArray occupiedNodesVis;
// std_msgs::ColorRGBA m_color_occupied;
// visualization_msgs::MarkerArray freeNodesVis;
// std_msgs::ColorRGBA m_color_free;
ros::Publisher marker_occupied_pub;
ros::Publisher marker_free_pub;

// ros::Subscriber r_pose_sub;
// geometry_msgs::PoseStamped r_pose;

// std::string imu_topic;
std::string odom_topic;
// ros::Publisher odom_pub;
std::string child_frame_id;

// double grav = 9.81;
// double x, y, z;
// double vx, vy, vz;
// ros::Time last_time;
// bool initial_imu = true;
// bool calibration = true;
//
// tf::Vector3 init_grav_dir;
// int sample_num;

////////////////////////////////////////////////////////////////////////////////
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_INFO("odomCallback!!!");
}
// void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
// {
//   ros::Time cur_time = ros::Time::now();
//
//   if (initial_imu)
//   {
//     init_grav_dir = tf::Vector3(
//                       imu->linear_acceleration.x,
//                       imu->linear_acceleration.y,
//                       imu->linear_acceleration.z);
//     sample_num = 1;
//     last_time = cur_time;
//     initial_imu = false;
//     return;
//   }
//
//   double dt = (cur_time - last_time).toSec();
//
//   if (calibration)
//   {
//     init_grav_dir += tf::Vector3(
//                       imu->linear_acceleration.x,
//                       imu->linear_acceleration.y,
//                       imu->linear_acceleration.z);
//     sample_num++;
//     if (dt > 3.0 && sample_num >= 100)
//     {
//       init_grav_dir /= sample_num;
//       grav = init_grav_dir.length();
//
//       // double roll, pitch, yaw;
//       // tf::Matrix3x3 mat(q);
//       // mat.getRPY(roll, pitch, yaw);
//       // ROS_INFO_STREAM("roll: " << (roll/M_PI*180) << ", pitch: " << (pitch/M_PI*180) << ", yaw: " << (yaw/M_PI*180));
//
//       last_time = cur_time;
//       calibration = false;
//     }
//     return;
//   }
//
//   last_time = cur_time;
//
//   // velocities
//   x += vx * dt;
//   y += vy * dt;
//   z += vz * dt;
//
//   // acceleration
//   tf::Vector3 acc_vec(
//     imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z
//   );
//   tf::Transform transform(
//     tf::Quaternion(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w),
//     tf::Vector3(0,0,0));
//   acc_vec = transform * acc_vec;
//   double ax = acc_vec.x();
//   double ay = acc_vec.y();
//   double az = acc_vec.z() - grav;
//   //if (std::fabs(ax) >= 1.0)
//     vx += ax * dt;
//   //if (std::fabs(ay) >= 1.0)
//     vy += ay * dt;
//   //if (std::fabs(az) >= 1.0)
//     vz += az * dt;
//
//   ROS_INFO_STREAM("dt: " << dt << ", ax: " << ax << ", ay: " << ay << ", az: " << az);
//
//   nav_msgs::Odometry odom;
//   odom.header = imu->header;
//   odom.header.frame_id = "world";
//   odom.child_frame_id = child_frame_id;
//   odom.pose.pose.position.x = x;
//   odom.pose.pose.position.y = y;
//   odom.pose.pose.position.z = z;
//   odom.pose.pose.orientation = imu->orientation;
//   odom.twist.twist.linear.x = vx;
//   odom.twist.twist.linear.y = vy;
//   odom.twist.twist.linear.z = vz;
//   odom.twist.twist.angular = imu->angular_velocity;
//
//   odom_pub.publish(odom);
// }

// void foo()
// {
//
//   // octomath::Vector3 point_sensor(rng_u[i].range, 0, 0);
//   //
//   // octomath::Vector3 sensor_global = pose_robot.transform(pose_u[i].trans());
//   // octomath::Vector3 point_global
//   //   = pose_robot.transform(pose_u[i].transform(point_sensor));
//   //
//   // m_octree->insertRay(sensor_global, point_global, 9.0);
//
//   auto rostime = ros::Time::now();
//
//   octomath::Vector3 r_position(
//     r_pose.pose.position.x, r_pose.pose.position.y, r_pose.pose.position.z);
//   octomath::Quaternion r_rotation(
//     r_pose.pose.orientation.w, r_pose.pose.orientation.x,
//     r_pose.pose.orientation.y, r_pose.pose.orientation.z);
//   octomath::Pose6D pose_robot(r_position, r_rotation);
//
//
//   octomap_msgs::Octomap map;
//   map.header.frame_id = "world";
//   map.header.stamp = rostime;
//   if (octomap_msgs::fullMapToMsg(*m_octree, map))
//     map_pub.publish(map);
//   else
//     ROS_ERROR("Error serializing OctoMap");
//
//   if (marker_counter >= 5)
//   {
//     m_octree->toMaxLikelihood();
//     m_octree->prune();
//
//     for (
//       octomap::OcTree::iterator it = m_octree->begin(m_octree->getTreeDepth()),
//       end = m_octree->end(); it != end; ++it)
//     {
//       if (m_octree->isNodeAtThreshold(*it))
//       {
//         double x = it.getX();
//         double z = it.getZ();
//         double y = it.getY();
//
//         unsigned idx = it.getDepth();
//         geometry_msgs::Point cubeCenter;
//         cubeCenter.x = x;
//         cubeCenter.y = y;
//         cubeCenter.z = z;
//
//         if (m_octree->isNodeOccupied(*it))
//         {
//           occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
//
//           double cosR = std::cos(PI*z/10.0)*0.8+0.2;
//           double cosG = std::cos(PI*(2.0/3.0+z/10.0))*0.8+0.2;
//           double cosB = std::cos(PI*(4.0/3.0+z/10.0))*0.8+0.2;
//           std_msgs::ColorRGBA clr;
//           clr.r = (cosR > 0)? cosR: 0;
//           clr.g = (cosG > 0)? cosG: 0;
//           clr.b = (cosB > 0)? cosB: 0;
//           clr.a = 0.5;
//           occupiedNodesVis.markers[idx].colors.push_back(clr);
//         }
//         else
//         {
//           freeNodesVis.markers[idx].points.push_back(cubeCenter);
//         }
//       }
//     }
//
//     for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
//     {
//       double size = m_octree->getNodeSize(i);
//
//       occupiedNodesVis.markers[i].header.frame_id = "world";
//       occupiedNodesVis.markers[i].header.stamp = rostime;
//       occupiedNodesVis.markers[i].ns = "robot";
//       occupiedNodesVis.markers[i].id = i;
//       occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
//       occupiedNodesVis.markers[i].scale.x = size;
//       occupiedNodesVis.markers[i].scale.y = size;
//       occupiedNodesVis.markers[i].scale.z = size;
//
//       //occupiedNodesVis.markers[i].color = m_color_occupied;
//
//       if (occupiedNodesVis.markers[i].points.size() > 0)
//         occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
//       else
//         occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
//     }
//     marker_occupied_pub.publish(occupiedNodesVis);
//
//     for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i)
//     {
//       double size = m_octree->getNodeSize(i);
//
//       freeNodesVis.markers[i].header.frame_id = "world";
//       freeNodesVis.markers[i].header.stamp = rostime;
//       freeNodesVis.markers[i].ns = "robot";
//       freeNodesVis.markers[i].id = i;
//       freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
//       freeNodesVis.markers[i].scale.x = size;
//       freeNodesVis.markers[i].scale.y = size;
//       freeNodesVis.markers[i].scale.z = size;
//
//       freeNodesVis.markers[i].color = m_color_free;
//
//       if (freeNodesVis.markers[i].points.size() > 0)
//         freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
//       else
//         freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
//     }
//     marker_free_pub.publish(freeNodesVis);
//
//     marker_counter = 0;
//   }
//   else
//   {
//     marker_counter++;
//   }
//
// }

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(const double &resol,
  const double &probHit, const double &probMiss,
  const double &threshMin, const double &threshMax)
{
  this->map = new octomap::OcTree(resol);
  this->map->setProbHit(probHit);
  this->map->setProbMiss(probMiss);
  this->map->setClampingThresMin(threshMin);
  this->map->setClampingThresMax(threshMax);
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(): Particle(0.25, 0.7, 0.4, 0.12, 0.97){}

////////////////////////////////////////////////////////////////////////////////
Particle::~Particle()
{
  delete this->map;
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  map_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);

  // occupiedNodesVis.markers.resize(m_octree->getTreeDepth()+1);
  // m_color_occupied.r = 1;
  // m_color_occupied.g = 1;
  // m_color_occupied.b = 0.3;
  // m_color_occupied.a = 0.5;
  // freeNodesVis.markers.resize(m_octree->getTreeDepth()+1);
  // m_color_free.r = 1;
  // m_color_free.g = 1;
  // m_color_free.b = 1;
  // m_color_free.a = 0.05;

  // r_pose_sub = n.subscribe("pose", 1, updateRobotPose);
  marker_occupied_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  marker_free_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_free", 1);

  // marker_counter = 0;

  // x = y = z = 0;
  // vx = vy = vz = 0;
  // last_time = ros::Time::now();

  // pnh.getParam("imu_topic", imu_topic);
  pnh.getParam("odom_topic", odom_topic);
  pnh.getParam("child_frame_id", child_frame_id);

  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1000, odomCallback);
  // odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
  //
  // ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1000, imuCallback);

  // === Initialize PF ===
  int n_particles;
  double update_freq;
  pnh.getParam("n_particles", n_particles);
  pnh.getParam("update_freq", update_freq);
  const ros::Duration update_phase(1.0/update_freq);

  double resol;
  double probHit;
  double probMiss;
  double threshMin;
  double threshMax;
  pnh.getParam("map_resol", resol);
  pnh.getParam("map_probHit", probHit);
  pnh.getParam("map_probMiss", probMiss);
  pnh.getParam("map_threshMin", threshMin);
  pnh.getParam("map_threshMax", threshMax);

  std::vector< std::shared_ptr<Particle> > particles;
  for (int i = 0; i < n_particles; ++i)
  {
    particles.push_back(
      std::make_shared<Particle>(
        resol, probHit, probMiss, threshMin, threshMax));
  }
  std::vector<double> weights(n_particles);
  std::vector<double> errors(n_particles);
  ros::Time last_update = ros::Time::now();

  while (ros::ok())
  {
    // === Update PF ===
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      double deltaT = (now - last_update).toSec();
      last_update = now;
      ROS_INFO_STREAM("PF update: " << now);
      // Get sensory data (IMU, sonar, depth cam)
      // TODO

      // init weights and errors
      for (int i = 0; i < n_particles; ++i)
      {
        weights[i] = 0;
        errors[i] = 0;
      }

      // predict PF
      // TODO

      // weight PF
      // TODO

      // resample PF (and update map)
      // TODO

      // initialize the time step
      // TODO

    }

    ros::spinOnce();
  }

  return(0);
}
