// rbpf.cpp
// 191028
// Rao-Blackwellized Particle Filter

#include <cmath>

#include <string>
#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <octomap/octomap.h>
#include <octomap/math/Pose6D.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265

#include "rbpf.h"

std::string odom_topic;
std::string pc_topic;
std::string world_frame_id;
std::string robot_frame_id;

nav_msgs::Odometry odom_buff;
std::mutex odom_mutex;

sensor_msgs::PointCloud2 pc_buff;
std::mutex pc_mutex;

////////////////////////////////////////////////////////////////////////////////
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(odom_mutex);
  odom_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(pc_mutex);
  pc_buff = *msg;
}

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

  this->pose = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  this->vel_linear = tf::Vector3(0, 0, 0);
  this->vel_angle = tf::Vector3(0, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(const Particle &src)
{
  // copy the localization data
  this->pose = src.pose;
  this->vel_linear = src.vel_linear;
  this->vel_angle = src.vel_angle;

  // copy the mapping data
  this->map = new octomap::OcTree(*src.map);
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(): Particle(0.25, 0.7, 0.4, 0.12, 0.97){}

////////////////////////////////////////////////////////////////////////////////
Particle::~Particle()
{
  delete this->map;
}

////////////////////////////////////////////////////////////////////////////////
const tf::Pose Particle::getPose()
{
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
const octomap::OcTree* Particle::getMap()
{
  return this->map;
}

////////////////////////////////////////////////////////////////////////////////
void Particle::predict(
  const tf::Vector3 &lin, const tf::Vector3 &ang,
  const double &deltaT, std::mt19937 &gen)
{
  this->vel_linear = lin;
  this->vel_angle = ang;
  std::normal_distribution<> motion_noise_lin(0, 0.1);
  std::normal_distribution<> motion_noise_ang(0, 0.01);
  tf::Vector3 mov_lin(
    lin.x() * deltaT + motion_noise_lin(gen),
    lin.y() * deltaT + motion_noise_lin(gen),
    lin.z() * deltaT + motion_noise_lin(gen));
  tf::Quaternion mov_ang;
  mov_ang.setRPY(
    ang.x() * deltaT + motion_noise_ang(gen),
    ang.y() * deltaT + motion_noise_ang(gen),
    ang.z() * deltaT + motion_noise_ang(gen));
  this->pose = this->pose * tf::Transform(mov_ang, mov_lin);
}

////////////////////////////////////////////////////////////////////////////////
double Particle::evaluate(const octomap::Pointcloud &scan)
{
  double log_lik = 0;

  // for all point in the point cloud
  octomap::OcTreeKey key;
  octomap::OcTreeNode *node;
  for (unsigned int ip = 0; ip < scan.size(); ++ip)
  {
    tf::Vector3 point
          = this->pose * tf::Vector3(scan[ip].x(), scan[ip].y(), scan[ip].z());
    if (this->map->coordToKeyChecked(
          point.x(), point.y(), point.z(), key)
        && (node = this->map->search(key,0)))
    {
      log_lik += std::log(node->getOccupancy());
    }
  }

  return std::exp(log_lik);
}

////////////////////////////////////////////////////////////////////////////////
void Particle::update_map(const octomap::Pointcloud &scan)
{
  octomath::Vector3 sensor_org(0,0,0);
  tf::Vector3 pose_org = this->pose.getOrigin();
  tf::Quaternion pose_rot = this->pose.getRotation();
  octomath::Pose6D frame_org(
    octomath::Vector3(pose_org.x(), pose_org.y(), pose_org.z()),
    octomath::Quaternion(
      pose_rot.w(), pose_rot.x(), pose_rot.y(), pose_rot.z())
  );
  this->map->insertPointCloud(scan, sensor_org, frame_org);
}

////////////////////////////////////////////////////////////////////////////////
void Particle::compress_map()
{
  this->map->toMaxLikelihood();
  this->map->prune();
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // random numbers
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<> dis(0, 1.0);

  tf::TransformBroadcaster tf_broadcaster;

  ros::Publisher map_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
  ros::Publisher marker_occupied_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  ros::Publisher vis_poses_pub
    = nh.advertise<geometry_msgs::PoseArray>("loc_vis_poses", 1, true);

  pnh.getParam("odom_topic", odom_topic);
  pnh.getParam("pc_topic", pc_topic);
  pnh.getParam("world_frame_id", world_frame_id);
  pnh.getParam("robot_frame_id", robot_frame_id);

  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1000, odomCallback);
  ros::Subscriber pc_sub = nh.subscribe(pc_topic, 1000, pcCallback);
  // odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
  //
  // ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1000, imuCallback);

  // === Initialize PF ===
  int n_particles;
  double update_freq;
  pnh.getParam("n_particles", n_particles);
  pnh.getParam("update_freq", update_freq);
  const ros::Duration update_phase(1.0/update_freq);

  int depth_cam_pc_downsample;
  pnh.getParam("depth_cam_pc_downsample", depth_cam_pc_downsample);

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

  double t_only_mapping;
  pnh.getParam("t_only_mapping", t_only_mapping);
  const ros::Duration phase_only_mapping(t_only_mapping);

  std::vector< std::shared_ptr<Particle> > particles;
  for (int i = 0; i < n_particles; ++i)
  {
    particles.push_back(
      std::make_shared<Particle>(
        resol, probHit, probMiss, threshMin, threshMax));
  }
  std::vector<double> weights(n_particles);
  std::vector<double> errors(n_particles);
  const ros::Time initial_update = ros::Time::now();
  ros::Time last_update = initial_update;

  tf::Pose pose_prev;
  tf::Pose pose_curr;
  tf::Transform vel;

  PointCloudT::Ptr depth_cam_pc(new PointCloudT());
  tf::Quaternion rotation;
  rotation.setRPY(-PI/2.0, 0, -PI/2.0);
  const tf::Pose camera_pose(rotation, tf::Vector3(0, 0, 0));

  int counts_publish = 0;
  int counts_visualize_map = 0;
  int counts_visualize_loc = 0;
  int counts_compress = 0;

  while (ros::ok())
  {
    // === Update PF ===
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      ROS_INFO("rbpf: new iteration");
      // initialize the time step
      last_update = now;

      octomap::Pointcloud octocloud;
      {
        std::lock_guard<std::mutex> lk(pc_mutex);
        if (pc_buff.height * pc_buff.width > 1)
        {
          pcl::fromROSMsg(pc_buff, *depth_cam_pc);
          std::vector<int> indx_map;
          pcl::removeNaNFromPointCloud(*depth_cam_pc, *depth_cam_pc, indx_map);
          for (unsigned int i = 0; i < indx_map.size(); ++i)
          {
            if (i % depth_cam_pc_downsample == 0)
            {
              tf::Vector3 point = camera_pose * tf::Vector3(
                                          depth_cam_pc->points[indx_map[i]].x,
                                          depth_cam_pc->points[indx_map[i]].y,
                                          depth_cam_pc->points[indx_map[i]].z);
              octocloud.push_back(
                octomap::point3d(point.x(), point.y(), point.z()));
            }
          }
          pc_buff.height = 0;
          pc_buff.width = 0;
        }
      }

      int index_best = 0;
      if (now > initial_update + phase_only_mapping)
      {
        // calculate the delta T
        double deltaT = (now - last_update).toSec();

        // Get sensory data (odom, depth cam)
        {
          std::lock_guard<std::mutex> lk(odom_mutex);
          pose_prev = pose_curr;
          tf::Vector3 pos(
            odom_buff.pose.pose.position.x,
            odom_buff.pose.pose.position.y,
            odom_buff.pose.pose.position.z);
          tf::Quaternion dir(
            odom_buff.pose.pose.orientation.x,
            odom_buff.pose.pose.orientation.y,
            odom_buff.pose.pose.orientation.z,
            odom_buff.pose.pose.orientation.w);
          pose_curr.setOrigin(pos);
          pose_curr.setRotation(dir);
          tf::Vector3 vel_lin(
            odom_buff.twist.twist.linear.x,
            odom_buff.twist.twist.linear.y,
            odom_buff.twist.twist.linear.z);
          tf::Quaternion vel_ang;
          vel_ang.setRPY(
            odom_buff.twist.twist.angular.x,
            odom_buff.twist.twist.angular.y,
            odom_buff.twist.twist.angular.z);
          vel.setOrigin(vel_lin);
          vel.setRotation(vel_ang);
        }

        // initialize weights and errors
        for (int i = 0; i < n_particles; ++i)
        {
          weights[i] = 0;
          errors[i] = 0;
        }

        // predict PF (use odometory)
        for (auto p: particles)
        {
          // move the particle
          p->predict(
            tf::Vector3(
              odom_buff.twist.twist.linear.x,
              odom_buff.twist.twist.linear.y,
              odom_buff.twist.twist.linear.z),
            tf::Vector3(
              odom_buff.twist.twist.angular.x,
              odom_buff.twist.twist.angular.y,
              odom_buff.twist.twist.angular.z),
            deltaT, gen);
        }

        // weight PF (use depth cam)
        double max_weight = 0;
        double weight_sum = 0;
        for (int i = 0; i < n_particles; ++i)
        {
          // Calculate a probability ranging from 0 to 1.
          weights[i] = particles[i]->evaluate(octocloud);
          weight_sum += weights[i];
          if (weights[i] > max_weight)
          {
            max_weight = weights[i];
            index_best = i;
          }
        }

        // resample PF (and update map)
        if (weight_sum != 0)
        {
          // create children population
          std::vector< std::shared_ptr<Particle> > new_generation;

          for (int i = 0; i < n_particles; ++i)
          {
            double rval = dis(gen);
            double weight_buff = 0;
            int index = 0;
            for (; index < n_particles - 1; ++index)
            {
              weight_buff += weights[index]/weight_sum;
              if (rval <= weight_buff)
                break;
            }
            // copy a particle specified by the index to the population
            new_generation.push_back(
              std::make_shared<Particle>(*particles[index]));
          }

          // Copy the children to the parents.
          particles.clear();
          particles = new_generation;
        }
      }
      // update the map
      if (octocloud.size() > 0)
      {
        for (auto p: particles)
        {
          p->update_map(octocloud);
        }
      }

      // compress maps
      if (counts_compress >= 7)
      {
        for (auto p: particles)
        {
          p->compress_map();
        }
        counts_compress = 0;
      }
      else
      {
        ++counts_compress;
      }

      // publish data
      if (counts_publish >= 5)
      {
        octomap_msgs::Octomap map;
        map.header.frame_id = "world";
        map.header.stamp = now;
        if (octomap_msgs::fullMapToMsg(*particles[index_best]->getMap(), map))
          map_pub.publish(map);
        else
          ROS_ERROR("Error serializing OctoMap");

        tf::StampedTransform tf_stamped(
          particles[index_best]->getPose(), now,
          world_frame_id, robot_frame_id);
        tf_broadcaster.sendTransform(tf_stamped);

        counts_publish = 0;
      }
      else
      {
        ++counts_publish;
      }

      // visualization
      if (counts_visualize_map >= 3)
      {
        const octomap::OcTree* m = particles[index_best]->getMap();
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

            if (m->isNodeOccupied(*it))
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
          }
        }

        // std_msgs::ColorRGBA m_color_occupied;
        // m_color_occupied.r = 1;
        // m_color_occupied.g = 1;
        // m_color_occupied.b = 0.3;
        // m_color_occupied.a = 0.5;
        for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
        {
          double size = m->getNodeSize(i);

          occupiedNodesVis.markers[i].header.frame_id = "world";
          occupiedNodesVis.markers[i].header.stamp = now;
          occupiedNodesVis.markers[i].ns = "iris";
          occupiedNodesVis.markers[i].id = i;
          occupiedNodesVis.markers[i].type
            = visualization_msgs::Marker::CUBE_LIST;
          occupiedNodesVis.markers[i].scale.x = size;
          occupiedNodesVis.markers[i].scale.y = size;
          occupiedNodesVis.markers[i].scale.z = size;

          // without this line, rviz complains orientation is uninitialized.
          occupiedNodesVis.markers[i].pose.orientation.w = 1;

          //occupiedNodesVis.markers[i].color = m_color_occupied;

          if (occupiedNodesVis.markers[i].points.size() > 0)
            occupiedNodesVis.markers[i].action
              = visualization_msgs::Marker::ADD;
          else
            occupiedNodesVis.markers[i].action
              = visualization_msgs::Marker::DELETE;
        }
        ROS_INFO("publishing markers");
        marker_occupied_pub.publish(occupiedNodesVis);
        counts_visualize_map = 0;
      }
      else
      {
        ++counts_visualize_map;
      }

      if (counts_visualize_loc >= 0)
      {
        // publish poses
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = world_frame_id;
        poseArray.header.stamp = now;
        poseArray.poses.resize(n_particles);
        for (int i = 0; i < n_particles; ++i)
        {
          tf::Pose pose = particles[i]->getPose();
          tf::Vector3 position = pose.getOrigin();
          tf::Quaternion orientation = pose.getRotation();
          poseArray.poses[i].position.x = position.x();
          poseArray.poses[i].position.y = position.y();
          poseArray.poses[i].position.z = position.z();
          poseArray.poses[i].orientation.x = orientation.x();
          poseArray.poses[i].orientation.y = orientation.y();
          poseArray.poses[i].orientation.z = orientation.z();
          poseArray.poses[i].orientation.w = orientation.w();
        }
        vis_poses_pub.publish(poseArray);

        counts_visualize_loc = 0;
      }
      else
      {
        ++counts_visualize_loc;
      }
    }

    ros::spinOnce();
  }

  return(0);
}
