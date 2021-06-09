// rbpf.cpp
// 191028
// Rao-Blackwellized Particle Filter

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mav_tunnel_nav/SrcDst.h>
#include <mav_tunnel_nav/Beacon.h>
#include <mav_tunnel_nav/Particles.h>
#include <mav_tunnel_nav/OctomapWithSegId.h>

#include <ros/ros.h>

// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>

#include <std_srvs/SetBool.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265

#include "rbpf.h"

std::string odom_topic;
std::string odom_reset_topic;
std::string pc_topic;
std::string world_frame_id;
std::string robot_frame_id;

nav_msgs::Odometry odom_buff;
std::mutex odom_mutex;

sensor_msgs::PointCloud2 pc_buff;
std::mutex pc_mutex;

std::vector<std::string> range_topics;
std::map<std::string, tf::Pose> range_poses;
std::map<std::string, double> range_buff;
std::mutex range_mutex;
double range_max, range_min;

std::mutex beacon_mutex;
std::map<std::string, mav_tunnel_nav::Beacon> beacon_buffer;
std::map<std::string, ros::Time> beacon_lasttime;

std::mutex sync_mutex;
std::string last_sync_src;
std::deque<mav_tunnel_nav::SrcDst> sync_msgs_buffer;

std::mutex data_mutex;
std::map<std::string, mav_tunnel_nav::Particles> data_buffer;
std::map<std::string, ros::Time> data_lasttime;
std::string last_data_src;
enum INTERACT_STATE
  { Init, LocalSLAM, SyncInit, DataSending, SyncReact, DataWaiting, Update };
INTERACT_STATE state = Init;

////////////////////////////////////////////////////////////////////////////////
void beaconCallback(const mav_tunnel_nav::Beacon::ConstPtr& msg)
{
  if (state != Init && msg->destination.size() > 0)
  {
    std::lock_guard<std::mutex> lk(beacon_mutex);
    beacon_buffer[msg->source] = *msg;
    beacon_lasttime[msg->source] = ros::Time::now();
  }
}

////////////////////////////////////////////////////////////////////////////////
void syncCallback(const mav_tunnel_nav::SrcDst::ConstPtr& msg)
{
  if (state == LocalSLAM)
  {
    std::lock_guard<std::mutex> lk(sync_mutex);
    sync_msgs_buffer.push_back(*msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
void dataCallback(const mav_tunnel_nav::Particles::ConstPtr& msg)
{
  if (state == SyncInit || state == DataWaiting)
  {
    std::lock_guard<std::mutex> lk(data_mutex);
    data_buffer[msg->source] = *msg;
    data_lasttime[msg->source] = ros::Time::now();
    last_data_src = msg->source;

    if (state == SyncInit)
    {
      state = DataSending;
    }
    else if (state == DataWaiting)
    {
      state = Update;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // update odom_pose by the odometry data.
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
void rangeCallback(const sensor_msgs::Range::ConstPtr& new_range)
{
  std::lock_guard<std::mutex> lk(range_mutex);
  // frame_id looks like "ray_xxxx_link". we need "xxxx" part.
  int len = new_range->header.frame_id.length();
  int pos1 = 0;
  while (pos1 < len && new_range->header.frame_id[pos1] != '_'){ ++pos1; }
  ++pos1;
  int pos2 = pos1;
  while (pos2 < len && new_range->header.frame_id[pos2] != '_'){ ++pos2; }

  // then need "range_xxxx" so "range_" is appended
  // NOTE: There may be some bug in the simulator generating very small value
  //       intermitently. At the moment, it is replaced with range_max
  if (new_range->range >= 0.09)
    range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
      = new_range->range;
  // range_buff["range_" + new_range->header.frame_id.substr(pos1, pos2 - pos1)]
  //   = new_range->range;
}

////////////////////////////////////////////////////////////////////////////////
inline int drawIndex(
  const std::vector<double>& cumul_weights, std::mt19937& gen)
{
  std::uniform_real_distribution<>
    dist(0, cumul_weights[cumul_weights.size() - 1]);

  double val = dist(gen);
  int lo = 0;
  int hi = cumul_weights.size() - 1;

  while (lo < hi - 1)
  {
    int mid = (hi + lo)/2;

    if (val > cumul_weights[mid])
    {
      lo = mid;
    }
    else
    {
      hi = mid;
    }
  }
  int indx = hi;
  if (val <= cumul_weights[lo])
    indx = lo;
  return indx;
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(
  const double &init_x, const double &init_y, const double &init_z,
  const double &init_Y, const double &resol,
  const double &probHit, const double &probMiss,
  const double &threshMin, const double &threshMax,
  const double &new_motion_noise_lin_sigma,
  const double &new_motion_noise_rot_sigma,
  const double &new_sensor_noise_range_sigma,
  const double &new_sensor_noise_depth_sigma,
  const std::shared_ptr<Particle>& newPrev):
    motion_noise_lin_sigma(new_motion_noise_lin_sigma),
    motion_noise_rot_sigma(new_motion_noise_rot_sigma),
    sensor_noise_range_sigma(new_sensor_noise_range_sigma),
    sensor_noise_depth_sigma(new_sensor_noise_depth_sigma),
    prev(newPrev)
{
  this->map = new octomap::OcTree(resol);
  this->map->setProbHit(probHit);
  this->map->setProbMiss(probMiss);
  this->map->setClampingThresMin(threshMin);
  this->map->setClampingThresMax(threshMax);

  tf::Quaternion rot_buff;
  rot_buff.setRPY(0, 0, init_Y);
  this->pose = tf::Transform(rot_buff, tf::Vector3(init_x, init_y, init_z));
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(
  const std::shared_ptr<Particle>& src, const double &resol,
  const double &probHit, const double &probMiss,
  const double &threshMin, const double &threshMax):
    motion_noise_lin_sigma(src->motion_noise_lin_sigma),
    motion_noise_rot_sigma(src->motion_noise_rot_sigma),
    sensor_noise_range_sigma(src->sensor_noise_range_sigma),
    sensor_noise_depth_sigma(src->sensor_noise_depth_sigma),
    prev(src)
{
  this->pose = src->pose;
  this->map = new octomap::OcTree(resol);
  this->map->setProbHit(probHit);
  this->map->setProbMiss(probMiss);
  this->map->setClampingThresMin(threshMin);
  this->map->setClampingThresMax(threshMax);
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(const std::shared_ptr<Particle> &src): Particle(*src)
{
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle(const Particle &src):
  motion_noise_lin_sigma(src.motion_noise_lin_sigma),
  motion_noise_rot_sigma(src.motion_noise_rot_sigma),
  sensor_noise_range_sigma(src.sensor_noise_range_sigma),
  sensor_noise_depth_sigma(src.sensor_noise_depth_sigma),
  prev(src.prev)
{
  // copy the localization data
  this->pose = src.pose;
//  this->vel_linear = src.vel_linear;

  // copy the mapping data
  this->map = new octomap::OcTree(*src.map);
}

////////////////////////////////////////////////////////////////////////////////
Particle::Particle():
  Particle(
    0.0, 0.0, 0.0, 0.0,
    0.25, 0.7, 0.4, 0.12, 0.97,
    0.05, 0.02,
    1.0, 1.0){}

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
// const tf::Vector3 Particle::getVel()
// {
//   return this->vel_linear;
// }
////////////////////////////////////////////////////////////////////////////////
const octomap::OcTree* Particle::getMap()
{
  return this->map;
}

////////////////////////////////////////////////////////////////////////////////
void Particle::initPosition(const tf::Vector3 &position)
{
  this->pose.setOrigin(position);
}

////////////////////////////////////////////////////////////////////////////////
void Particle::initOrientation(const tf::Quaternion &orientation)
{
  this->pose.setRotation(orientation);
}

////////////////////////////////////////////////////////////////////////////////
void Particle::predict(
  const tf::Vector3 &delta_pos, const tf::Quaternion &delta_rot,
  std::mt19937 &gen)
{
  std::normal_distribution<> motion_noise_lin(0, motion_noise_lin_sigma);
  tf::Vector3 delta_pos_noise(
    delta_pos.x() + motion_noise_lin(gen),
    delta_pos.y() + motion_noise_lin(gen),
    delta_pos.z() + motion_noise_lin(gen));
  std::normal_distribution<> motion_noise_rot(0, motion_noise_rot_sigma);
  tf::Quaternion delta_rot_noise(
    tf::Vector3(0,0,1), motion_noise_rot(gen));
  this->pose
    = this->pose
      * tf::Transform(delta_rot, delta_pos_noise)
      * tf::Transform(delta_rot_noise, tf::Vector3(0, 0, 0));
}

////////////////////////////////////////////////////////////////////////////////
double Particle::evaluate(
  const std::map<std::string, double> &range_data,
  const octomap::Pointcloud &scan, const bool use_prev)
{
  octomap::OcTree *map2use;
  if (use_prev)
  {
    if (this->prev)
      map2use = this->prev->map;
    else
      ROS_ERROR("No previous particle!");
  }
  else
  {
    map2use = this->map;
  }

  double log_lik_rng = 0;
  int hits_rng = 0;
  // evaluation by range data
  for (auto range_name: range_topics)
  {
    auto data = range_data.find(range_name);
    if (data == range_data.end())
      continue;
    double dist = data->second;
    if (dist >= range_max || dist <= range_min)
      continue;
    tf::Vector3 tf_pos = range_poses[range_name].getOrigin();
    tf::Vector3 tf_sens(dist, 0, 0);
    tf::Vector3 tf_target = range_poses[range_name] * tf_sens;

    octomap::point3d oct_pos(tf_pos.x(), tf_pos.y(), tf_pos.z());
    octomap::point3d oct_target(tf_target.x(), tf_target.y(), tf_target.z());
    octomap::point3d direction = oct_pos - oct_target;
    octomap::point3d hit;
    if (map2use->castRay(oct_target, direction, hit,
        true, dist)) //ignoreUnknownCells = true, maxRange
    {
      // if (this->map->coordToKeyChecked(hit, key) &&
      //  (node = this->map->search(key,0 /*depth*/)))
      // {
        double err = (oct_target - hit).norm();

        if (err > 0.2)
        {
          log_lik_rng +=
            -std::log(
              2*3.14159*sensor_noise_range_sigma*sensor_noise_range_sigma)/2.0
            -err*err/sensor_noise_range_sigma/sensor_noise_range_sigma/2.0;
        }
        else
        {
          log_lik_rng +=
            -std::log(
              2*3.14159*sensor_noise_range_sigma*sensor_noise_range_sigma)/2.0;
        }
        ++hits_rng;
      // }
    }
    else if (map2use->castRay(oct_target, -direction, hit,
        true, 2.0)) //ignoreUnknownCells = true, maxRange
    {
      // if (this->map->coordToKeyChecked(hit, key) &&
      //  (node = this->map->search(key,0 /*depth*/)))
      // {
        double err = (oct_target - hit).norm();

        if (err > 0.2)
        {
          log_lik_rng +=
            -std::log(
              2*3.14159*sensor_noise_range_sigma*sensor_noise_range_sigma)/2.0
            -err*err/sensor_noise_range_sigma/sensor_noise_range_sigma/2.0;
        }
        else
        {
          log_lik_rng +=
            -std::log(
              2*3.14159*sensor_noise_range_sigma*sensor_noise_range_sigma)/2.0;
        }
        ++hits_rng;
      // }
    }
  }

  double log_lik = 0;
  int hits = 0;
  tf::Pose sens_pose = this->pose;
  // for all point in the point cloud
  octomap::OcTreeKey key;
  //octomap::OcTreeNode *node;
  int offset = scan.size()/30;
  for (unsigned int ip = 0; ip < scan.size(); ip += offset)
  {
    tf::Vector3 tf_pos = sens_pose.getOrigin();
    tf::Vector3 tf_sens = tf::Vector3(scan[ip].x(), scan[ip].y(), scan[ip].z());
    tf::Vector3 tf_target
          = sens_pose * tf_sens;

    double dist = tf_sens.length();
    if (dist < 0.5)
      continue;
    octomap::point3d oct_pos(tf_pos.x(), tf_pos.y(), tf_pos.z());
    octomap::point3d oct_target(tf_target.x(), tf_target.y(), tf_target.z());
    octomap::point3d direction = oct_pos - oct_target;
    octomap::point3d hit;
    if (map2use->castRay(oct_target, direction, hit,
        true, dist)) //ignoreUnknownCells = true, maxRange
    {
      // if (this->map->coordToKeyChecked(hit, key) &&
      //  (node = this->map->search(key,0 /*depth*/)))
      // {
        double err = (oct_target - hit).norm();
        if (err > 0.2)
        {

          log_lik +=
            -std::log(
              2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0
            -err*err/sensor_noise_depth_sigma/sensor_noise_depth_sigma/2.0;
        }
        else
        {
          log_lik +=
            -std::log(
              2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0;
        }
        ++hits;
      // }
    }
    else if (map2use->castRay(oct_target, -direction, hit,
            true, 2.0)) //ignoreUnknownCells = true, maxRange
    {
      // if (this->map->coordToKeyChecked(hit, key) &&
      //  (node = this->map->search(key,0 /*depth*/)))
      // {
      double err = (oct_target - hit).norm();
      if (err > 0.2)
      {

        log_lik +=
          -std::log(
            2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0
          -err*err/sensor_noise_depth_sigma/sensor_noise_depth_sigma/2.0;
      }
      else
      {
        log_lik +=
          -std::log(
            2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0;
      }
      ++hits;
      // }
    }
  }

  // for (unsigned int ip = 0; ip < scan.size(); ++ip)
  // {
  //   tf::Vector3 point
  //         = sens_pose * tf::Vector3(scan[ip].x(), scan[ip].y(), scan[ip].z());
  //   if (this->map->coordToKeyChecked(
  //         point.x(), point.y(), point.z(), key)
  //       && (node = this->map->search(key,0)))
  //   {
  //     log_lik += std::log(node->getOccupancy());
  //     ++hits;
  //   }
  // }
  // return 0 if the major part of the point cloud landed on unknown cells.
  //ROS_DEBUG_STREAM("hits: " << hits);
  if (hits_rng > 0)
    log_lik_rng /= hits_rng;
  if (hits > 0)
    log_lik /= hits;
  return (hits_rng + hits > 0)? std::exp(log_lik_rng + log_lik): 0;
}

////////////////////////////////////////////////////////////////////////////////
void Particle::update_map(const octomap::Pointcloud &scan)
{
  octomath::Vector3 sensor_org(0, 0, 0);
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
inline void prepareDataMsg(
  mav_tunnel_nav::Particles& data_msg, std::string& destination,
  std::vector<double>& cumul_weights, std::vector<double>& cumul_weights_comp,
  const double& conserv_omega, const double& sigma_kde_squared_x2,
  const std::vector< std::shared_ptr<Particle> >& particles,
  const int& Nref, std::mt19937& gen_cooploc,
  const bool enable_conservative = true)
{
  const int n_particles = particles.size();

  // set the destination
  data_msg.destination = destination;

  if (enable_conservative)
  {
    // calculate the weights
    for (int i = 0; i < n_particles; ++i)
    {
      cumul_weights[i] = 1.0;
    }
    for (int i = 0; i < n_particles; ++i)
    {
      // KDE: kernel density estimation.
      // estimate the value on the point of the probability distribution
      for (int j = i + 1; j < n_particles; ++j)
      {
        double diff
          = (particles[i]->getPose().getOrigin()
            - particles[j]->getPose().getOrigin()).length();
        double val = exp(-diff*diff/sigma_kde_squared_x2);
        cumul_weights[i] += val;
        cumul_weights[j] += val;
      }
      double buff = pow(cumul_weights[i], conserv_omega);

      // p^omega / p = p^(omega - 1)
      cumul_weights[i] = buff / cumul_weights[i];
      if (i > 0)
      {
        cumul_weights[i] += cumul_weights[i-1];
      }
      // p^(1-omega) / p = p^(-omega) = 1 / p^omega
      cumul_weights_comp[i] = 1.0 / buff;
      if (i > 0)
      {
        cumul_weights_comp[i] += cumul_weights_comp[i-1];
      }
    }
  }
  else
  {
    // without conservative method
    // i.e., the weights are uniform
    for (int i = 0; i < n_particles; ++i)
    {
      cumul_weights[i] = i + 1;
      cumul_weights_comp[i] = i + 1;
    }
  }

  data_msg.particles.resize(Nref);
  data_msg.cumul_weights.resize(Nref);
  for (int i = 0; i < Nref; ++i)
  {
    // get a particle of the other robot by msg.cumul_weights
    int indx = drawIndex(cumul_weights_comp, gen_cooploc);
    // NOTE: set the weight
    // (just in case. Maybe this part is no longer necessary as the particles
    // have already been resampled)
    data_msg.cumul_weights[i] = cumul_weights_comp[indx];
    // set the particle's poses.
    tf::Vector3 pos = particles[indx]->getPose().getOrigin();
    tf::Quaternion rot = particles[indx]->getPose().getRotation();
    data_msg.particles[i].position.x = pos.x();
    data_msg.particles[i].position.y = pos.y();
    data_msg.particles[i].position.z = pos.z();
    data_msg.particles[i].orientation.w = rot.w();
    data_msg.particles[i].orientation.x = rot.x();
    data_msg.particles[i].orientation.y = rot.y();
    data_msg.particles[i].orientation.z = rot.z();
  }
}

////////////////////////////////////////////////////////////////////////////////
void pf_main()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get this robot's name
  std::string robot_name;
  if (!pnh.getParam("robot_name", robot_name))
    ROS_ERROR_STREAM("no ros parameter: robot_name");

  double comm_range;
  if (!nh.getParam("/comm_range", comm_range))
    ROS_ERROR_STREAM("no parameter for rbpf: comm_range");

  // publisher for synchronization of exchange
  std::string sync_up_topic;
  ros::Publisher sync_pub;
  if (nh.getParam("/sync_up_topic", sync_up_topic)) // global param
  {
    sync_pub = nh.advertise<mav_tunnel_nav::SrcDst>(sync_up_topic, 1);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: sync_up_topic");
  }

  // publisher for data exchange
  std::string data_up_topic;
  ros::Publisher data_pub;
  if (nh.getParam("/data_up_topic", data_up_topic)) // global param
  {
    data_pub = nh.advertise<mav_tunnel_nav::Particles>(data_up_topic, 1);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: data_up_topic");
  }

  // random numbers
  //std::random_device rd{};
  //std::mt19937 gen{rd()};
  int seed_indivloc;
  if (!pnh.getParam("seed_indivloc", seed_indivloc))
  ROS_ERROR_STREAM("no param: seed_indivloc");
  std::mt19937 gen_indivloc{seed_indivloc};

  //std::uniform_real_distribution<> dis(0, 1.0);

  tf::TransformBroadcaster tf_broadcaster;

  if(!pnh.getParam("odom_reset_topic", odom_reset_topic))
    ROS_ERROR_STREAM("no param: odom_reset_topic");
  ros::Publisher odom_reset_pub
    = nh.advertise<nav_msgs::Odometry>(odom_reset_topic, 1);

  std::string octomap_topic;
  if(!pnh.getParam("octomap_topic", octomap_topic))
    ROS_ERROR_STREAM("no param: octomap_topic");
  ros::Publisher map_pub
    = nh.advertise<mav_tunnel_nav::OctomapWithSegId>(octomap_topic, 1);
  ros::Publisher marker_occupied_pub
    = nh.advertise<visualization_msgs::MarkerArray>("map_marker_occupied", 1);
  ros::Publisher vis_poses_pub
    = nh.advertise<geometry_msgs::PoseArray>("loc_vis_poses", 1, true);

  if(!pnh.getParam("world_frame_id", world_frame_id))
    ROS_ERROR_STREAM("no param: world_frame_id");
  if(!pnh.getParam("robot_frame_id", robot_frame_id))
    ROS_ERROR_STREAM("no param: robot_frame_id");

  // === Initialize PF ===
  int n_particles;
  double update_freq;
  if(!pnh.getParam("n_particles", n_particles))
    ROS_ERROR_STREAM("no param: n_particles");
  if(!pnh.getParam("update_freq", update_freq))
    ROS_ERROR_STREAM("no param: update_freq");
  const ros::Duration update_phase(1.0/update_freq);

  int depth_cam_pc_downsample;
  if(!pnh.getParam("depth_cam_pc_downsample", depth_cam_pc_downsample))
    ROS_ERROR_STREAM("no param: depth_cam_pc_downsample");

  double init_x;
  double init_y;
  double init_z;
  double init_Y;
  double resol;
  double probHit;
  double probMiss;
  double threshMin;
  double threshMax;
  if(!pnh.getParam("init_x", init_x))
    ROS_ERROR_STREAM("no param: init_x");
  if(!pnh.getParam("init_y", init_y))
    ROS_ERROR_STREAM("no param: init_y");
  if(!pnh.getParam("init_z", init_z))
    ROS_ERROR_STREAM("no param: init_z");
  if(!pnh.getParam("init_Y", init_Y))
    ROS_ERROR_STREAM("no param: init_Y");
  if(!pnh.getParam("map_resol", resol))
    ROS_ERROR_STREAM("no param: map_resol");
  if(!pnh.getParam("map_probHit", probHit))
    ROS_ERROR_STREAM("no param: map_probHit");
  if(!pnh.getParam("map_probMiss", probMiss))
    ROS_ERROR_STREAM("no param: map_probMiss");
  if(!pnh.getParam("map_threshMin", threshMin))
    ROS_ERROR_STREAM("no param: map_threshMin");
  if(!pnh.getParam("map_threshMax", threshMax))
    ROS_ERROR_STREAM("no param: map_threshMax");
  double motion_noise_lin_sigma;
  if(!pnh.getParam("motion_noise_lin_sigma", motion_noise_lin_sigma))
    ROS_ERROR_STREAM("no param: motion_noise_lin_sigma");
  double motion_noise_rot_sigma;
  if(!pnh.getParam("motion_noise_rot_sigma", motion_noise_rot_sigma))
    ROS_ERROR_STREAM("no param: motion_noise_rot_sigma");
  double sensor_noise_range_sigma;
  if(!pnh.getParam("sensor_noise_range_sigma", sensor_noise_range_sigma))
    ROS_ERROR_STREAM("no param: sensor_noise_range_sigma");
  double sensor_noise_depth_sigma;
  if(!pnh.getParam("sensor_noise_depth_sigma", sensor_noise_depth_sigma))
    ROS_ERROR_STREAM("no param: sensor_noise_depth_sigma");

  double t_pose_adjust;
  if(!pnh.getParam("t_pose_adjust", t_pose_adjust))
    ROS_ERROR_STREAM("no param: t_pose_adjust");
  const ros::Duration phase_pose_adjust(t_pose_adjust);
  double t_only_mapping;
  if(!pnh.getParam("t_only_mapping", t_only_mapping))
    ROS_ERROR_STREAM("no param: t_only_mapping");
  const ros::Duration phase_only_mapping(t_only_mapping);
  int mapping_interval;
  if(!pnh.getParam("mapping_interval", mapping_interval))
    ROS_ERROR_STREAM("no param: mapping_interval");
  int publish_interval;
  if(!pnh.getParam("publish_interval", publish_interval))
    ROS_ERROR_STREAM("no param: publish_interval");
  int vismap_interval;
  if(!pnh.getParam("vismap_interval", vismap_interval))
    ROS_ERROR_STREAM("no param: vismap_interval");
  int visloc_interval;
  if(!pnh.getParam("visloc_interval", visloc_interval))
    ROS_ERROR_STREAM("no param: visloc_interval");
  int compress_interval;
  if(!pnh.getParam("compress_interval", compress_interval))
    ROS_ERROR_STREAM("no param: compress_interval");
  bool enable_indivLoc;
  if(!pnh.getParam("enable_indivLoc", enable_indivLoc))
    ROS_ERROR_STREAM("no param: enable_indivLoc");
  // int locdata_interval;
  // pnh.getParam("locdata_interval", locdata_interval);

  //       each vector of particles represent a segment.
  std::vector< std::vector< std::shared_ptr<Particle> > > segments(1);
  int iseg = 0;
  for (int i = 0; i < n_particles; ++i)
  {
    segments[iseg].push_back(
      std::make_shared<Particle>(
        init_x, init_y, init_z, init_Y,
        resol, probHit, probMiss, threshMin, threshMax,
        motion_noise_lin_sigma, motion_noise_rot_sigma,
        sensor_noise_range_sigma, sensor_noise_depth_sigma));
  }
  std::vector< int > segments_index_best(1);
  std::vector<double> cumul_weights_slam(n_particles);
  std::vector<double> errors(n_particles);
  tf::Pose init_segment_pose;
  ros::Time init_segment_time;

  tf::Pose pose_prev;
  tf::Pose pose_curr;
  //tf::Transform vel;

  PointCloudT::Ptr depth_cam_pc(new PointCloudT());
  tf::Quaternion rotation;
  rotation.setRPY(-PI/2.0, 0, -PI/2.0);
  //const tf::Pose camera_pose(rotation, tf::Vector3(0, 0, 0));
  const tf::Pose camera_pose(rotation, tf::Vector3(0.03, 0, -0.06));

  std::shared_ptr<tf::TransformListener> tf_listener
    = std::make_shared<tf::TransformListener>();
  bool save_traj = false;
  std::string traj_filename;
  if (pnh.getParam("save_traj", save_traj))
  {
    traj_filename
      = "./" + robot_name + "_"
        + std::to_string((int)(ros::WallTime::now().toSec()))
        + "_traj.txt";
  }
  else
  {
    save_traj = false;
  }

  int counts_publish = 0;
  int counts_visualize_map = 0;
  int counts_visualize_loc = 0;
  int counts_map_update = 0;
  int counts_compress = 0;
  // int counts_locdata = 0;

  std::uniform_int_distribution<int> dwnsmp_start(0, depth_cam_pc_downsample-1);

  ros::Time last_update = ros::Time::now();
  // wait for the tf of initial ground truth
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    if (now > last_update + update_phase)
    {
      // initialize the time step
      last_update = now;
      tf::StampedTransform ground_truth_tf;
      try
      {
        tf_listener->waitForTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ros::Duration(1));
        tf_listener->lookupTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ground_truth_tf);

        break;
      }
      catch (tf::TransformException& ex)
      {
        // NOTE: do nothing. Just wait.
      }
    }
  }

  // initialize the estimated pose
  ros::Time initial_update = ros::Time::now();
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    if (now > initial_update + phase_pose_adjust)
    {
      // initialize the time step
      tf::StampedTransform ground_truth_tf;
      try
      {
        tf_listener->waitForTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ros::Duration(1));
        tf_listener->lookupTransform(
          world_frame_id, robot_name + "_groundtruth",
          ros::Time(0), ground_truth_tf);

        tf::Vector3 position = ground_truth_tf.getOrigin();
        tf::Quaternion orientation = ground_truth_tf.getRotation();
        for (auto p: segments[iseg])
        {
          p->initPosition(position);
          p->initOrientation(orientation);
        }
        init_segment_pose = ground_truth_tf;
        break;
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR_STREAM(
          "Transfrom from " << robot_name + "_groundtruth" <<
          " to " << world_frame_id << " is not available yet.");
      }
    }
  }

  // Initialization of prev pose
  // Odometry data
  tf::Transform diff_pose;
  {
    std::lock_guard<std::mutex> lk(odom_mutex);
    tf::Vector3 pos(
      odom_buff.pose.pose.position.x,
      odom_buff.pose.pose.position.y,
      odom_buff.pose.pose.position.z);
    tf::Quaternion dir(
      odom_buff.pose.pose.orientation.x,
      odom_buff.pose.pose.orientation.y,
      odom_buff.pose.pose.orientation.z,
      odom_buff.pose.pose.orientation.w);
    pose_prev.setOrigin(pos);
    pose_prev.setRotation(dir);
  }

  // parameters for cooperative localization
  int Nref;
  if (!pnh.getParam("Nref", Nref))
    ROS_ERROR_STREAM("no param: Nref");
  int seed_cooploc;
  if (!pnh.getParam("seed_cooploc", seed_cooploc))
    ROS_ERROR_STREAM("no param: seed_cooploc");
  bool enable_cooploc;
  if (!pnh.getParam("enable_cooploc", enable_cooploc))
    ROS_ERROR_STREAM("no param: enable_cooploc");
  bool enable_conservative;
  if (!pnh.getParam("enable_conservative", enable_conservative))
    ROS_ERROR_STREAM("no param: enable_conservative");

  double conserv_omega;
  if (!pnh.getParam("conserv_omega", conserv_omega))
    ROS_ERROR_STREAM("no param: conserv_omega");
  double sigma_kde;
  if (!pnh.getParam("sigma_kde", sigma_kde))
    ROS_ERROR_STREAM("no param: sigma_kde");

  double sigmaLocR;
  if (!pnh.getParam("sigmaLocR", sigmaLocR))
    ROS_ERROR_STREAM("no param: sigmaLocR");
  double sigmaLocT;
  if (!pnh.getParam("sigmaLocT", sigmaLocT))
    ROS_ERROR_STREAM("no param: sigmaLocT");
  // parameters for evaluation
  double gl_eval_cons;
  if (!pnh.getParam("gl_eval_cons", gl_eval_cons))
    ROS_ERROR_STREAM("no param: gl_eval_cons");
  double ml_eval_cons;
  if (!pnh.getParam("ml_eval_cons", ml_eval_cons))
    ROS_ERROR_STREAM("no param: ml_eval_cons");

  double beacon_lifetime_buff;
  if (!pnh.getParam("beacon_lifetime", beacon_lifetime_buff))
    ROS_ERROR_STREAM("no param: beacon_lifetime");
  const ros::Duration beacon_lifetime(beacon_lifetime_buff);
  double cooploc_phase_buff;
  if (!pnh.getParam("cooploc_phase", cooploc_phase_buff))
    ROS_ERROR_STREAM("no param: cooploc_phase");
  const ros::Duration cooploc_phase(cooploc_phase_buff);
  double syncinit_timeout_buff;
  if (!pnh.getParam("syncinit_timeout", syncinit_timeout_buff))
    ROS_ERROR_STREAM("no param: syncinit_timeout");
  const ros::Duration syncinit_timeout(syncinit_timeout_buff);

  std::mt19937 gen_cooploc;
  gen_cooploc.seed(seed_cooploc);

  std::default_random_engine gen_cooploc_select;

  ros::Time last_cooploc = ros::Time::now();
  ros::Time last_syncinit;

  bool enable_segmentation;
  if (!pnh.getParam("enable_segmentation", enable_segmentation))
    ROS_ERROR_STREAM("no param: enable_segmentation");
  double init_seg_phase_buff;
  if (!pnh.getParam("init_seg_phase", init_seg_phase_buff))
    ROS_ERROR_STREAM("no param: init_seg_phase");
  ros::Duration init_seg_phase(init_seg_phase_buff);
  double next_seg_thresh;
  if (!pnh.getParam("next_seg_thresh", next_seg_thresh))
    ROS_ERROR_STREAM("no param: next_seg_thresh");
  bool enable_clr4seg;
  if (!pnh.getParam("enable_clr4seg", enable_clr4seg))
    ROS_ERROR_STREAM("no param: enable_clr4seg");

  // === For data exchange. ==
  // 95 % of difference should be in approx. 2.7 * sigma_kde
  // https://stats.stackexchange.com/questions/35012/mahalanobis-distance-and-percentage-of-the-distribution-represented
  const double sigma_kde_squared_x2 = 2 * sigma_kde * sigma_kde;
  mav_tunnel_nav::Particles data_msg;
  data_msg.source = robot_name;
  std::vector<double> cumul_weights(n_particles);
  std::vector<double> cumul_weights_comp(n_particles);

  // For synchronization
  mav_tunnel_nav::SrcDst sync_msg;
  sync_msg.source = robot_name;

  initial_update = ros::Time::now();
  last_update = initial_update;
  // the main loop
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();

    if (state == SyncInit)
    {
      // NOTE: If timed out, it will switch back to LocalSLAM. Otherwise, the
      //       state is switched to DataSending by the callback once the data
      //       from the other robot is received.
      if (now >= last_syncinit + syncinit_timeout)
      {
        state = LocalSLAM;
      }
    }
    else if (state == SyncReact || state == DataSending)
    {
      // Odometry data
      tf::Transform diff_pose;
      {
        std::lock_guard<std::mutex> lk(odom_mutex);
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
        diff_pose = pose_prev.inverse() * pose_curr;
        pose_prev = pose_curr;
      }
      // predict PF (use odometory)
      const tf::Vector3 delta_pos = diff_pose.getOrigin();
      const tf::Quaternion delta_rot = diff_pose.getRotation();
      for (auto p: segments[iseg])
      {
        // move the particle
        // call predict with the relative pose.
        p->predict(delta_pos, delta_rot, gen_indivloc);
      }

      std::string dest;
      if (state == SyncReact)
        dest = last_sync_src;
      else
        dest = last_data_src;

      prepareDataMsg(
        data_msg, dest, cumul_weights, cumul_weights_comp,
        conserv_omega, sigma_kde_squared_x2, segments[iseg], Nref, gen_cooploc,
        enable_conservative);

      // send data to the other
      data_pub.publish(data_msg);

      if (state == SyncReact)
        state = DataWaiting;
      else
        state = Update;
    }
    else if (state == DataWaiting)
    {
      // NOTE: DataWaiting: do nothing in the main loop.
      //       the state is switched to Update by the callback once the
      //       data from the other robot is received.
      //
      //       If it takes too long, there must be some problem.
      //       In such a case, print out an error and returns to the initial
      //       state.
      if (now >= last_syncinit + syncinit_timeout)
      {
        ROS_ERROR_STREAM("" << robot_name << ": timeout in DataWaiting");
        state = LocalSLAM;
      }
    }
    else if (state == Update)
    {
      mav_tunnel_nav::Particles msg;
      {
        std::lock_guard<std::mutex> lk(data_mutex);
        msg = data_buffer[last_data_src];
        //data_lasttime[last_data_src]
      }
      // === calculate weights for resampling === //
      // so what it has at this point?
      // cumul_weights
      // msg.estimated_distance
      // msg.estimated_orientation
      // msg.cumul_weights
      // msg.particles

      // convert geometry_msgs::Point to tf::Vector3.
      tf::Vector3 msg_estimated_orientation(
        msg.estimated_orientation.x,
        msg.estimated_orientation.y,
        msg.estimated_orientation.z);

      // cumulative weights for resampling.
      std::vector<double> cumul_weights_update(n_particles);

      double weight_max = 0;
      int index_best = -1;

      // for all particles
      for (int ip = 0; ip < n_particles; ++ip)
      {
        // initialize the wegith
        cumul_weights_update[ip] = 0;
        // get the particle's pose
        tf::Pose robot_pose = segments[iseg][ip]->getPose();
        // for Nref
        for (unsigned int i = 0; i < msg.particles.size(); ++i)
        {
          // get a particle of the other robot by msg.cumul_weights
          auto neighbor_pose_msg = msg.particles[i];
          tf::Pose neighbor_pose(
            tf::Quaternion(
              neighbor_pose_msg.orientation.x,
              neighbor_pose_msg.orientation.y,
              neighbor_pose_msg.orientation.z,
              neighbor_pose_msg.orientation.w),
            tf::Vector3(
              neighbor_pose_msg.position.x,
              neighbor_pose_msg.position.y,
              neighbor_pose_msg.position.z));
          // simulate a measurement based on the sampled poses.
          tf::Vector3 sampled_loc
            = (neighbor_pose.inverse() * robot_pose).getOrigin();
          double sampled_range = sampled_loc.length();
          tf::Vector3 sampled_orientation = sampled_loc / sampled_range;

          // difference from the actual sensory data
          double diff_range = sampled_range - msg.estimated_distance;
          double diff_rad
            = std::acos(sampled_orientation.dot(msg_estimated_orientation));

          // calculate weight and add it
          if (msg.particles.size() == 1) // in case of global
          {
            cumul_weights_update[ip]
              += std::exp(
                  -(diff_range*diff_range)
                    /sigmaLocR/sigmaLocR/gl_eval_cons
                  -(diff_rad*diff_rad)
                    /sigmaLocT/sigmaLocT/gl_eval_cons);
          }
          else
          {
            cumul_weights_update[ip]
              += std::exp(
                  -(diff_range*diff_range)
                    /sigmaLocR/sigmaLocR/ml_eval_cons
                  -(diff_rad*diff_rad)
                    /sigmaLocT/sigmaLocT/ml_eval_cons);
          }
        }

        //   multiply with the original weights
        cumul_weights_update[ip]
          *= ((ip > 0)? cumul_weights[ip] - cumul_weights[ip-1]:
                        cumul_weights[0]);

        if (index_best == -1 || cumul_weights_update[ip] > weight_max)
        {
          index_best = ip;
          weight_max = cumul_weights_update[ip];
        }

        // make it cumuluative
        if (ip > 0)
          cumul_weights_update[ip] += cumul_weights_update[ip - 1];
      }

      // resampling
      std::vector<int> indx_list(n_particles);
      for (int ip = 0; ip < n_particles; ++ip)
      {
        indx_list[ip] = drawIndex(cumul_weights_update, gen_cooploc);
      }

      std::vector< std::shared_ptr<Particle> > new_generation;
      std::vector<int> indx_unused(n_particles, -1);
      for (int ip = 0; ip < n_particles; ++ip)
      {
        const int prev_indx = indx_unused[indx_list[ip]];
        if (prev_indx == -1)
        {
          new_generation.push_back(std::move(segments[iseg][indx_list[ip]]));
          indx_unused[indx_list[ip]] = ip;
        }
        else
        {
          new_generation.push_back(
            std::make_shared<Particle>(*new_generation[prev_indx]));
        }
      }
      segments[iseg].swap(new_generation);

      {
        // publish the location
        tf::StampedTransform tf_stamped(
          segments[iseg][index_best]->getPose(), now,
          world_frame_id, robot_frame_id);
        tf_broadcaster.sendTransform(tf_stamped);
      }

      {
        // publish poses
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = world_frame_id;
        poseArray.header.stamp = now;
        poseArray.poses.resize(n_particles);
        for (int i = 0; i < n_particles; ++i)
        {
          tf::Pose pose = segments[iseg][i]->getPose();
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
      }

      if (!enable_indivLoc)
      {
        // if the individual localization is disabled,
        // perform mapping and segmentation here.
        // if far away from the init position of the segment
        octomap::Pointcloud octocloud;
        {
          // Depthcam data
          std::lock_guard<std::mutex> lk(pc_mutex);
          if (pc_buff.height * pc_buff.width > 1)
          {
            pcl::fromROSMsg(pc_buff, *depth_cam_pc);

            pcl::VoxelGrid<PointT> downSizeFilter;
            downSizeFilter.setLeafSize(resol, resol, resol);
            downSizeFilter.setInputCloud(depth_cam_pc);
            downSizeFilter.filter(*depth_cam_pc);
            for (unsigned int i = 0; i < depth_cam_pc->points.size(); ++i)
            {
              tf::Vector3 point = camera_pose * tf::Vector3(
                                          depth_cam_pc->points[i].x,
                                          depth_cam_pc->points[i].y,
                                          depth_cam_pc->points[i].z);
              octocloud.push_back(
                octomap::point3d(point.x(), point.y(), point.z()));
            }
            pc_buff.height = 0;
            pc_buff.width = 0;
          }
        }
        bool do_segment = false;
        tf::Pose best_pose;
        if (enable_segmentation)
        {
          best_pose = segments[iseg][index_best]->getPose();
          tf::Pose pose_in_seg = init_segment_pose.inverse() * best_pose;
          if (pose_in_seg.getOrigin().length() > next_seg_thresh)
            do_segment = true;
        }

        if (do_segment)
        {
          // create a new segment
          std::vector< std::shared_ptr<Particle> > new_seg(
            n_particles, nullptr);
          segments_index_best.push_back(index_best);
          // copy each resampled particle.
          for (int i = 0; i < n_particles; ++i)
          {
            int indx = i;

            new_seg[i]
              = std::make_shared<Particle>(
                  segments[iseg][indx],
                  resol, probHit, probMiss, threshMin, threshMax);
          }
          // build a map anyway
          if (octocloud.size() > 0)
          {
            for (int i = 0; i < n_particles; ++i)
              new_seg[i]->update_map(octocloud);
          }
          counts_map_update = 0;
          // add the segment to the list.
          segments.push_back(new_seg);
          // increment iseg
          ++iseg;
          // set the initial pose of the segment to that of the best particle.
          init_segment_pose = best_pose;
          // set the initial time of the segment to the current one.
          init_segment_time = now;
        }
        else if (counts_map_update >= mapping_interval && octocloud.size() > 0)
        {
          for (auto p: segments[iseg])
          {
            p->update_map(octocloud);
          }
          counts_map_update = 0;
        }
        else
        {
          ++counts_map_update;
        }
      }

      last_cooploc = now;
      state = LocalSLAM;
    }
    else if (state == LocalSLAM && now <= last_update + update_phase) // in the default state
    {
      if (enable_cooploc)
      {
        if (now < last_cooploc + cooploc_phase)
        {
          std::lock_guard<std::mutex> lk(sync_mutex);
          while(state == LocalSLAM && sync_msgs_buffer.size() > 0)
          {
            mav_tunnel_nav::SrcDst msg = sync_msgs_buffer.front();
            sync_msgs_buffer.pop_front();
            if (now - msg.stamp < syncinit_timeout * 0.7)
            {
              last_sync_src = msg.source;
              state = SyncReact;
              last_syncinit = now;
            }
          }
        }
        else
        {
          // decide if it should initiate interactions with a neighbor.

          // list up candidates to sync.
          std::vector<std::string> candidates;
          for (auto p: beacon_lasttime)
          {
            // NOTE: add an candiate if the packet is "fresh" enough and it is
            //       within 90% of comm range.
            if (now <= p.second + beacon_lifetime
              && beacon_buffer[p.first].estimated_distance < 0.9 * comm_range)
            {
              candidates.push_back(p.first);
            }
          }

          if (candidates.size() > 0)
          {
            // randomly select a neighbor to interact.
            std::uniform_int_distribution<int> dist(0, candidates.size() - 1);

            // send a sync packet.
            sync_msg.stamp = ros::Time::now();
            sync_msg.destination = candidates[dist(gen_cooploc_select)];
            sync_pub.publish(sync_msg);

            last_syncinit = now;
            state = SyncInit;
          }
        }
      }
    }
    else // perform the local SLAM
    {
      // initialize the time step
      last_update = now;

      std::map<std::string, double> range_data;
      {
        std::lock_guard<std::mutex> lk(range_mutex);
        for (auto item: range_buff)
        {
          range_data[item.first] = item.second;
        }
      }
      octomap::Pointcloud octocloud;
      {
        // Depthcam data
        std::lock_guard<std::mutex> lk(pc_mutex);
        if (pc_buff.height * pc_buff.width > 1)
        {
          pcl::fromROSMsg(pc_buff, *depth_cam_pc);

          pcl::VoxelGrid<PointT> downSizeFilter;
          downSizeFilter.setLeafSize(resol, resol, resol);
          downSizeFilter.setInputCloud(depth_cam_pc);
          downSizeFilter.filter(*depth_cam_pc);
          for (unsigned int i = 0; i < depth_cam_pc->points.size(); ++i)
          {
            // if (pcl_isfinite(depth_cam_pc->points[i].x) &&
            //     pcl_isfinite(depth_cam_pc->points[i].y) &&
            //     pcl_isfinite(depth_cam_pc->points[i].z))
            // {
              tf::Vector3 point = camera_pose * tf::Vector3(
                                          depth_cam_pc->points[i].x,
                                          depth_cam_pc->points[i].y,
                                          depth_cam_pc->points[i].z);
              octocloud.push_back(
                octomap::point3d(point.x(), point.y(), point.z()));
            // }
          }
          // std::vector<int> indices;
          // pcl::removeNaNFromPointCloud(*depth_cam_pc, *depth_cam_pc, indices);
          //
          // for (unsigned int i = 0; i < indices.size(); ++i)
          // {
          //   // if (pcl_isfinite(depth_cam_pc->points[i].x) &&
          //   //     pcl_isfinite(depth_cam_pc->points[i].y) &&
          //   //     pcl_isfinite(depth_cam_pc->points[i].z))
          //   // {
          //     tf::Vector3 point = camera_pose * tf::Vector3(
          //                                 depth_cam_pc->points[indices[i]].x,
          //                                 depth_cam_pc->points[indices[i]].y,
          //                                 depth_cam_pc->points[indices[i]].z);
          //     octocloud.push_back(
          //       octomap::point3d(point.x(), point.y(), point.z()));
          //   // }
          // }
          pc_buff.height = 0;
          pc_buff.width = 0;
        }
      }

      int index_best = 0;
      double max_weight = 0;
      double weight_sum = 0;
      if (state == LocalSLAM ||
        now > initial_update + phase_pose_adjust + phase_only_mapping)
      {
        if (state == Init)
        {
          // if (robot_name == "robot2")
          // {
          //   ROS_INFO("start local SLAM!!!!");
          //   ROS_INFO_STREAM("now: " << now);
          //   ROS_INFO_STREAM("initial_update: " << initial_update);
          //   ROS_INFO_STREAM("the time limit: " << (initial_update + phase_pose_adjust + phase_only_mapping));
          //   ROS_INFO_STREAM("phase_only_mapping: " << phase_only_mapping.toSec());
          //   exit(-100);
          // }
          // auto enable: call the ROS service to fly.
          bool auto_enable_by_slam;
          if (!pnh.getParam("auto_enable_by_slam", auto_enable_by_slam))
            auto_enable_by_slam = false;
          if (auto_enable_by_slam)
          {
            ros::ServiceClient srv_client
              = nh.serviceClient<std_srvs::SetBool>(
                  "/" + robot_name + "/enable");
            std_srvs::SetBool srv;
            srv.request.data = true;
            srv_client.call(srv);
          }

          state = LocalSLAM;
        }

        if (enable_indivLoc)
        {
          // ===== update on the particles ===== //
          // - individual SLAM: update based on local sensory data.

          // Odometry data
          tf::Transform diff_pose;
          {
            std::lock_guard<std::mutex> lk(odom_mutex);
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
            diff_pose = pose_prev.inverse() * pose_curr;
            pose_prev = pose_curr;
          }

          // initialize weights and errors
          for (int i = 0; i < n_particles; ++i)
          {
            cumul_weights_slam[i] = 0;
            errors[i] = 0;
          }

          // predict PF (use odometory)
          const tf::Vector3 delta_pos = diff_pose.getOrigin();
          const tf::Quaternion delta_rot = diff_pose.getRotation();
          for (auto p: segments[iseg])
          {
            // move the particle
            // call predict with the relative pose.
            p->predict(delta_pos, delta_rot, gen_indivloc);
          }

          // weight PF (use depth cam)
          for (int i = 0; i < n_particles; ++i)
          {
            // if the current time is not far away from the initial time
            if (iseg != 0 && now <= init_segment_time + init_seg_phase)
            {
              // call evaluate with the flag which is set to true.
              cumul_weights_slam[i]
                = segments[iseg][i]->evaluate(range_data, octocloud, true);
            }
            else // otherwise, just call evaluate in the default way.
            {
              // Calculate a probability ranging from 0 to 1.
              cumul_weights_slam[i]
                = segments[iseg][i]->evaluate(range_data, octocloud);
            }

            if (i == 0 || max_weight < cumul_weights_slam[i])
            {
              max_weight = cumul_weights_slam[i];
              index_best = i;
            }
            if (i > 0)
              cumul_weights_slam[i] += cumul_weights_slam[i-1];
          }
          weight_sum = cumul_weights_slam[n_particles-1];
          if (weight_sum != 0)
            max_weight /= weight_sum;
          segments_index_best[iseg] = index_best;

          // if far away from the init position of the segment
          bool do_segment = false;
          tf::Pose best_pose;
          if (enable_segmentation)
          {
            best_pose = segments[iseg][index_best]->getPose();
            tf::Pose pose_in_seg = init_segment_pose.inverse() * best_pose;
            if (pose_in_seg.getOrigin().length() > next_seg_thresh)
              do_segment = true;
          }

          if (do_segment)
          {
            // create a new segment
            std::vector< std::shared_ptr<Particle> > new_seg(
              n_particles, nullptr);
            segments_index_best.push_back(index_best);
            // copy each resampled particle.
            for (int i = 0; i < n_particles; ++i)
            {
              // if individual localization is disabled, just pass i so the
              // resampling is disabled.
              int indx;
              if (enable_indivLoc)
                indx = drawIndex(cumul_weights_slam, gen_indivloc);
              else
                indx = i;

              new_seg[i]
                = std::make_shared<Particle>(
                    segments[iseg][indx],
                    resol, probHit, probMiss, threshMin, threshMax);
            }
            // build a map anyway
            if (octocloud.size() > 0)
            {
              for (int i = 0; i < n_particles; ++i)
                new_seg[i]->update_map(octocloud);
            }
            counts_map_update = 0;
            // add the segment to the list.
            segments.push_back(new_seg);
            // increment iseg
            ++iseg;
            // set the initial pose of the segment to that of the best particle.
            init_segment_pose = best_pose;
            // set the initial time of the segment to the current one.
            init_segment_time = now;
          }
          else if (weight_sum != 0)
          {
            // resample PF (and update map)
            std::vector<int> indx_list(n_particles);
            for (int i = 0; i < n_particles; ++i)
            {
              indx_list[i] = drawIndex(cumul_weights_slam, gen_indivloc);
            }

            std::vector< std::shared_ptr<Particle> > new_generation;
            std::vector<int> indx_unused(n_particles, -1);
            for (int i = 0; i < n_particles; ++i)
            {
              const int prev_indx = indx_unused[indx_list[i]];
              if (prev_indx == -1)
              {
                new_generation.push_back(
                  std::move(segments[iseg][indx_list[i]]));
                indx_unused[indx_list[i]] = i;

                // update the map
                if (counts_map_update >= mapping_interval
                  && octocloud.size() > 0)
                {
                  new_generation[i]->update_map(octocloud);
                }
              }
              else
              {
                new_generation.push_back(
                  std::make_shared<Particle>(*new_generation[prev_indx]));
              }
            }
            // Copy the children to the parents.
            segments[iseg].swap(new_generation);

            // update the map count
            if (counts_map_update >= mapping_interval)
            {
              counts_map_update = 0;
            }
            else
            {
              ++counts_map_update;
            }
          }
          else
          {
            // update the map
            if (counts_map_update >= mapping_interval && octocloud.size() > 0)
            {
              for (auto p: segments[iseg])
              {
                p->update_map(octocloud);
              }
              counts_map_update = 0;
            }
            else
            {
              ++counts_map_update;
            }
          }
        }
      }
      else if (octocloud.size() > 0)
      {
        // mapping only at the beginning
        for (auto p: segments[iseg])
        {
          p->update_map(octocloud);
        }
      }

      // compress maps
      if (counts_compress >= compress_interval)
      {
        for (auto p: segments[iseg])
        {
          p->compress_map();
        }
        counts_compress = 0;
      }
      else
      {
        ++counts_compress;
      }

      double x, y, z;
      x = 0;
      y = 0;
      z = 0;
      for (int i = 0; i < n_particles; ++i)
      {
        tf::Vector3 buff = segments[iseg][i]->getPose().getOrigin();
        x += buff.x()/n_particles;
        y += buff.y()/n_particles;
        z += buff.z()/n_particles;
      }
      tf::Vector3 average_loc(x, y, z);

      // publish data
      if (counts_publish >= publish_interval)
      {
        mav_tunnel_nav::OctomapWithSegId map;
        map.header.frame_id = world_frame_id;
        map.header.stamp = now;
        std::stringstream ss;
        ss << robot_name << "-" << std::setw(3) << std::setfill('0') << iseg;
        map.segid = ss.str();
        if (octomap_msgs::fullMapToMsg(
            *segments[iseg][segments_index_best[iseg]]->getMap(), map.octomap))
          map_pub.publish(map);
        else
          ROS_ERROR("Error serializing OctoMap");

        tf::StampedTransform tf_stamped(
          segments[iseg][segments_index_best[iseg]]->getPose(), now,
          world_frame_id, robot_frame_id);
        tf_broadcaster.sendTransform(tf_stamped);
        counts_publish = 0;

        if (save_traj)
        {
          std::fstream
            fin(traj_filename, std::fstream::out | std::fstream::app);
          if (!fin)
          {
            ROS_ERROR_STREAM("FILE NOT OPEN: " << traj_filename);
          }
          else
          {
            // save ground truth x, y, z
            tf::StampedTransform ground_truth_tf;
            try
            {
              tf_listener->waitForTransform(
                world_frame_id, robot_name + "_groundtruth",
                ros::Time(0), ros::Duration(1));
              tf_listener->lookupTransform(
                world_frame_id, robot_name + "_groundtruth",
                ros::Time(0), ground_truth_tf);
              tf::Vector3 loc = ground_truth_tf.getOrigin();
              fin << loc.x() << " "
                  << loc.y() << " "
                  << loc.z() << " ";
            }
            catch (tf::TransformException& ex)
            {
              ROS_ERROR_STREAM(
                "Transfrom from " << robot_name + "_groundtruth" <<
                " to " << world_frame_id << " is not available yet.");
              fin << "0 0 0 ";
            }
            // save estimated loc x, y, z
            fin << average_loc.x() << " "
                << average_loc.y() << " "
                << average_loc.z() << std::endl;
            fin.close();
          }
        }
      }
      else
      {
        ++counts_publish;
      }

      // if (counts_locdata >= locdata_interval)
      // {
      //   nav_msgs::Odometry locdata;
      //   locdata.header.frame_id = world_frame_id;
      //   locdata.header.stamp = now;
      //   locdata.child_frame_id = robot_frame_id;
      //   locdata.pose.pose.position.x = average_loc.x();
      //   locdata.pose.pose.position.y = average_loc.y();
      //   locdata.pose.pose.position.z = average_loc.z();
      //   locdata.twist.twist.linear.x = 0;
      //   locdata.twist.twist.linear.y = 0;
      //   locdata.twist.twist.linear.z = 0;
      //   odom_reset_pub.publish(locdata);
      //   counts_locdata = 0;
      // }
      // else
      // {
      //   ++counts_locdata;
      // }

      // visualization
      if (counts_visualize_map >= vismap_interval)
      {
        for (int is = 0; is <= iseg; ++is)
        {
          const octomap::OcTree* m
            = segments[is][segments_index_best[is]]->getMap();
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
              clr.a = 1.0;
              double cosR;
              double cosG;
              double cosB;

              if (enable_clr4seg)
              {
                double val = 0.8 * is;
                cosR = std::cos(PI*val);
                cosG = std::cos(PI*(2.0/3.0+val));
                cosB = std::cos(PI*(4.0/3.0+val));
              }

              if (m->isNodeOccupied(*it))
              {
                occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

                if (enable_clr4seg)
                {
                  double brightness = -z/20.0;
                  while (brightness < 0)
                    brightness += 1.0;
                  while (brightness > 1.0)
                    brightness -= 1.0;
                  if (brightness >= 0.5)
                    brightness = (brightness - 0.5)*2.0;
                  else
                    brightness = -(brightness - 0.5)*2.0;
                  brightness = brightness * 0.8 + 0.2;
                  clr.r = (cosR > 0)? cosR * brightness: 0;
                  clr.g = (cosG > 0)? cosG * brightness: 0;
                  clr.b = (cosB > 0)? cosB * brightness: 0;
                }
                else
                {
                  double brightness = (is + 1.0)/(iseg + 1.0);
                  cosR = std::cos(PI*z/10.0)*0.8+0.2;
                  cosG = std::cos(PI*(2.0/3.0+z/10.0))*0.8+0.2;
                  cosB = std::cos(PI*(4.0/3.0+z/10.0))*0.8+0.2;
                  clr.r = (cosR > 0)? cosR * brightness: 0;
                  clr.g = (cosG > 0)? cosG * brightness: 0;
                  clr.b = (cosB > 0)? cosB * brightness: 0;
                }

                occupiedNodesVis.markers[idx].colors.push_back(clr);
              }
            }
          }
          // std_msgs::ColorRGBA m_color_occupied;
          // m_color_occupied.r = 1;
          // m_color_occupied.g = 1;
          // m_color_occupied.b = 0.3;
          // m_color_occupied.a = 0.5;
          for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i)
          {
            double size = m->getNodeSize(i);

            occupiedNodesVis.markers[i].header.frame_id = "world";
            occupiedNodesVis.markers[i].header.stamp = now;
            occupiedNodesVis.markers[i].ns
              = robot_name + "-" + std::to_string(is);
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
          marker_occupied_pub.publish(occupiedNodesVis);
        }
        counts_visualize_map = 0;
      }
      else
      {
        ++counts_visualize_map;
      }

      if (counts_visualize_loc >= visloc_interval)
      {
        // publish poses
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = world_frame_id;
        poseArray.header.stamp = now;
        poseArray.poses.resize(n_particles);
        for (int i = 0; i < n_particles; ++i)
        {
          tf::Pose pose = segments[iseg][i]->getPose();
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
  }
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rbpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // get this robot's name
  std::string robot_name;
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
  // subscriber for synchronization of exchange
  std::string sync_down_topic;
  ros::Subscriber sync_sub;
  if (nh.getParam("/sync_down_topic", sync_down_topic)) // global param
  {
    sync_sub = nh.subscribe(sync_down_topic, 1000, syncCallback);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: sync_down_topic");
  }
  // subscriber for data exchange
  std::string data_down_topic;
  ros::Subscriber data_sub;
  if (nh.getParam("/data_down_topic", data_down_topic)) // global param
  {
    data_sub = nh.subscribe(data_down_topic, 1000, dataCallback);
  }
  else
  {
    ROS_ERROR_STREAM("no parameter: data_down_topic");
  }

  pnh.getParam("odom_topic", odom_topic);
  pnh.getParam("pc_topic", pc_topic);
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 1000, odomCallback);
  ros::Subscriber pc_sub = nh.subscribe(pc_topic, 1000, pcCallback);

  pnh.getParam("range_max", range_max);
  pnh.getParam("range_min", range_min);
  std::string str_buff;
  pnh.param<std::string>("range_list", str_buff, "");
  std::stringstream ss(str_buff);
  std::string token;
  double x,y,z,P,R,Y;
  std::vector<ros::Subscriber> range_subs;
  while (ss >> token >> x >> y >> z >> R >> P >> Y)
  {
    range_subs.push_back(
      nh.subscribe(token, 1000, rangeCallback)
    );
    range_topics.push_back(token);
    tf::Vector3 pos(x, y, z);
    tf::Quaternion rot;
    rot.setRPY(R * PI / 180.0, P * PI / 180.0, Y * PI / 180.0);
    tf::Pose range_pose(rot, pos);
    range_poses[token] = range_pose;
  }

  std::thread t(pf_main);

  ros::spin();

  t.join();

  return(0);
}
