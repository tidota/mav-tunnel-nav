// particle.cpp
// 210711
// particle class

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "particle.h"

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
void Particle::setMap(const octomap::OcTree* m)
{
  if (this->map)
    delete this->map;
  this->map = new octomap::OcTree(*m);
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
  std::mt19937 &gen, double ratio)
{
  std::normal_distribution<> motion_noise_lin(0, motion_noise_lin_sigma);
  double vel = (ratio != 0)? delta_pos.length() / ratio: 0;
  tf::Vector3 delta_pos_noise(
    delta_pos.x() + motion_noise_lin(gen) * vel,
    delta_pos.y() + motion_noise_lin(gen) * vel,
    delta_pos.z() + motion_noise_lin(gen) * vel);
  std::normal_distribution<> motion_noise_rot(0, motion_noise_rot_sigma);
  tf::Quaternion delta_rot_noise(
    tf::Vector3(0,0,1), motion_noise_rot(gen) * vel);
  this->pose
    = this->pose
      * tf::Transform(delta_rot, delta_pos_noise)
      * tf::Transform(delta_rot_noise, tf::Vector3(0, 0, 0));
}

////////////////////////////////////////////////////////////////////////////////
double Particle::evaluate(
  const std::map<std::string, double> &range_data,
  const double& range_min, const double& range_max,
  const std::vector<std::string>& range_topics,
  const std::map<std::string, tf::Pose>& range_poses,
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
    tf::Vector3 tf_pos = range_poses.at(range_name).getOrigin();
    tf::Vector3 tf_sens(dist, 0, 0);
    tf::Vector3 tf_target = range_poses.at(range_name) * tf_sens;

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

  tf::Pose sens_pose = this->pose;
  // {
  //   // NOTE: if the pose seems in an unknown area, just reutrn 0.
  //   tf::Vector3 tf_sens = tf::Vector3(0.3, 0, 0);
  //   tf::Vector3 tf_target = sens_pose * tf_sens;
  //   octomap::point3d query
  //     = octomap::point3d(tf_target.getX(), tf_target.getY(), tf_target.getZ());
  //   octomap::OcTreeNode* res2check = map2use->search(query);
  //   if (!res2check)
  //   {
  //     return 0;
  //   }
  // }
  double log_lik = 0;
  int hits = 0;
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
        // if (err > 0.2)
        {

          log_lik +=
            -std::log(
              2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0
            -err*err/sensor_noise_depth_sigma/sensor_noise_depth_sigma/2.0;
        }
        // else
        // {
        //   log_lik +=
        //     -std::log(
        //       2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0;
        // }
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
      //if (err > 0.2)
      {

        log_lik +=
          -std::log(
            2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0
          -err*err/sensor_noise_depth_sigma/sensor_noise_depth_sigma/2.0;
      }
      // else
      // {
      //   log_lik +=
      //     -std::log(
      //       2*3.14159*sensor_noise_depth_sigma*sensor_noise_depth_sigma)/2.0;
      // }
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
