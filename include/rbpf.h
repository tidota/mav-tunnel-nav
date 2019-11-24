#ifndef __RBPF_H__
#define __RBPF_H__

#include <memory>
#include <random>

#include <octomap/octomap.h>
#include <tf/transform_datatypes.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Particle
{
  // pose
  private: tf::Pose pose;
  // velocities
  private: tf::Vector3 vel_linear;
  // angular rate
  private: tf::Vector3 vel_angle;
  // map
  private: octomap::OcTree *map;

  public: Particle(const double &resol,
    const double &probHit, const double &probMiss,
    const double &threshMin, const double &threshMax);
  public: Particle(const Particle &src);
  public: Particle();
  public: ~Particle();
  public: void predict(
    const tf::Vector3 &lin, const tf::Vector3 &ang,
    const double &deltaT, std::mt19937 &gen);
  public: double evaluate(const octomap::Pointcloud &scan);
  public: void update_map(const octomap::Pointcloud &scan);
};

#endif
