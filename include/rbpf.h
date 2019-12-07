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
  public: const tf::Pose getPose();
  // velocities
  private: tf::Vector3 vel_linear;
  // map
  private: octomap::OcTree *map;
  public: const octomap::OcTree* getMap();

  public: Particle(const double &resol,
    const double &probHit, const double &probMiss,
    const double &threshMin, const double &threshMax);
  public: Particle(const Particle &src);
  public: Particle();
  public: ~Particle();
  public: void predict(
    const tf::Vector3 &vel, const tf::Quaternion &ori,
    const double &deltaT, std::mt19937 &gen);
  public: double evaluate(const octomap::Pointcloud &scan);
  public: void update_map(const octomap::Pointcloud &scan);
  public: void compress_map();
};

#endif
